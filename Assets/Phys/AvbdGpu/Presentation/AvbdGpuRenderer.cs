using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu.Presentation
{
    /// <summary>Draws an <see cref="AvbdGpuWorld"/> without any readback: instanced draws reading the solver's pose buffers (the
    /// collision box of every body, or an arbitrary mesh for a range of bodies, see <see cref="MeshRanges"/>), plus optional
    /// contact crosses and joint / spring lines written by small compute kernels.</summary>
    public sealed class AvbdGpuRenderer : IDisposable
    {
        /// <summary>Palette: tints and hash colours; GraphColor: the solver's colouring; Uniform: one colour; Sleep: sleeping bodies
        /// in a dim colour hashed from their island, awake ones in the palette.</summary>
        public enum ColorModes { Palette = 0, GraphColor = 1, Uniform = 2, Sleep = 3 }
        public const int ColorModeCount = 4;

        /// <summary>Bodies [Start, Start + Count) are drawn with Mesh (scaled by Scale, shifted by Offset in body space) instead of their box.</summary>
        public struct MeshRange
        {
            public Mesh Mesh;
            public float Scale;
            public float3 Offset;
            public int Start, Count;
            /// <summary>Draw with the world-sized bounds instead of the range's own (<see cref="RangeCulling"/>): for a pool whose spawns
            /// may appear anywhere before the bounds read back catch up with them.</summary>
            public bool NoCulling;
        }

        struct BoundsRequest
        {
            public AsyncGPUReadbackRequest Request;
            public (int start, int count)[] Ranges;
            public int Generation;
        }

        readonly AvbdGpuWorld m_World;
        readonly Mesh m_Cube;
        readonly Material m_BoxMaterial;
        readonly Material m_LineMaterial;
        readonly ComputeShader m_Debug;
        readonly int m_DebugArgs, m_DebugContacts, m_DebugJoints, m_RangeBoundsKernel;
        readonly GraphicsBuffer m_DebugVerts, m_DebugJointVerts, m_DrawArgs, m_Tints;
        GraphicsBuffer m_RangeList, m_RangeBounds;
        uint2[] m_RangeArray = new uint2[64];
        readonly List<(int start, int count)> m_BoundsRanges = new List<(int, int)>();
        readonly Queue<BoundsRequest> m_BoundsRequests = new Queue<BoundsRequest>();
        readonly Dictionary<(int start, int count), Bounds> m_KnownBounds = new Dictionary<(int, int), Bounds>();
        int m_BoundsGeneration;
        readonly MaterialPropertyBlock m_ContactProps = new MaterialPropertyBlock();
        readonly MaterialPropertyBlock m_JointProps = new MaterialPropertyBlock();
        readonly List<MaterialPropertyBlock> m_DrawProps = new List<MaterialPropertyBlock>();
        readonly List<(int start, int count)> m_BoxRanges = new List<(int, int)>();
        readonly List<MeshRange> m_SortedRanges = new List<MeshRange>();
        readonly int m_ContactCap;

        public ColorModes ColorMode = ColorModes.Palette;
        public bool DrawContacts;
        public bool DrawJoints = true;
        /// <summary>Draw the collision boxes of the bodies covered by <see cref="MeshRanges"/> as well (debug).</summary>
        public bool DrawCollisionBoxes;
        /// <summary>Cast and receive main-light shadows (the box and mesh draws).</summary>
        public bool Shadows;
        /// <summary>Cast the shadows of the mesh ranges from the bodies' collision boxes (twelve triangles each) instead of the meshes:
        /// a shadow map draws every body once per cascade, and a brick's studs do not show in its shadow. The meshes still receive.</summary>
        public bool BoxShadows = true;
        /// <summary>Draw calls issued by the last <see cref="Render"/> for the bodies (box ranges, mesh ranges and their shadow proxies).</summary>
        public int LastDraws { get; private set; }
        /// <summary>Draw each mesh range with its own bounds, computed on the GPU from the bodies' poses every frame and read back a few
        /// frames later (<see cref="BoundsMargin"/> covers what moved meanwhile), so that Unity culls the ranges outside the camera's
        /// frustum and outside each shadow cascade; until a range's bounds are known it is drawn with world-sized bounds.</summary>
        public bool RangeCulling = true;
        /// <summary>Metres added around a range's bounds on every side: what its bodies can move in the frames the read back takes.</summary>
        public float BoundsMargin = 5f;
        /// <summary>Mesh-range draws of the last <see cref="Render"/> that used their own bounds (of <see cref="LastDraws"/>).</summary>
        public int BoundedDraws { get; private set; }
        const int MaxBoundsRequests = 4;
        public float ContactCrossSize = 0.06f;
        public int Layer;
        /// <summary>Body ranges drawn with a mesh instead of the collision box. Ranges must not overlap.</summary>
        public readonly List<MeshRange> MeshRanges = new List<MeshRange>();

        static readonly int s_BodyPos = Shader.PropertyToID("_BodyPos");
        static readonly int s_BodyRot = Shader.PropertyToID("_BodyRot");
        static readonly int s_BodyDef = Shader.PropertyToID("_BodyDef");
        static readonly int s_BodyColor = Shader.PropertyToID("_BodyColor");
        static readonly int s_BodyTint = Shader.PropertyToID("_BodyTint");
        static readonly int s_BodySleep = Shader.PropertyToID("_BodySleep");
        static readonly int s_BodyLabel = Shader.PropertyToID("_BodyLabel");
        static readonly int s_ColorMode = Shader.PropertyToID("_ColorMode");
        static readonly int s_InstanceOffset = Shader.PropertyToID("_InstanceOffset");
        static readonly int s_MeshScale = Shader.PropertyToID("_MeshScale");
        static readonly int s_MeshOffset = Shader.PropertyToID("_MeshOffset");
        static readonly int s_DebugVerts = Shader.PropertyToID("_DebugVerts");

        public AvbdGpuRenderer(AvbdGpuWorld world, int debugContactCap = 262144)
        {
            m_World = world;
            m_Cube = Resources.GetBuiltinResource<Mesh>("Cube.fbx");
            var boxShader = Resources.Load<Shader>("AvbdGpu/AvbdBox");
            var lineShader = Resources.Load<Shader>("AvbdGpu/AvbdLines");
            if (boxShader == null || lineShader == null) throw new InvalidOperationException("AvbdGpu shaders not found in Resources/AvbdGpu");
            m_BoxMaterial = new Material(boxShader) { name = "AvbdBox" };
            m_LineMaterial = new Material(lineShader) { name = "AvbdLines" };
            m_Debug = Resources.Load<ComputeShader>("AvbdGpu/AvbdDebug");
            m_DebugArgs = m_Debug.FindKernel("DebugArgs");
            m_DebugContacts = m_Debug.FindKernel("DebugContacts");
            m_DebugJoints = m_Debug.FindKernel("DebugJoints");
            m_RangeBoundsKernel = m_Debug.FindKernel("RangeBounds");
            m_ContactCap = math.min(debugContactCap, world.Config.MaxContacts);
            m_DebugVerts = new GraphicsBuffer(GraphicsBuffer.Target.Structured, m_ContactCap * 6, 32);
            m_DebugJointVerts = new GraphicsBuffer(GraphicsBuffer.Target.Structured, math.max(1, (world.Config.MaxJoints + world.Config.MaxSprings) * 2), 32);
            m_DrawArgs = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments | GraphicsBuffer.Target.Raw, 4, 4);
            m_DrawArgs.SetData(new uint[] { 0, 1, 0, 0 });
            m_Tints = new GraphicsBuffer(GraphicsBuffer.Target.Structured, world.Config.MaxBodies, 4);
            m_Tints.SetData(new uint[world.Config.MaxBodies]);
        }

        public void Dispose()
        {
            m_BoundsRequests.Clear();
            m_RangeList?.Dispose();
            m_RangeBounds?.Dispose();
            m_DebugVerts?.Dispose();
            m_DebugJointVerts?.Dispose();
            m_DrawArgs?.Dispose();
            m_Tints?.Dispose();
            if (m_BoxMaterial != null) UnityEngine.Object.DestroyImmediate(m_BoxMaterial);
            if (m_LineMaterial != null) UnityEngine.Object.DestroyImmediate(m_LineMaterial);
        }

        /// <summary>Per-body colours (RGBA8, alpha 0 = untinted: hash palette) of bodies [start, start + count), indexed by body in
        /// <paramref name="rgba"/>; used in Palette mode.</summary>
        public void SetTints(uint[] rgba, int start, int count)
        {
            if (count <= 0) return;
            m_Tints.SetData(rgba, start, start, count);
        }

        public static uint Tint(Color32 c) => (uint)c.r | ((uint)c.g << 8) | ((uint)c.b << 16) | ((uint)math.max((int)c.a, 1) << 24);

        /// <summary>Clears every tint (scene reset).</summary>
        public void ClearTints() => m_Tints.SetData(new uint[m_Tints.count]);

        /// <summary>The bounds a mesh range [start, start + count) was last drawn with, if its read back has arrived (tests, HUD).</summary>
        public bool TryGetRangeBounds(int start, int count, out Bounds bounds) => m_KnownBounds.TryGetValue((start, count), out bounds);

        /// <summary>Forgets every range's bounds and the read backs in flight (a scene reset: the same start and count may now be
        /// other bodies); the ranges are drawn with world-sized bounds until their new bounds are in.</summary>
        public void ClearRangeBounds()
        {
            m_KnownBounds.Clear();
            m_BoundsGeneration++;
        }

        /// <summary>Takes the range bounds that have arrived, then dispatches this frame's reduction over the mesh ranges and asks for
        /// its read back (a few requests may be in flight; a range is looked up by its start and count, so a range that changed gets
        /// the world bounds until its new result is in).</summary>
        void UpdateRangeBounds(int bodies)
        {
            while (m_BoundsRequests.Count > 0 && m_BoundsRequests.Peek().Request.done)
            {
                var r = m_BoundsRequests.Dequeue();
                if (r.Request.hasError || r.Generation != m_BoundsGeneration) continue;
                var data = r.Request.GetData<float4>();
                for (int i = 0; i < r.Ranges.Length; i++)
                {
                    float3 lo = data[2 * i].xyz, hi = data[2 * i + 1].xyz;
                    if (math.any(lo > hi)) { m_KnownBounds.Remove(r.Ranges[i]); continue; }   // every body of the range is dead
                    var b = new Bounds((lo + hi) * 0.5f, hi - lo);
                    b.Expand(2f * BoundsMargin);
                    m_KnownBounds[r.Ranges[i]] = b;
                }
            }
            if (m_BoundsRequests.Count >= MaxBoundsRequests) return;
            m_BoundsRanges.Clear();
            foreach (var r in MeshRanges)
            {
                int start = math.max(r.Start, 0), count = math.min(r.Start + r.Count, bodies) - start;
                if (r.Mesh == null || count <= 0 || r.NoCulling) continue;
                m_BoundsRanges.Add((start, count));
            }
            int n = m_BoundsRanges.Count;
            if (n == 0) return;
            if (m_RangeList == null || m_RangeList.count < n)
            {
                int cap = math.max(64, math.ceilpow2(n));
                m_RangeList?.Dispose();
                m_RangeBounds?.Dispose();
                m_RangeList = new GraphicsBuffer(GraphicsBuffer.Target.Structured, cap, 8);
                m_RangeBounds = new GraphicsBuffer(GraphicsBuffer.Target.Structured, cap * 2, 16);
                m_RangeArray = new uint2[cap];
            }
            for (int i = 0; i < n; i++) m_RangeArray[i] = new uint2((uint)m_BoundsRanges[i].start, (uint)m_BoundsRanges[i].count);
            m_RangeList.SetData(m_RangeArray, 0, 0, n);
            var b2 = m_World.Buffers;
            m_Debug.SetBuffer(m_RangeBoundsKernel, "_Ranges", m_RangeList);
            m_Debug.SetBuffer(m_RangeBoundsKernel, "_RangeBounds", m_RangeBounds);
            m_Debug.SetBuffer(m_RangeBoundsKernel, "_BodyPos", b2.BodyPos);
            m_Debug.SetBuffer(m_RangeBoundsKernel, "_BodyDef", b2.BodyDef);
            m_Debug.Dispatch(m_RangeBoundsKernel, n, 1, 1);
            m_BoundsRequests.Enqueue(new BoundsRequest { Request = AsyncGPUReadback.Request(m_RangeBounds, n * 2 * 16, 0), Ranges = m_BoundsRanges.ToArray(), Generation = m_BoundsGeneration });
        }

        /// <summary>Issues this frame's draws. Call once per frame (Update / LateUpdate).</summary>
        public void Render(Camera camera = null)
        {
            var b = m_World.Buffers;
            int bodies = m_World.BodyCount;
            var bounds = new Bounds(Vector3.zero, Vector3.one * 100000f);
            if (RangeCulling && bodies > 0) UpdateRangeBounds(bodies);
            else m_KnownBounds.Clear();
            BoundedDraws = 0;

            if (bodies > 0)
            {
                m_BoxMaterial.SetBuffer(s_BodyPos, b.BodyPos);
                m_BoxMaterial.SetBuffer(s_BodyRot, b.BodyRot);
                m_BoxMaterial.SetBuffer(s_BodyDef, b.BodyDef);
                m_BoxMaterial.SetBuffer(s_BodyColor, b.BodyColor);
                m_BoxMaterial.SetBuffer(s_BodyTint, m_Tints);
                m_BoxMaterial.SetBuffer(s_BodySleep, b.BodySleep);
                m_BoxMaterial.SetBuffer(s_BodyLabel, b.BodyLabel);
                m_BoxMaterial.SetFloat(s_ColorMode, (int)ColorMode);
                var rp = new RenderParams(m_BoxMaterial)
                {
                    worldBounds = bounds,
                    shadowCastingMode = Shadows ? ShadowCastingMode.On : ShadowCastingMode.Off,
                    receiveShadows = Shadows,
                    layer = Layer,
                    camera = camera,
                };
                int draw = 0;
                foreach (var (start, count) in BoxRanges(bodies))
                {
                    var props = DrawProps(draw++);
                    props.SetInteger(s_InstanceOffset, start);
                    props.SetFloat(s_MeshScale, 0f);
                    rp.matProps = props;
                    Graphics.RenderMeshPrimitives(rp, m_Cube, 0, count);
                }
                bool proxies = Shadows && BoxShadows;
                foreach (var r in MeshRanges)
                {
                    int start = math.max(r.Start, 0), count = math.min(r.Start + r.Count, bodies) - start;
                    if (r.Mesh == null || count <= 0) continue;
                    var props = DrawProps(draw++);
                    props.SetInteger(s_InstanceOffset, start);
                    props.SetFloat(s_MeshScale, math.max(r.Scale, 1e-6f));
                    props.SetVector(s_MeshOffset, new Vector4(r.Offset.x, r.Offset.y, r.Offset.z, 0f));
                    rp.matProps = props;
                    rp.shadowCastingMode = proxies ? ShadowCastingMode.Off : Shadows ? ShadowCastingMode.On : ShadowCastingMode.Off;
                    rp.worldBounds = bounds;
                    if (RangeCulling && !r.NoCulling && m_KnownBounds.TryGetValue((start, count), out var known)) { rp.worldBounds = known; BoundedDraws++; }
                    Graphics.RenderMeshPrimitives(rp, r.Mesh, 0, count);
                    if (!proxies) continue;
                    // the same bodies as their collision boxes, into the shadow maps only
                    var shadow = DrawProps(draw++);
                    shadow.SetInteger(s_InstanceOffset, start);
                    shadow.SetFloat(s_MeshScale, 0f);
                    rp.matProps = shadow;
                    rp.shadowCastingMode = ShadowCastingMode.ShadowsOnly;
                    Graphics.RenderMeshPrimitives(rp, m_Cube, 0, count);
                }
                LastDraws = draw;
            }

            if (DrawContacts)
            {
                var cs = m_Debug;
                cs.SetBuffer(m_DebugArgs, "_Stats", b.Stats);
                cs.SetBuffer(m_DebugArgs, "_DrawArgs", m_DrawArgs);
                cs.SetInt("_DebugContactCap", m_ContactCap);
                cs.SetFloat("_CrossSize", ContactCrossSize);
                cs.Dispatch(m_DebugArgs, 1, 1, 1);
                cs.SetBuffer(m_DebugContacts, "_Stats", b.Stats);
                cs.SetBuffer(m_DebugContacts, "_BodyPos", b.BodyPos);
                cs.SetBuffer(m_DebugContacts, "_BodyRot", b.BodyRot);
                cs.SetBuffer(m_DebugContacts, "_ManifoldPrev", m_World.LatestManifolds);
                cs.SetBuffer(m_DebugContacts, "_ContactsPrev", m_World.LatestContacts);
                cs.SetBuffer(m_DebugContacts, "_DebugVerts", m_DebugVerts);
                cs.DispatchIndirect(m_DebugContacts, b.DispatchArgs, 2 * 12);   // ARG_MANIFOLDS
                m_ContactProps.SetBuffer(s_DebugVerts, m_DebugVerts);
                var lp = new RenderParams(m_LineMaterial) { worldBounds = bounds, layer = Layer, camera = camera, shadowCastingMode = ShadowCastingMode.Off, receiveShadows = false, matProps = m_ContactProps };
                Graphics.RenderPrimitivesIndirect(lp, MeshTopology.Lines, m_DrawArgs);
            }

            int links = m_World.JointCount + m_World.SpringCount;
            if (DrawJoints && links > 0)
            {
                var cs = m_Debug;
                cs.SetBuffer(m_DebugJoints, "_BodyPos", b.BodyPos);
                cs.SetBuffer(m_DebugJoints, "_BodyRot", b.BodyRot);
                cs.SetBuffer(m_DebugJoints, "_JointDef", b.JointDef);
                cs.SetBuffer(m_DebugJoints, "_JointState", b.JointState);
                cs.SetBuffer(m_DebugJoints, "_SpringDef", b.SpringDef);
                cs.SetBuffer(m_DebugJoints, "_DebugJointVerts", m_DebugJointVerts);
                cs.SetConstantBuffer("AvbdParams", b.Params, 0, GpuParams.Stride);
                cs.Dispatch(m_DebugJoints, (links + 63) / 64, 1, 1);
                m_JointProps.SetBuffer(s_DebugVerts, m_DebugJointVerts);
                var lp = new RenderParams(m_LineMaterial) { worldBounds = bounds, layer = Layer, camera = camera, shadowCastingMode = ShadowCastingMode.Off, receiveShadows = false, matProps = m_JointProps };
                Graphics.RenderPrimitives(lp, MeshTopology.Lines, links * 2);
            }
        }

        MaterialPropertyBlock DrawProps(int i)
        {
            while (m_DrawProps.Count <= i) m_DrawProps.Add(new MaterialPropertyBlock());
            return m_DrawProps[i];
        }

        /// <summary>The body ranges drawn as boxes: everything not covered by a mesh range (or everything with <see cref="DrawCollisionBoxes"/>).</summary>
        List<(int start, int count)> BoxRanges(int bodies)
        {
            m_BoxRanges.Clear();
            if (DrawCollisionBoxes || MeshRanges.Count == 0) { m_BoxRanges.Add((0, bodies)); return m_BoxRanges; }
            m_SortedRanges.Clear();
            m_SortedRanges.AddRange(MeshRanges);
            m_SortedRanges.Sort((x, y) => x.Start.CompareTo(y.Start));
            int cursor = 0;
            foreach (var r in m_SortedRanges)
            {
                int start = math.max(r.Start, cursor), end = math.min(r.Start + r.Count, bodies);
                if (start > cursor) m_BoxRanges.Add((cursor, start - cursor));
                cursor = math.max(cursor, end);
            }
            if (cursor < bodies) m_BoxRanges.Add((cursor, bodies - cursor));
            return m_BoxRanges;
        }
    }
}
