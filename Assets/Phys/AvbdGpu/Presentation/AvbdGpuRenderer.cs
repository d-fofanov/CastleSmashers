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
        }

        readonly AvbdGpuWorld m_World;
        readonly Mesh m_Cube;
        readonly Material m_BoxMaterial;
        readonly Material m_LineMaterial;
        readonly ComputeShader m_Debug;
        readonly int m_DebugArgs, m_DebugContacts, m_DebugJoints;
        readonly GraphicsBuffer m_DebugVerts, m_DebugJointVerts, m_DrawArgs, m_Tints;
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

        /// <summary>Issues this frame's draws. Call once per frame (Update / LateUpdate).</summary>
        public void Render(Camera camera = null)
        {
            var b = m_World.Buffers;
            int bodies = m_World.BodyCount;
            var bounds = new Bounds(Vector3.zero, Vector3.one * 100000f);

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
                foreach (var r in MeshRanges)
                {
                    int start = math.max(r.Start, 0), count = math.min(r.Start + r.Count, bodies) - start;
                    if (r.Mesh == null || count <= 0) continue;
                    var props = DrawProps(draw++);
                    props.SetInteger(s_InstanceOffset, start);
                    props.SetFloat(s_MeshScale, math.max(r.Scale, 1e-6f));
                    props.SetVector(s_MeshOffset, new Vector4(r.Offset.x, r.Offset.y, r.Offset.z, 0f));
                    rp.matProps = props;
                    Graphics.RenderMeshPrimitives(rp, r.Mesh, 0, count);
                }
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
