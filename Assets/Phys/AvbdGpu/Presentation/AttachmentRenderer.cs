using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu.Presentation
{
    /// <summary>Meshes drawn on top of the bodies: attached to a body (its pose read straight from the solver buffers by the
    /// vertex shader, so a weapon follows the hand without a frame of lag) or free in world space, opaque (lit, casting shadows)
    /// or transparent (queue Transparent, no depth write, two-sided). The caller rebuilds the list every frame: <see cref="Begin"/>,
    /// <see cref="Add"/> per instance, <see cref="Render"/>; one instanced draw per mesh and material (AvbdAttach.shader).</summary>
    public sealed class AttachmentRenderer : IDisposable
    {
        /// <summary>One drawn mesh (AvbdAttachInstancing.hlsl Attachment).</summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct Instance
        {
            /// <summary>In the parent's frame: body space, or the world for <see cref="WorldSpace"/>.</summary>
            public float3 Position;
            /// <summary>0: lit like the bodies; 1: the flat colour (a glowing spell).</summary>
            public float Unlit;
            public float4 Rotation;
            public float3 Scale;
            /// <summary>The parent body, or <see cref="WorldSpace"/>.</summary>
            public uint Body;
            /// <summary>RGBA8; the alpha is the opacity of a transparent instance.</summary>
            public uint Color;
            public uint Pad0, Pad1, Pad2;
            public const int Stride = 64;
        }

        public const uint WorldSpace = 0xFFFFFFFFu;

        sealed class Batch
        {
            public readonly List<Instance> Items = new List<Instance>();
            public GraphicsBuffer Buffer;
            public Instance[] Upload = new Instance[16];
            public readonly MaterialPropertyBlock Props = new MaterialPropertyBlock();   // one per batch: the draws of a frame keep their own buffer
        }

        readonly AvbdGpuWorld m_World;
        readonly Material m_Opaque, m_Transparent;
        readonly Material[] m_Materials;
        readonly Dictionary<(Mesh mesh, bool transparent), Batch> m_Batches = new Dictionary<(Mesh, bool), Batch>();
        readonly List<(Mesh mesh, bool transparent)> m_Order = new List<(Mesh, bool)>();
        static readonly int s_Attachments = Shader.PropertyToID("_Attachments");
        static readonly int s_BodyPos = Shader.PropertyToID("_BodyPos"), s_BodyRot = Shader.PropertyToID("_BodyRot"), s_BodyDef = Shader.PropertyToID("_BodyDef");

        /// <summary>Opaque instances cast and receive shadows.</summary>
        public bool Shadows = true;
        public int Layer;
        public int LastDraws { get; private set; }
        public int LastInstances { get; private set; }

        public AttachmentRenderer(AvbdGpuWorld world)
        {
            m_World = world;
            var shader = Resources.Load<Shader>("AvbdGpu/AvbdAttach");
            if (shader == null) throw new InvalidOperationException("AvbdGpu/AvbdAttach shader not found in Resources");
            m_Opaque = new Material(shader) { name = "AvbdAttach" };
            m_Transparent = new Material(shader) { name = "AvbdAttachTransparent", renderQueue = (int)RenderQueue.Transparent };
            m_Transparent.SetFloat("_SrcBlend", (float)BlendMode.SrcAlpha);
            m_Transparent.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
            m_Transparent.SetFloat("_ZWrite", 0f);
            m_Transparent.SetFloat("_Cull", (float)CullMode.Off);
            m_Materials = new[] { m_Opaque, m_Transparent };
        }

        public void Dispose()
        {
            foreach (var b in m_Batches.Values) b.Buffer?.Dispose();
            m_Batches.Clear();
            m_Order.Clear();
            if (m_Opaque != null) UnityEngine.Object.Destroy(m_Opaque);
            if (m_Transparent != null) UnityEngine.Object.Destroy(m_Transparent);
        }

        /// <summary>Starts a frame's list (the buffers are kept).</summary>
        public void Begin()
        {
            foreach (var b in m_Batches.Values) b.Items.Clear();
        }

        public void Add(Mesh mesh, in Instance instance, bool transparent = false)
        {
            if (mesh == null) return;
            var key = (mesh, transparent);
            if (!m_Batches.TryGetValue(key, out var batch))
            {
                batch = new Batch();
                m_Batches.Add(key, batch);
                m_Order.Add(key);
            }
            batch.Items.Add(instance);
        }

        /// <summary>A body-relative instance.</summary>
        public static Instance Attached(int body, float3 position, quaternion rotation, float3 scale, Color32 color, float unlit = 0f) => new Instance
        {
            Body = (uint)body, Position = position, Rotation = rotation.value, Scale = scale, Color = AvbdGpuRenderer.Tint(color), Unlit = unlit,
        };

        /// <summary>A world-space instance.</summary>
        public static Instance Free(float3 position, quaternion rotation, float3 scale, Color32 color, float unlit = 0f) => new Instance
        {
            Body = WorldSpace, Position = position, Rotation = rotation.value, Scale = scale, Color = AvbdGpuRenderer.Tint(color), Unlit = unlit,
        };

        /// <summary>Draws this frame's list (once per frame, after the bodies).</summary>
        public void Render(Camera camera = null)
        {
            LastDraws = 0; LastInstances = 0;
            var b = m_World.Buffers;
            foreach (var mat in m_Materials)
            {
                mat.SetBuffer(s_BodyPos, b.BodyPos);
                mat.SetBuffer(s_BodyRot, b.BodyRot);
                mat.SetBuffer(s_BodyDef, b.BodyDef);
            }
            var bounds = new Bounds(Vector3.zero, Vector3.one * 100000f);
            foreach (var key in m_Order)
            {
                var batch = m_Batches[key];
                int n = batch.Items.Count;
                if (n == 0) continue;
                if (batch.Buffer == null || batch.Buffer.count < n)
                {
                    batch.Buffer?.Dispose();
                    batch.Buffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, math.ceilpow2(math.max(n, 16)), Instance.Stride);
                }
                if (batch.Upload.Length < n) batch.Upload = new Instance[math.ceilpow2(n)];
                batch.Items.CopyTo(batch.Upload, 0);
                batch.Buffer.SetData(batch.Upload, 0, 0, n);
                batch.Props.SetBuffer(s_Attachments, batch.Buffer);
                bool opaque = !key.transparent;
                var rp = new RenderParams(opaque ? m_Opaque : m_Transparent)
                {
                    worldBounds = bounds,
                    shadowCastingMode = opaque && Shadows ? ShadowCastingMode.On : ShadowCastingMode.Off,
                    receiveShadows = Shadows,
                    layer = Layer,
                    camera = camera,
                    matProps = batch.Props,
                };
                Graphics.RenderMeshPrimitives(rp, key.mesh, 0, n);
                LastDraws++;
                LastInstances += n;
            }
        }

        // ------------------------------------------------------------------------------------------------ meshes

        /// <summary>A rounded unit square in the xz plane (-0.5 .. 0.5) with a thin skirt: the top at y = <paramref name="thickness"/>,
        /// the sides down to y = 0; <paramref name="corner"/> is the corner radius in units of the side.</summary>
        public static Mesh BuildPlate(float corner = 0.2f, float thickness = 0.04f, int cornerSegments = 6)
        {
            corner = math.clamp(corner, 0f, 0.5f);
            var ring = new List<Vector3>();
            int total = 4 * (cornerSegments + 1);
            for (int c = 0; c < 4; c++)
            {
                float a0 = c * 90f;
                float2 centre = c == 0 ? new float2(0.5f - corner, 0.5f - corner) : c == 1 ? new float2(-0.5f + corner, 0.5f - corner) : c == 2 ? new float2(-0.5f + corner, -0.5f + corner) : new float2(0.5f - corner, -0.5f + corner);
                for (int i = 0; i <= cornerSegments; i++)
                {
                    float a = math.radians(a0 + 90f * i / cornerSegments);
                    ring.Add(new Vector3(centre.x + corner * math.cos(a), 0f, centre.y + corner * math.sin(a)));
                }
            }
            var verts = new List<Vector3>();
            var normals = new List<Vector3>();
            var tris = new List<int>();
            // the ring runs counter-clockwise seen from above; Unity front faces are clockwise, so the fan is wound the other way
            verts.Add(new Vector3(0f, thickness, 0f)); normals.Add(Vector3.up);
            for (int i = 0; i < total; i++) { verts.Add(ring[i] + Vector3.up * thickness); normals.Add(Vector3.up); }
            for (int i = 0; i < total; i++)
            {
                int a = 1 + i, b2 = 1 + (i + 1) % total;
                tris.Add(0); tris.Add(b2); tris.Add(a);
            }
            // the skirt
            for (int i = 0; i < total; i++)
            {
                Vector3 p0 = ring[i], p1 = ring[(i + 1) % total];
                Vector3 n = Vector3.Cross(Vector3.up, p1 - p0).normalized;   // outward: the ring runs counter-clockwise seen from above
                int k = verts.Count;
                verts.Add(p0); verts.Add(p1); verts.Add(p1 + Vector3.up * thickness); verts.Add(p0 + Vector3.up * thickness);
                for (int j = 0; j < 4; j++) normals.Add(n);
                tris.Add(k); tris.Add(k + 2); tris.Add(k + 1); tris.Add(k); tris.Add(k + 3); tris.Add(k + 2);
            }
            var mesh = new Mesh { name = "AvbdPlate" };
            mesh.SetVertices(verts);
            mesh.SetNormals(normals);
            mesh.SetTriangles(tris, 0);
            mesh.bounds = new Bounds(new Vector3(0f, thickness * 0.5f, 0f), new Vector3(1f, thickness, 1f));
            return mesh;
        }
    }
}
