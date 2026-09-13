using System;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu.Presentation
{
    /// <summary>Draws an <see cref="AvbdGpuWorld"/> without any readback: one instanced cube draw reading the solver's pose
    /// buffers, plus optional contact crosses and joint / spring lines written by small compute kernels.</summary>
    public sealed class AvbdGpuRenderer : IDisposable
    {
        public enum ColorModes { Palette = 0, GraphColor = 1, Uniform = 2 }

        readonly AvbdGpuWorld m_World;
        readonly Mesh m_Cube;
        readonly Material m_BoxMaterial;
        readonly Material m_LineMaterial;
        readonly ComputeShader m_Debug;
        readonly int m_DebugArgs, m_DebugContacts, m_DebugJoints;
        readonly GraphicsBuffer m_DebugVerts, m_DebugJointVerts, m_DrawArgs;
        readonly MaterialPropertyBlock m_ContactProps = new MaterialPropertyBlock();
        readonly MaterialPropertyBlock m_JointProps = new MaterialPropertyBlock();
        readonly int m_ContactCap;

        public ColorModes ColorMode = ColorModes.Palette;
        public bool DrawContacts;
        public bool DrawJoints = true;
        public float ContactCrossSize = 0.06f;
        public int Layer;

        static readonly int s_BodyPos = Shader.PropertyToID("_BodyPos");
        static readonly int s_BodyRot = Shader.PropertyToID("_BodyRot");
        static readonly int s_BodyDef = Shader.PropertyToID("_BodyDef");
        static readonly int s_BodyColor = Shader.PropertyToID("_BodyColor");
        static readonly int s_ColorMode = Shader.PropertyToID("_ColorMode");
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
        }

        public void Dispose()
        {
            m_DebugVerts?.Dispose();
            m_DebugJointVerts?.Dispose();
            m_DrawArgs?.Dispose();
            if (m_BoxMaterial != null) UnityEngine.Object.DestroyImmediate(m_BoxMaterial);
            if (m_LineMaterial != null) UnityEngine.Object.DestroyImmediate(m_LineMaterial);
        }

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
                m_BoxMaterial.SetFloat(s_ColorMode, (int)ColorMode);
                var rp = new RenderParams(m_BoxMaterial)
                {
                    worldBounds = bounds,
                    shadowCastingMode = ShadowCastingMode.Off,
                    receiveShadows = false,
                    layer = Layer,
                    camera = camera,
                };
                Graphics.RenderMeshPrimitives(rp, m_Cube, 0, bodies);
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
                cs.SetBuffer(m_DebugContacts, "_ManifoldPrev", b.ManifoldPrev);
                cs.SetBuffer(m_DebugContacts, "_ContactsPrev", b.ContactsPrev);
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
    }
}
