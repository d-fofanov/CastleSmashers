using Phys.AvbdGpu;
using Phys.AvbdGpu.Presentation;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Phys.Demo
{
    /// <summary>Selects, builds and drives the catalog scenes on the GPU solver; draws the HUD; mouse drag and box shooting.</summary>
    public class DemoBootstrap : MonoBehaviour
    {
        [Range(0, 16)] public int StartScene = AvbdScenes.Pyramid;
        public bool ShowHud = true;
        /// <summary>Body capacity of the GPU world (the largest catalog scene has 73 811 bodies).</summary>
        public int MaxBodies = 81920;

        AvbdGpuWorld m_World;
        AvbdGpuRenderer m_Renderer;
        DemoCamera m_Camera;
        int m_Scene;
        bool m_Paused;
        bool m_StepOnce;
        int m_DragJoint = -1;
        float m_DragDistance;
        int m_Frames;
        readonly FrameTiming[] m_Timings = new FrameTiming[1];
        float m_GpuMs;

        // Player-build verification: -avbd-scene <n> -avbd-screenshot <file> [-avbd-frames <n>] captures a screenshot after n frames and quits.
        string m_ScreenshotPath;
        int m_ScreenshotFrame = 150;

        public AvbdGpuWorld World => m_World;
        public int Scene => m_Scene;

        void Start()
        {
            m_Camera = FindFirstObjectByType<DemoCamera>();
            Application.runInBackground = true;   // a player launched without focus must keep stepping
            Application.targetFrameRate = -1;
            var args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
            {
                if (args[i] == "-avbd-scene" && int.TryParse(args[i + 1], out int sc)) StartScene = sc;
                if (args[i] == "-avbd-screenshot") m_ScreenshotPath = args[i + 1];
                if (args[i] == "-avbd-frames" && int.TryParse(args[i + 1], out int fr)) m_ScreenshotFrame = fr;
            }
            if (!AvbdGpuKernels.Supported) { Debug.LogError("Compute shaders are not supported on this device"); enabled = false; return; }
            m_World = new AvbdGpuWorld(AvbdGpuConfig.ForBodies(MaxBodies)) { ReadbackPoses = true };
            m_Renderer = new AvbdGpuRenderer(m_World);
            Load(StartScene);
        }

        void OnDestroy()
        {
            m_Renderer?.Dispose();
            m_World?.Dispose();
        }

        public void Load(int index)
        {
            index = math.clamp(index, 0, AvbdScenes.Count - 1);
            m_Scene = index;
            m_DragJoint = -1;
            m_World.BuildScene(index);
            m_World.Upload();
            var info = AvbdScenes.All[index];
            if (m_Camera != null) m_Camera.Frame(info.CameraTarget, info.CameraDistance);
        }

        void Update()
        {
            if (m_World == null) return;
            HandleKeys();
            HandleMouse();
            if (!m_Paused || m_StepOnce)
            {
                m_World.Step();
                m_StepOnce = false;
            }
            if (FrameTimingManager.IsFeatureEnabled())
            {
                FrameTimingManager.CaptureFrameTimings();
                if (FrameTimingManager.GetLatestTimings(1, m_Timings) > 0) m_GpuMs = (float)m_Timings[0].gpuFrameTime;
            }
        }

        void LateUpdate()
        {
            if (m_World == null) return;
            m_Renderer.Render();
            if (m_ScreenshotPath == null) return;
            m_Frames++;
            if (m_Frames == m_ScreenshotFrame) ScreenCapture.CaptureScreenshot(m_ScreenshotPath);
            if (m_Frames == m_ScreenshotFrame + 20) Application.Quit();
        }

        void HandleKeys()
        {
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb == null) return;
            for (int i = 0; i < 10; i++)
            {
                Key key = i == 9 ? Key.Digit0 : (Key)((int)Key.Digit1 + i);
                if (kb[key].wasPressedThisFrame) { Load(i); return; }
            }
            if (kb.rKey.wasPressedThisFrame) { Load(m_Scene); return; }
            if (kb.periodKey.wasPressedThisFrame) { Load((m_Scene + 1) % AvbdScenes.Count); return; }
            if (kb.commaKey.wasPressedThisFrame) { Load((m_Scene + AvbdScenes.Count - 1) % AvbdScenes.Count); return; }
            if (kb.spaceKey.wasPressedThisFrame) m_Paused = !m_Paused;
            if (kb.nKey.wasPressedThisFrame) m_StepOnce = true;
            if (kb.f1Key.wasPressedThisFrame) m_Renderer.DrawContacts = !m_Renderer.DrawContacts;
            if (kb.f2Key.wasPressedThisFrame) m_Renderer.ColorMode = (AvbdGpuRenderer.ColorModes)(((int)m_Renderer.ColorMode + 1) % 3);
            if (kb.f3Key.wasPressedThisFrame) m_World.Params.PostStabilize = !m_World.Params.PostStabilize;
            if (kb.f4Key.wasPressedThisFrame) m_World.Params.RotatedInertia = !m_World.Params.RotatedInertia;
            if (kb.f5Key.wasPressedThisFrame) m_Renderer.DrawJoints = !m_Renderer.DrawJoints;
            if (kb.equalsKey.wasPressedThisFrame || kb.numpadPlusKey.wasPressedThisFrame) m_World.Params.Iterations = math.min(64, m_World.Params.Iterations + 1);
            if (kb.minusKey.wasPressedThisFrame || kb.numpadMinusKey.wasPressedThisFrame) m_World.Params.Iterations = math.max(1, m_World.Params.Iterations - 1);
            if (kb.rightBracketKey.wasPressedThisFrame) m_World.Params.Substeps = math.min(8, m_World.Params.Substeps + 1);
            if (kb.leftBracketKey.wasPressedThisFrame) m_World.Params.Substeps = math.max(1, m_World.Params.Substeps - 1);
            if (kb.hKey.wasPressedThisFrame) ShowHud = !ShowHud;
            if (kb.bKey.wasPressedThisFrame || kb.enterKey.wasPressedThisFrame) Shoot();
            if (kb.gKey.wasPressedThisFrame) m_World.Params.Gravity = math.lengthsq(m_World.Params.Gravity) > 0f ? float3.zero : new float3(0, -10f, 0);
#endif
        }

        void Shoot()
        {
            var cam = Camera.main;
            if (cam == null) return;
            float3 forward = cam.transform.forward;
            float3 pos = (float3)cam.transform.position + forward * 3f;
            m_World.AddBody(new float3(1, 1, 1), 1f, 0.5f, pos, quaternion.identity, forward * 25f);
        }

        void HandleMouse()
        {
#if ENABLE_INPUT_SYSTEM
            var mouse = Mouse.current;
            var cam = Camera.main;
            if (mouse == null || cam == null) return;
            Vector2 mp = mouse.position.ReadValue();
            if (mouse.leftButton.wasPressedThisFrame && m_DragJoint < 0)
            {
                var ray = cam.ScreenPointToRay(mp);
                int body = m_World.Pick(ray.origin, ray.direction, out float3 local, out float dist);
                if (body >= 0)
                {
                    float3 worldHit = (float3)ray.origin + (float3)ray.direction * dist;
                    m_DragDistance = math.max(dist, 0.1f);
                    m_DragJoint = m_World.AddJointIndexed(-1, body, worldHit, local, 5000f, 0f);
                }
            }
            else if (mouse.leftButton.isPressed && m_DragJoint >= 0)
            {
                var ray = cam.ScreenPointToRay(mp);
                m_World.SetJointAnchor(m_DragJoint, (float3)ray.origin + (float3)ray.direction * m_DragDistance);
            }
            else if (!mouse.leftButton.isPressed && m_DragJoint >= 0)
            {
                m_World.RemoveJoint(m_DragJoint);
                m_DragJoint = -1;
            }
#endif
        }

        void OnGUI()
        {
            if (!ShowHud || m_World == null) return;
            var info = AvbdScenes.All[m_Scene];
            var stats = m_World.Stats;
            var p = m_World.Params;
            var box = new GUIStyle(GUI.skin.box) { alignment = TextAnchor.UpperLeft, fontSize = 13, richText = true, padding = new RectOffset(10, 10, 8, 8) };
            string overflow = stats.OverflowFlags != 0 ? $"  <color=#ff5555>capacity overflow {stats.OverflowFlags}</color>" : "";
            string text =
                $"<b>[{m_Scene + 1}] {info.Name}</b>{(m_Paused ? "  <color=#ffcc55>PAUSED</color>" : "")}\n" +
                $"{info.Description}\n\n" +
                $"step submit {stats.LastStepMs:F2} ms (avg {stats.AvgStepMs:F2})  gpu frame {(FrameTimingManager.IsFeatureEnabled() ? $"{m_GpuMs:F2} ms" : "n/a")}  {1f / math.max(Time.smoothDeltaTime, 1e-4f):F0} fps\n" +
                $"bodies {m_World.BodyCount}  joints {m_World.JointCount}  springs {m_World.SpringCount}  pairs {stats.Pairs}  manifolds {stats.Manifolds}  contacts {stats.Contacts}\n" +
                $"colours {stats.ColorsUsed} / active {stats.ActiveColors}  overflow bodies {stats.OverflowBodies}  large bodies {stats.LargeBodies}{overflow}\n" +
                $"dt 1/{math.round(1f / p.Dt)}  substeps {p.Substeps}  iterations {p.Iterations}  alpha {p.Alpha}  beta {p.BetaLin}/{p.BetaAng}  gamma {p.Gamma}  " +
                $"post-stabilise {(p.PostStabilize ? "on" : "off")}  rotated inertia {(p.RotatedInertia ? "on" : "off")}  colour mode {m_Renderer.ColorMode}\n\n" +
                "1-0 scene  , . prev/next scene  R reset  Space pause  N step  F1 contacts  F2 colour mode  F3 post-stabilise  F4 rotated inertia  F5 joints\n" +
                "+/- iterations  [ ] substeps  B/Enter shoot box  G gravity  H hide HUD  LMB drag body  RMB orbit  MMB pan  wheel / Q E zoom  W A S D orbit";
            GUI.Box(new Rect(10, 10, 900, 190), text, box);
        }
    }
}
