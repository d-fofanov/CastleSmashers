using Phys.AvbdGpu;
using Phys.AvbdGpu.Presentation;
using Unity.Mathematics;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Phys.Demo
{
    /// <summary>Shared machinery of the demos: GPU world and renderer lifecycle, scene selection keys, pause / single step, mouse drag
    /// joint, box shooting, player flags and the HUD box. Derived classes build the scenes and write the HUD text.</summary>
    public abstract class DemoBase : MonoBehaviour
    {
        public int StartScene;
        public bool ShowHud = true;
        /// <summary>Body capacity of the GPU world.</summary>
        public int MaxBodies = 81920;

        protected AvbdGpuWorld m_World;
        protected AvbdGpuRenderer m_Renderer;
        protected DemoCamera m_Camera;
        protected int m_Scene;
        protected bool m_Paused;
        bool m_StepOnce;
        int m_DragJoint = -1;
        float m_DragDistance;
        int m_Frames;
        readonly FrameTiming[] m_Timings = new FrameTiming[1];
        protected float m_GpuMs;

        // Player-build verification: -avbd-scene <n> -avbd-screenshot <file> [-avbd-frames <n>] captures a screenshot after n frames and quits;
        // -avbd-bench [-avbd-frames <n>] logs the average frame time of the second half of the run and quits;
        // -avbd-yaw <deg> -avbd-pitch <deg> -avbd-distance <m> override the camera.
        string m_ScreenshotPath;
        int m_ScreenshotFrame = 150;
        bool m_Bench;
        double m_BenchMs; int m_BenchFrames;
        float? m_Yaw, m_Pitch, m_Distance;

        public AvbdGpuWorld World => m_World;
        public AvbdGpuRenderer Renderer => m_Renderer;
        public int Scene => m_Scene;
        public bool Paused { get => m_Paused; set => m_Paused = value; }

        /// <summary>Number of scenes selectable with the digit keys and , . </summary>
        protected abstract int SceneCount { get; }
        protected abstract string SceneName(int index);
        /// <summary>Adds the bodies of scene <paramref name="index"/> to the cleared world (and mesh ranges / tints to the renderer).</summary>
        protected abstract void BuildScene(int index, out float3 cameraTarget, out float cameraDistance);
        protected abstract string HudText();
        protected virtual float HudHeight => 190f;
        protected virtual AvbdGpuConfig CreateConfig() => AvbdGpuConfig.ForBodies(MaxBodies);
        /// <summary>Solver parameters and renderer options, applied once after the world is created.</summary>
        protected virtual void Configure() { }
        /// <summary>Extra keys of the derived demo.</summary>
        protected virtual void HandleSceneKeys() { }
        protected virtual float3 ShotSize => new float3(1, 1, 1);
        protected virtual float ShotDensity => 1f;
        protected virtual float ShotSpeed => 25f;
        protected virtual float ShotDistance => 3f;
        /// <summary>Stiffness of the soft world joint that drags a picked body.</summary>
        protected virtual float DragStiffness => 5000f;
        protected virtual void OnShot(int body) { }
        /// <summary>Steps run right after a scene is built, before it is shown (lets stacked scenes settle their penalties).</summary>
        protected virtual int SettleSteps => 0;

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
                if (args[i] == "-avbd-yaw" && float.TryParse(args[i + 1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float yaw)) m_Yaw = yaw;
                if (args[i] == "-avbd-pitch" && float.TryParse(args[i + 1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float pitch)) m_Pitch = pitch;
                if (args[i] == "-avbd-distance" && float.TryParse(args[i + 1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float dist)) m_Distance = dist;
            }
            for (int i = 0; i < args.Length; i++) if (args[i] == "-avbd-bench") m_Bench = true;
            if (!AvbdGpuKernels.Supported) { Debug.LogError("Compute shaders are not supported on this device"); enabled = false; return; }
            m_World = new AvbdGpuWorld(CreateConfig()) { ReadbackPoses = true };
            m_Renderer = new AvbdGpuRenderer(m_World);
            Configure();
            Load(StartScene);
        }

        void OnDestroy()
        {
            m_Renderer?.Dispose();
            m_World?.Dispose();
        }

        public void Load(int index)
        {
            index = math.clamp(index, 0, SceneCount - 1);
            m_Scene = index;
            m_DragJoint = -1;
            m_World.Clear();
            m_Renderer.ClearTints();
            m_Renderer.MeshRanges.Clear();
            BuildScene(index, out float3 target, out float distance);
            m_World.Upload();
            for (int i = 0; i < SettleSteps; i++) m_World.Step();
            if (m_Camera != null)
            {
                if (m_Yaw.HasValue) m_Camera.Yaw = m_Yaw.Value;
                if (m_Pitch.HasValue) m_Camera.Pitch = m_Pitch.Value;
                m_Camera.Frame(target, m_Distance ?? distance);
            }
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
            if (m_ScreenshotPath == null && !m_Bench) return;
            m_Frames++;
            if (m_Bench && m_Frames > m_ScreenshotFrame / 2) { m_BenchMs += Time.unscaledDeltaTime * 1000.0; m_BenchFrames++; }
            if (m_ScreenshotPath != null && m_Frames == m_ScreenshotFrame) ScreenCapture.CaptureScreenshot(m_ScreenshotPath);
            if (m_Frames == m_ScreenshotFrame + 20)
            {
                if (m_Bench)
                {
                    var st = m_World.Stats;
                    Debug.Log($"AVBD bench: scene {m_Scene} '{SceneName(m_Scene)}' bodies {m_World.BodyCount} iterations {m_World.Params.Iterations} substeps {m_World.Params.Substeps}: " +
                        $"frame {m_BenchMs / math.max(m_BenchFrames, 1):F2} ms ({m_BenchFrames / (m_BenchMs / 1000.0):F0} fps), submit {st.AvgStepMs:F2} ms, " +
                        $"pairs {st.Pairs} manifolds {st.Manifolds} contacts {st.Contacts} colours {st.ColorsUsed}/{st.ActiveColors} overflow bodies {st.OverflowBodies} flags {st.OverflowFlags}");
                }
                Application.Quit();
            }
        }

        void HandleKeys()
        {
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb == null) return;
            for (int i = 0; i < 10; i++)
            {
                Key key = i == 9 ? Key.Digit0 : (Key)((int)Key.Digit1 + i);
                if (kb[key].wasPressedThisFrame && i < SceneCount) { Load(i); return; }
            }
            if (kb.rKey.wasPressedThisFrame) { Load(m_Scene); return; }
            if (kb.periodKey.wasPressedThisFrame) { Load((m_Scene + 1) % SceneCount); return; }
            if (kb.commaKey.wasPressedThisFrame) { Load((m_Scene + SceneCount - 1) % SceneCount); return; }
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
            HandleSceneKeys();
#endif
        }

        /// <summary>Shoots a box from the camera along its view direction.</summary>
        public void Shoot()
        {
            var cam = Camera.main;
            if (cam == null) return;
            float3 forward = cam.transform.forward;
            float3 pos = (float3)cam.transform.position + forward * ShotDistance;
            int body = m_World.AddBody(ShotSize, ShotDensity, 0.5f, pos, quaternion.identity, forward * ShotSpeed);
            OnShot(body);
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
                    m_DragJoint = m_World.AddJointIndexed(-1, body, worldHit, local, DragStiffness, 0f);
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

        /// <summary>The line of solver statistics and parameters shared by the HUDs.</summary>
        protected string StatsText()
        {
            var stats = m_World.Stats;
            var p = m_World.Params;
            string overflow = stats.OverflowFlags != 0 ? $"  <color=#ff5555>capacity overflow {stats.OverflowFlags}</color>" : "";
            return
                $"frame {Time.smoothDeltaTime * 1000f:F1} ms ({1f / math.max(Time.smoothDeltaTime, 1e-4f):F0} fps)  step submit {stats.LastStepMs:F2} ms (avg {stats.AvgStepMs:F2})  gpu render {(FrameTimingManager.IsFeatureEnabled() ? $"{m_GpuMs:F2} ms" : "n/a")}\n" +
                $"bodies {m_World.BodyCount}  joints {m_World.JointCount}  springs {m_World.SpringCount}  pairs {stats.Pairs}  manifolds {stats.Manifolds}  contacts {stats.Contacts}\n" +
                $"colours {stats.ColorsUsed} / active {stats.ActiveColors}  overflow bodies {stats.OverflowBodies}  large bodies {stats.LargeBodies}{overflow}\n" +
                $"dt 1/{math.round(1f / p.Dt)}  substeps {p.Substeps}  iterations {p.Iterations}  alpha {p.Alpha}  beta {p.BetaLin}/{p.BetaAng}  gamma {p.Gamma}  " +
                $"post-stabilise {(p.PostStabilize ? "on" : "off")}  rotated inertia {(p.RotatedInertia ? "on" : "off")}  colour mode {m_Renderer.ColorMode}";
        }

        protected string PausedText => m_Paused ? "  <color=#ffcc55>PAUSED</color>" : "";

        void OnGUI()
        {
            if (!ShowHud || m_World == null) return;
            var box = new GUIStyle(GUI.skin.box) { alignment = TextAnchor.UpperLeft, fontSize = 13, richText = true, padding = new RectOffset(10, 10, 8, 8) };
            GUI.Box(new Rect(10, 10, 900, HudHeight), HudText(), box);
        }
    }
}
