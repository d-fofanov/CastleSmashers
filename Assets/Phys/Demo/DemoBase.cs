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
    /// <summary>The procedural terrain of a demo scene (<see cref="Heightfield.Generate"/>); None keeps the flat ground box.</summary>
    [System.Serializable]
    public struct TerrainSettings
    {
        [Tooltip("Hills, a valley or a ridge under the scene; None: the flat ground box. A Terrain assigned to SceneTerrain is used instead of the preset.")]
        public TerrainPreset Preset;
        public uint Seed;
        [Tooltip("Samples per axis (2^n + 1) and metres per sample.")]
        public int Resolution;
        public float Cell;
        [Tooltip("Relief (m) and the size of the features (m).")]
        public float Amplitude;
        public float FeatureSize;
        [Tooltip("Metres beyond the castle footprint over which its plateau blends into the terrain.")]
        public float Skirt;
        [Tooltip("Level ground kept around the castle (m) on top of the few studs the demo adds around the footprint: the hills start beyond it and the skirt.")]
        public float Margin;

        public static TerrainSettings Default => new TerrainSettings { Preset = TerrainPreset.None, Seed = 1, Resolution = 513, Cell = 1f, Amplitude = 12f, FeatureSize = 80f, Skirt = 8f, Margin = 0f };

        /// <summary>What Castle.unity and Preview.unity carry: 513 samples 3 m apart (1.5 km), hills of some 30 m every 140 m or so,
        /// level ground for 60 m around the castle (the armies march on the flat) blending into the hills over 40 m; the castle scene
        /// opens on the hills, the preview scene on the flat ground (T switches either).</summary>
        public static TerrainSettings CastleScene => new TerrainSettings { Preset = TerrainPreset.Hills, Seed = 11, Resolution = 513, Cell = 3f, Amplitude = 30f, FeatureSize = 140f, Skirt = 40f, Margin = 60f };

        /// <summary>Fields a scene was serialised without come out as zero.</summary>
        public TerrainSettings WithDefaults()
        {
            var d = Default;
            var t = this;
            if (t.Resolution < 2) t.Resolution = d.Resolution;
            if (t.Cell <= 0f) t.Cell = d.Cell;
            if (t.Amplitude <= 0f) t.Amplitude = d.Amplitude;
            if (t.FeatureSize <= 0f) t.FeatureSize = d.FeatureSize;
            if (t.Skirt < 0f) t.Skirt = d.Skirt;
            if (t.Margin < 0f) t.Margin = d.Margin;
            return t;
        }
    }

    /// <summary>Shared machinery of the demos: GPU world and renderer lifecycle, scene selection keys, pause / single step, mouse drag
    /// joint, box shooting, player flags and the HUD box. Derived classes build the scenes and write the HUD text.</summary>
    public abstract class DemoBase : MonoBehaviour
    {
        public int StartScene;
        public bool ShowHud = true;
        /// <summary>Body capacity of the GPU world.</summary>
        public int MaxBodies = 81920;
        [Tooltip("A Terrain of the scene: its heights are the world's terrain and it draws the result (its data is cloned, never edited); " +
                 "none: the procedural terrain of TerrainParams, drawn on a Terrain the demo creates.")]
        public Terrain SceneTerrain;
        public TerrainSettings TerrainParams = TerrainSettings.Default;

        protected AvbdGpuWorld m_World;
        protected AvbdGpuRenderer m_Renderer;
        protected TerrainView m_TerrainView;
        protected DemoCamera m_Camera;
        protected int m_Scene;
        protected bool m_Paused;
        bool m_StepOnce;
        int m_DragJoint = -1, m_DragBody = -1;
        float m_DragDistance;
        int m_Frames;
        readonly FrameTiming[] m_Timings = new FrameTiming[1];
        protected float m_GpuMs;

        // Player-build verification: -avbd-scene <n> -avbd-screenshot <file> [-avbd-frames <n>] captures a screenshot after n frames and quits;
        // -avbd-bench [-avbd-frames <n>] logs the average frame time of the second half of the run and quits;
        // -avbd-yaw <deg> -avbd-pitch <deg> -avbd-distance <m> override the camera; -avbd-shoot <n> fires a box at frame n.
        string m_ScreenshotPath;
        int m_ScreenshotFrame = 150;
        int m_ShootFrame = -1;
        bool m_Bench;
        double m_BenchMs; int m_BenchFrames;
        float? m_Yaw, m_Pitch, m_Distance;

        public AvbdGpuWorld World => m_World;
        public AvbdGpuRenderer Renderer => m_Renderer;
        public TerrainView TerrainView => m_TerrainView;
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
        /// <summary>Called before every world step that the frame loop runs (not paused, or single-stepping).</summary>
        protected virtual void OnStep() { }
        /// <summary>Steps run right after a scene is built, before it is shown (lets stacked scenes settle their penalties).</summary>
        protected virtual int SettleSteps => 0;
        /// <summary>Substeps used for the settle steps (0 = the current parameter).</summary>
        protected virtual int SettleSubsteps => 0;

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
                if (args[i] == "-avbd-shoot" && int.TryParse(args[i + 1], out int sh)) m_ShootFrame = sh;
                if (args[i] == "-avbd-yaw" && float.TryParse(args[i + 1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float yaw)) m_Yaw = yaw;
                if (args[i] == "-avbd-pitch" && float.TryParse(args[i + 1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float pitch)) m_Pitch = pitch;
                if (args[i] == "-avbd-distance" && float.TryParse(args[i + 1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float dist)) m_Distance = dist;
                if (args[i] == "-avbd-terrain" && TryParseTerrain(args[i + 1], out var preset)) TerrainParams.Preset = preset;
                if (args[i] == "-avbd-terrain-seed" && uint.TryParse(args[i + 1], out uint seed)) TerrainParams.Seed = seed;
            }
            TerrainParams = TerrainParams.WithDefaults();
            for (int i = 0; i < args.Length; i++) if (args[i] == "-avbd-bench") m_Bench = true;
            if (!AvbdGpuKernels.Supported) { Debug.LogError("Compute shaders are not supported on this device"); enabled = false; return; }
            m_World = new AvbdGpuWorld(CreateConfig()) { ReadbackPoses = true };
            m_Renderer = new AvbdGpuRenderer(m_World);
            m_TerrainView = new TerrainView();
            Configure();
            Load(StartScene);
        }

        void OnDestroy()
        {
            m_TerrainView?.Dispose();
            m_Renderer?.Dispose();
            m_World?.Dispose();
        }

        public void Load(int index)
        {
            index = math.clamp(index, 0, SceneCount - 1);
            m_Scene = index;
            m_DragJoint = m_DragBody = -1;
            m_World.Clear();
            m_Renderer.ClearTints();
            m_Renderer.MeshRanges.Clear();
            BuildScene(index, out float3 target, out float distance);
            m_TerrainView.Show(m_World.Terrain, SceneTerrain);   // hides the terrain when the scene has none
            m_World.Upload();
            if (SettleSteps > 0)
            {
                int substeps = m_World.Params.Substeps;
                if (SettleSubsteps > 0) m_World.Params.Substeps = SettleSubsteps;
                for (int i = 0; i < SettleSteps; i++) m_World.Step();
                m_World.Params.Substeps = substeps;
            }
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
                OnStep();
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
            m_Frames++;
            if (m_Frames == m_ShootFrame) Shoot();
            if (m_ScreenshotPath == null && !m_Bench) return;
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

        static bool TryParseTerrain(string s, out TerrainPreset preset)
        {
            if (int.TryParse(s, out int n) && n >= 0 && n <= (int)TerrainPreset.Ridge) { preset = (TerrainPreset)n; return true; }
            return System.Enum.TryParse(s, true, out preset);
        }

        /// <summary>The heightfield of the scene being built, or null for the flat ground: the scene terrain's heights when one is
        /// assigned, otherwise the procedural preset (a fresh field every load, so a plateau can be cut into it).</summary>
        protected Heightfield CreateTerrain()
        {
            if (SceneTerrain != null) return TerrainView.FromTerrainData(m_TerrainView.OriginalData(SceneTerrain), SceneTerrain.transform.position);
            var t = TerrainParams;
            if (t.Preset == TerrainPreset.None) return null;
            return Heightfield.Generate(t.Preset, t.Seed, t.Resolution, t.Cell, t.Amplitude, t.FeatureSize);
        }

        /// <summary>Levels a plateau for a footprint centred on the origin (half extents <paramref name="halfExtent"/> plus
        /// <paramref name="border"/> plus the settings' margin) at the mean terrain height under it, blending over the skirt;
        /// returns the plateau height.</summary>
        protected float CutPlateau(Heightfield field, float2 halfExtent, float border)
        {
            float2 lo = -halfExtent - border - TerrainParams.Margin, hi = halfExtent + border + TerrainParams.Margin;
            float plateau = field.MeanHeight(lo, hi);
            field.Flatten(lo, hi, plateau, TerrainParams.Skirt);
            return plateau;
        }

        /// <summary>The terrain, or the flat ground box the scenes had before it: one or the other under every demo scene.
        /// Returns the ground height at the origin (the plateau).</summary>
        protected float AddGround(Heightfield field, float friction, float2 plateauHalfExtent, float plateauBorder, out int groundBody)
        {
            if (field != null)
            {
                float plateau = CutPlateau(field, plateauHalfExtent, plateauBorder);
                groundBody = m_World.SetTerrain(field, friction);
                return plateau;
            }
            const float ground = 2000f;                             // out to the horizon
            groundBody = m_World.AddBody(new float3(ground, 1f, ground), 0f, friction, new float3(0f, -0.5f, 0f), quaternion.identity, float3.zero);
            return 0f;
        }

        /// <summary>The terrain line of the HUD.</summary>
        protected string TerrainText(float plateau)
        {
            var field = m_World.Terrain;
            if (field == null) return "flat ground (T: terrain)";
            string source = SceneTerrain != null ? $"scene terrain {SceneTerrain.name}" : $"{TerrainParams.Preset.ToString().ToLowerInvariant()} (seed {TerrainParams.Seed})";
            return $"<color=#a8d878>terrain</color> {source}: {field.ResX} x {field.ResZ} samples {field.Cell.x:G3} m apart, heights {field.MinHeight:F1} .. {field.MaxHeight:F1} m, plateau at {plateau:F1} m";
        }

        /// <summary>Next procedural preset (T key): None, Hills, Valley, Ridge, None ...; a scene terrain is left alone.</summary>
        protected void CycleTerrain()
        {
            if (SceneTerrain != null) return;
            TerrainParams.Preset = (TerrainPreset)(((int)TerrainParams.Preset + 1) % ((int)TerrainPreset.Ridge + 1));
            Load(m_Scene);
        }

        /// <summary>Shoots a box from the camera along its view direction.</summary>
        public virtual void Shoot()
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
                    m_DragBody = body;
                }
            }
            else if (mouse.leftButton.isPressed && m_DragJoint >= 0)
            {
                var ray = cam.ScreenPointToRay(mp);
                m_World.SetJointAnchor(m_DragJoint, (float3)ray.origin + (float3)ray.direction * m_DragDistance);
            }
            else if (!mouse.leftButton.isPressed && m_DragJoint >= 0)
                ReleaseDrag();
#endif
        }

        /// <summary>The body held by the mouse drag joint, or -1.</summary>
        protected int DragBody => m_DragJoint >= 0 ? m_DragBody : -1;

        /// <summary>Removes the mouse drag joint (also needed before its body is retired).</summary>
        protected void ReleaseDrag()
        {
            if (m_DragJoint < 0) return;
            m_World.RemoveJoint(m_DragJoint);
            m_DragJoint = m_DragBody = -1;
        }

        /// <summary>The line of solver statistics and parameters shared by the HUDs.</summary>
        protected string StatsText()
        {
            var stats = m_World.Stats;
            var p = m_World.Params;
            string overflow = stats.OverflowFlags != 0 ? $"  <color=#ff5555>capacity overflow {stats.OverflowFlags}</color>" : "";
            return
                $"frame {Time.smoothDeltaTime * 1000f:F1} ms ({1f / math.max(Time.smoothDeltaTime, 1e-4f):F0} fps)  step submit {stats.LastStepMs:F2} ms (avg {stats.AvgStepMs:F2})  gpu render {(FrameTimingManager.IsFeatureEnabled() ? $"{m_GpuMs:F2} ms" : "n/a")}\n" +
                $"bodies {m_World.BodyCount}  joints {m_World.JointCount}  springs {m_World.SpringCount}  pairs {stats.Pairs}  manifolds {stats.Manifolds}" +
                (m_World.Terrain != null ? $" ({stats.TerrainManifolds} terrain)" : "") + $"  contacts {stats.Contacts}\n" +
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
