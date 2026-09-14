using Phys.AvbdGpu;
using Phys.AvbdGpu.Presentation;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdGpu.Siege;
using Unity.Mathematics;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Phys.Demo
{
    /// <summary>A castle of construction bricks (Assets/Models/ConstructorBlock2x3) on the GPU solver (Castle.unity): every brick is a
    /// box body drawn with the brick model. Bricks are either dry-stacked (friction only) or snapped together with breakable joints.
    /// A siege (key U) surrounds it with figures that march in and fire volleys at the garrison and the walls.</summary>
    public class CastleDemo : DemoBase
    {
        [Tooltip("The brick model (Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx); the collision boxes are drawn when unset.")]
        public Mesh BrickMesh;
        [Tooltip("The figure model (Assets/Models/ConstructorFigure/ConstructorFigure.fbx) drawn for the units.")]
        public Mesh FigureMesh;
        [Tooltip("The arrow model (Assets/Models/ConstructorArrow/ConstructorArrow.fbx) drawn for arrows and rockets.")]
        public Mesh ArrowMesh;
        [Tooltip("Solver metres per model metre. 1 simulates the true 0.2 x 0.12 x 0.3 m brick; the default 5 puts the bricks in the " +
                 "metre / kilogram regime the solver's penalty ramp is tuned for (and slows the motion accordingly).")]
        public float BrickScale = 5f;
        [Tooltip("Mass of one brick (kg). Light enough that the snapped gatehouse, bending over its passage, stays below the snap limits.")]
        public float BrickMass = 0.25f;
        public float BrickFriction = 0.6f;
        [Tooltip("Snap the bricks together with hard ball-socket joints that break under load.")]
        public bool Snap;
        [Tooltip("Sideways force (N) that breaks a snap connection (a brick weighs BrickMass x 10 N).")]
        public float SnapFractureLateral = 300f;
        [Tooltip("Upward pull (N) that breaks a snap connection; a snap also comes apart once the bricks separate by half the stud height.")]
        public float SnapFractureTension = 50f;
        public bool Shadows = true;
        [Tooltip("Cannonball: cube size (model metres), mass (kg) and speed (model metres per second).")]
        public float ShotCube = 0.12f;
        public float ShotMass = 30f;
        public float ShotVelocity = 24f;
        [Tooltip("Spawn the armies when a castle is loaded (also the -avbd-siege flag).")]
        public bool SiegeOnLoad;
        [Tooltip("Body pools reserved for the siege: units, arrow-shaped and cube projectiles.")]
        public int UnitCapacity = 512;
        public int ArrowCapacity = 2048;
        public int ShotCapacity = 512;
        public float UnitMass = 1f;
        public float ArrowMass = 0.02f;
        public SiegeSettings SiegeParams = SiegeSettings.Default;

        BrickLayout m_Layout;
        BrickSpec m_Spec;
        int m_FirstBrick, m_SnapJoints;
        uint[] m_Tints;
        SiegeSystem m_Siege;
        bool m_SiegeActive;
        static readonly Color32 s_Iron = new Color32(70, 72, 78, 255);

        void Reset()
        {
            StartScene = 3;
            MaxBodies = 40960;   // the largest preset has 34 925 bricks
        }

        public BrickLayout Layout => m_Layout;
        public int FirstBrick => m_FirstBrick;
        public int BrickCount => m_Layout?.Bricks.Count ?? 0;
        public BrickSpec Spec => m_Spec;
        public SiegeSystem Siege => m_Siege;
        public bool SiegeActive => m_SiegeActive;

        protected override int SceneCount => CastlePlan.Presets.Length;
        protected override string SceneName(int index) => CastlePlan.Presets[index].Name;
        protected override float HudHeight => 250f;
        protected override float3 ShotSize => ShotCube * BrickScale;
        protected override float ShotDensity => ShotMass / math.pow(ShotCube * BrickScale, 3f);
        /// <summary>Speeds scale with the square root of lengths under the same gravity (dynamic similarity).</summary>
        protected override float ShotSpeed => ShotVelocity * math.sqrt(BrickScale);
        protected override float ShotDistance => 0.5f * BrickScale;
        /// <summary>Same frequency as the reference's 5000 N/m on a 1 kg box.</summary>
        protected override float DragStiffness => 5000f * m_Spec.BrickMass;
        /// <summary>The contact penalties ramp up from their minimum over the first steps and the stacks sink a few centimetres
        /// meanwhile; settle that before showing the castle (single substeps: nothing moves fast yet).</summary>
        protected override int SettleSteps => 180;
        protected override int SettleSubsteps => 1;

        protected override AvbdGpuConfig CreateConfig()
        {
            var cfg = AvbdGpuConfig.ForBodies(MaxBodies);
            cfg.MaxJoints = MaxBodies * 12;                   // four snap joints per brick overlap (about ten per brick) plus drag joints
            cfg.MaxLinks = cfg.MaxJoints * 2 + 4096;
            cfg.MaxSpawns = math.max(cfg.MaxSpawns, UnitCapacity + ArrowCapacity + ShotCapacity);
            return cfg;
        }

        protected override void Configure()
        {
            foreach (var arg in System.Environment.GetCommandLineArgs())
            {
                if (arg == "-avbd-snap") Snap = true;
                if (arg == "-avbd-siege") SiegeOnLoad = true;
            }
            SiegeParams = SiegeParams.WithDefaults();   // fields a scene was serialised without come out as zero
            m_Renderer.Shadows = Shadows;
            m_Renderer.DrawJoints = false;
#if UNITY_EDITOR
            if (BrickMesh == null) BrickMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx");
            if (FigureMesh == null) FigureMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorFigure/ConstructorFigure.fbx");
            if (ArrowMesh == null) ArrowMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorArrow/ConstructorArrow.fbx");
#endif
            if (BrickMesh == null) Debug.LogWarning("CastleDemo: no brick mesh assigned, drawing the collision boxes");
            if (FigureMesh == null || ArrowMesh == null) Debug.LogWarning("CastleDemo: figure / arrow mesh not assigned, drawing the collision boxes");
        }

        protected override void BuildScene(int index, out float3 cameraTarget, out float cameraDistance)
        {
            var plan = CastlePlan.Presets[index];
            m_Layout = BrickCastle.Generate(plan);
            float2 c = BrickCastle.Center(plan) * Brick.Pitch * BrickScale;
            float volume = Brick.Width * Brick.BodyHeight * Brick.Length * BrickScale * BrickScale * BrickScale;
            m_Spec = new BrickSpec
            {
                Scale = BrickScale, Density = BrickMass / volume, Friction = BrickFriction, Margin = AvbdGpuConstants.CollisionMargin,
                Origin = new float3(-c.x, 0f, -c.y),
            };
            float ground = 2000f;                             // out to the horizon
            int groundBody = m_World.AddBody(new float3(ground, 1f, ground), 0f, BrickFriction, new float3(0f, -0.5f, 0f), quaternion.identity, float3.zero);
            m_FirstBrick = BrickCastle.Build(m_World, m_Layout, m_Spec);
            m_SnapJoints = Snap ? BrickCastle.AddSnapJoints(m_World, m_Layout, m_FirstBrick, m_Spec, SnapFractureLateral, SnapFractureTension) : 0;

            // colours: the ground, then one tint per brick from its tone with a little per-brick variation
            int n = m_Layout.Bricks.Count;
            if (m_Tints == null || m_Tints.Length < m_FirstBrick + n) m_Tints = new uint[m_FirstBrick + n];
            m_Tints[groundBody] = AvbdGpuRenderer.Tint(new Color32(78, 112, 58, 255));
            var rng = new Unity.Mathematics.Random(0x9E3779B9u);
            for (int i = 0; i < n; i++) m_Tints[m_FirstBrick + i] = AvbdGpuRenderer.Tint(ToneColor(m_Layout.Bricks[i].Tone, rng.NextFloat()));
            m_Renderer.SetTints(m_Tints, 0, m_FirstBrick + n);
            if (BrickMesh != null)
                m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = BrickMesh, Scale = BrickScale, Offset = m_Spec.MeshOffset, Start = m_FirstBrick, Count = n });

            // the siege's body pools (retired slots until an army is spawned), drawn with the figure and arrow models
            var siegeSpec = SiegeSpec.Default;
            siegeSpec.Scale = BrickScale; siegeSpec.Margin = AvbdGpuConstants.CollisionMargin; siegeSpec.Friction = BrickFriction;
            siegeSpec.UnitMass = UnitMass; siegeSpec.ArrowMass = ArrowMass; siegeSpec.BallCube = ShotCube;
            m_Siege = new SiegeSystem(m_World, siegeSpec, UnitCapacity, ArrowCapacity, ShotCapacity, SiegeParams) { OnSpawned = OnSiegeSpawn, OnRetiring = OnSiegeRetire };
            m_SiegeActive = false;
            if (FigureMesh != null)
                m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = FigureMesh, Scale = BrickScale, Offset = siegeSpec.UnitMeshOffset, Start = m_Siege.Units.Start, Count = m_Siege.Units.Capacity });
            if (ArrowMesh != null)
                m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = ArrowMesh, Scale = BrickScale, Offset = siegeSpec.ArrowMeshOffset, Start = m_Siege.Arrows.Start, Count = m_Siege.Arrows.Capacity });
            if (SiegeOnLoad) ToggleSiege();

            // fewer substeps for the big castles: their cannonball crosses a fraction of the wall thickness per step even at one
            m_World.Params.Substeps = n > 20000 ? 1 : n > 8000 ? 2 : 3;
            cameraTarget = new float3(0f, 1.2f * plan.WallCourses * Brick.BodyHeight * BrickScale, 0f);
            cameraDistance = 1.4f * plan.Side * Brick.Pitch * BrickScale;
        }

        /// <summary>Stone in three shades, dark slate battlements, sandstone gatehouse, red keep top, wooden stairs.</summary>
        public static Color32 ToneColor(BrickTone tone, float u)
        {
            switch (tone)
            {
                case BrickTone.Slate: return new Color32(72, 78, 90, 255);
                case BrickTone.Tan: return Color32.Lerp(new Color32(205, 178, 128, 255), new Color32(224, 200, 150, 255), u);
                case BrickTone.Red: return Color32.Lerp(new Color32(168, 44, 36, 255), new Color32(196, 64, 48, 255), u);
                case BrickTone.Wood: return new Color32(134, 96, 58, 255);
                default:
                    int shade = (int)(u * 3f);
                    return shade == 0 ? new Color32(158, 160, 164, 255) : shade == 1 ? new Color32(178, 180, 182, 255) : new Color32(140, 143, 150, 255);
            }
        }

        protected override void OnShot(int body)
        {
            SetTint(body, s_Iron);
        }

        void SetTint(int body, Color32 color)
        {
            if (m_Tints == null || m_Tints.Length <= body) System.Array.Resize(ref m_Tints, math.max(body + 1, 1024));
            m_Tints[body] = AvbdGpuRenderer.Tint(color);
            m_Renderer.SetTints(m_Tints, body, 1);
        }

        void OnSiegeSpawn(int body, Color32 color) => SetTint(body, color);

        void OnSiegeRetire(int body)
        {
            if (DragBody == body) ReleaseDrag();
        }

        /// <summary>The cannonball goes through the siege's shot pool, so it is purged like every other spent projectile.</summary>
        public override void Shoot()
        {
            var cam = Camera.main;
            if (cam == null || m_Siege == null) { base.Shoot(); return; }
            float3 forward = cam.transform.forward;
            m_Siege.Launch(ProjectileKind.Cannonball, SiegeSystem.Attackers, (float3)cam.transform.position + forward * ShotDistance, forward * ShotSpeed, default, ShotMass);
        }

        /// <summary>Spawns the armies around the castle, or clears them.</summary>
        public void ToggleSiege()
        {
            if (m_Siege == null) return;
            if (m_SiegeActive) { ReleaseDrag(); m_Siege.ClearArmies(); }
            else m_Siege.SpawnArmies(m_Layout, CastlePlan.Presets[m_Scene], m_Spec);
            m_SiegeActive = !m_SiegeActive;
        }

        protected override void OnStep()
        {
            if (m_Siege != null && (m_SiegeActive || m_Siege.ProjectileList.Count > 0))
                m_Siege.Tick(m_World.ReadPositions, m_World.ReadCount, m_World.ReadStep);
        }

        protected override void HandleSceneKeys()
        {
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb.jKey.wasPressedThisFrame) { Snap = !Snap; Load(m_Scene); }
            if (kb.f6Key.wasPressedThisFrame) m_Renderer.DrawCollisionBoxes = !m_Renderer.DrawCollisionBoxes;
            if (kb.f7Key.wasPressedThisFrame) { Shadows = !Shadows; m_Renderer.Shadows = Shadows; }
            if (kb.uKey.wasPressedThisFrame) ToggleSiege();
            if (kb.vKey.wasPressedThisFrame && m_Siege != null) m_Siege.Volley(m_World.ReadPositions, m_World.ReadCount, m_World.ReadStep);
            if (kb.kKey.wasPressedThisFrame && m_Siege != null) m_Siege.AutoVolleys = !m_Siege.AutoVolleys;
            if (kb.xKey.wasPressedThisFrame && m_Siege != null) { ReleaseDrag(); m_Siege.Purge(); }
#endif
        }

        protected override string HudText()
        {
            var plan = CastlePlan.Presets[m_Scene];
            float side = plan.Side * Brick.Pitch * BrickScale;
            return
                $"<b>[{m_Scene + 1}] {plan.Name}</b>{PausedText}\n" +
                $"{BrickCount} bricks of {Brick.Width * BrickScale * 100f:F0} x {Brick.BodyHeight * BrickScale * 100f:F0} x {Brick.Length * BrickScale * 100f:F0} cm (model x {BrickScale:G3}), {m_Spec.BrickMass:F2} kg; " +
                $"{side:F1} m square, walls {plan.WallCourses} courses, towers {plan.TowerCourses}, keep {plan.KeepCourses}\n" +
                (Snap ? $"<color=#88ddff>snapped</color>: {m_SnapJoints} joints; a snap breaks at {SnapFractureLateral:F0} N sideways, {SnapFractureTension:F0} N upward or {m_Spec.SnapBreakDistance * 100f:F1} cm apart  -  cannonball {ShotMass:F0} kg\n"
                      : $"dry-stacked (friction only)  -  cannonball {ShotMass:F0} kg\n") +
                (m_SiegeActive ? $"<color=#ffcc88>siege</color> (volleys {(m_Siege.AutoVolleys ? "auto" : "manual")}): {m_Siege.Summary()}\n\n" : "no siege (U)\n\n") +
                StatsText() + "\n\n" +
                "1-0 castle size  , . prev/next  R rebuild  J snap bricks on/off  U siege on/off  V volley  K auto volleys  X retire the dead and spent now  Space pause  N step\n" +
                "F1 contacts  F2 colour mode  F5 joints  F6 collision boxes  F7 shadows  +/- iterations  [ ] substeps  B/Enter cannonball  G gravity  H hide HUD\n" +
                "LMB drag  RMB orbit  MMB pan  wheel / Q E zoom  W A S D orbit";
        }
    }
}
