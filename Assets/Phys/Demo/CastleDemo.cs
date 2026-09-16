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
        [Tooltip("Copies of the castle built asleep around it, up to eight (key O cycles 0 / 4 / 8; -avbd-outlying n): a world with more " +
                 "bricks than it simulates at once. A copy wakes when something hits it and sleeps again once it has settled.")]
        public int Outlying;
        [Tooltip("Bodies reserved for the outlying castles on top of MaxBodies, which stays the number of bricks awake at a time.")]
        public int OutlyingBodies = 131072;
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
        int m_OutlyingCount, m_OutlyingBricks;
        float2[] m_OutlyingCentres = new float2[0];
        float m_Plateau;
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
        /// <summary>Ground height under the castle (the plateau levelled into the terrain; 0 on the flat ground).</summary>
        public float Plateau => m_Plateau;
        /// <summary>The outlying copies built asleep around the castle: how many, their centres, their bricks (contiguous after the
        /// castle's own).</summary>
        public int OutlyingCount => m_OutlyingCount;
        public float2[] OutlyingCentres => m_OutlyingCentres;
        public int OutlyingBricks => m_OutlyingBricks;
        public int FirstOutlyingBrick => m_FirstBrick + BrickCount;

        protected override int SceneCount => CastlePlan.Presets.Length;
        protected override string SceneName(int index) => CastlePlan.Presets[index].Name;
        protected override float HudHeight => 268f;
        protected override float3 ShotSize => ShotCube * BrickScale;
        protected override float ShotDensity => ShotMass / math.pow(ShotCube * BrickScale, 3f);
        /// <summary>Speeds scale with the square root of lengths under the same gravity (dynamic similarity).</summary>
        protected override float ShotSpeed => ShotVelocity * math.sqrt(BrickScale);
        protected override float ShotDistance => 0.5f * BrickScale;
        /// <summary>Same frequency as the reference's 5000 N/m on a 1 kg box.</summary>
        protected override float DragStiffness => 5000f * m_Spec.BrickMass;
        /// <summary>Tiled terrain steps of one plate (a third of a brick) at the brick scale.</summary>
        protected override float DefaultTileStep => Brick.BodyHeight / 3f * BrickScale;
        /// <summary>The contact penalties ramp up from their minimum over the first steps and the stacks sink a few centimetres
        /// meanwhile; settle that before showing the castle (single substeps: nothing moves fast yet).</summary>
        protected override int SettleSteps => 180;
        protected override int SettleSubsteps => 1;

        protected override AvbdGpuConfig CreateConfig()
        {
            // the outlying castles sleep: room for their bodies and joints, the pools stay sized for MaxBodies awake bricks
            int total = MaxBodies + (Outlying > 0 ? OutlyingBodies : 0);
            var cfg = AvbdGpuConfig.ForBodies(total, MaxBodies);
            cfg.MaxJoints = total * 12;                       // four snap joints per brick overlap (about ten per brick) plus drag joints
            cfg.MaxLinks = cfg.MaxJoints * 2 + 4096;
            cfg.MaxSpawns = math.max(cfg.MaxSpawns, UnitCapacity + ArrowCapacity + ShotCapacity);
            return cfg;
        }

        protected override void Configure()
        {
            var args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == "-avbd-snap") Snap = true;
                if (args[i] == "-avbd-siege") SiegeOnLoad = true;
                if (args[i] == "-avbd-outlying" && i + 1 < args.Length && int.TryParse(args[i + 1], out int outlying)) Outlying = outlying;
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
            int n = m_Layout.Bricks.Count;
            // the outlying copies: as many as fit the reserved bodies (eight at most), on the cells of a 3 x 3 grid around the castle
            // whose pitch keeps every plateau, skirt and army clear of the next
            float side = plan.Side * Brick.Pitch * BrickScale;
            float border = 4f * Brick.Pitch * BrickScale;
            m_OutlyingCount = Outlying > 0 && OutlyingBodies > 0 ? math.min(math.min(Outlying, 8), OutlyingBodies / math.max(n, 1)) : 0;
            m_OutlyingCentres = new float2[m_OutlyingCount];
            float pitch = side + 2f * border + 2f * TerrainParams.Margin + 2f * TerrainParams.Skirt;
            var cells = new[] { new float2(1, 0), new float2(-1, 0), new float2(0, 1), new float2(0, -1), new float2(1, 1), new float2(-1, -1), new float2(1, -1), new float2(-1, 1) };
            for (int k = 0; k < m_OutlyingCount; k++) m_OutlyingCentres[k] = cells[k] * pitch;
            // the terrain (or the flat ground), with a plateau under the castle's footprint and a few studs around it, and one
            // under every outlying copy (cut before the field goes up)
            float2 halfSide = side * 0.5f;
            var field = CreateTerrain();
            var outlyingPlateau = new float[m_OutlyingCount];
            if (field != null)
                for (int k = 0; k < m_OutlyingCount; k++) outlyingPlateau[k] = CutPlateau(field, m_OutlyingCentres[k], halfSide, border, 0f);
            m_Plateau = AddGround(field, BrickFriction, halfSide, border, out int groundBody);
            m_Spec = new BrickSpec
            {
                Scale = BrickScale, Density = BrickMass / volume, Friction = BrickFriction, Margin = AvbdGpuConstants.CollisionMargin,
                Origin = new float3(-c.x, m_Plateau, -c.y),
            };
            m_FirstBrick = BrickCastle.Build(m_World, m_Layout, m_Spec);
            m_SnapJoints = Snap ? BrickCastle.AddSnapJoints(m_World, m_Layout, m_FirstBrick, m_Spec, SnapFractureLateral, SnapFractureTension) : 0;
            // the outlying copies, each built asleep as one island right behind the castle's bricks
            for (int k = 0; k < m_OutlyingCount; k++)
            {
                var spec = m_Spec;
                spec.Origin = new float3(-c.x + m_OutlyingCentres[k].x, outlyingPlateau[k], -c.y + m_OutlyingCentres[k].y);
                int first = BrickCastle.Build(m_World, m_Layout, spec);
                if (Snap) BrickCastle.AddSnapJoints(m_World, m_Layout, first, spec, SnapFractureLateral, SnapFractureTension);
                m_World.SleepRange(first, n);
            }
            m_OutlyingBricks = m_OutlyingCount * n;

            // colours: the ground, then one tint per brick from its tone with a little per-brick variation (the copies alike)
            int bricks = n + m_OutlyingBricks;
            if (m_Tints == null || m_Tints.Length < m_FirstBrick + bricks) m_Tints = new uint[m_FirstBrick + bricks];
            m_Tints[groundBody] = AvbdGpuRenderer.Tint(new Color32(78, 112, 58, 255));
            var rng = new Unity.Mathematics.Random(0x9E3779B9u);
            for (int i = 0; i < n; i++) m_Tints[m_FirstBrick + i] = AvbdGpuRenderer.Tint(ToneColor(m_Layout.Bricks[i].Tone, rng.NextFloat()));
            for (int k = 1; k <= m_OutlyingCount; k++) System.Array.Copy(m_Tints, m_FirstBrick, m_Tints, m_FirstBrick + k * n, n);
            m_Renderer.SetTints(m_Tints, 0, m_FirstBrick + bricks);
            if (BrickMesh != null)
                m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = BrickMesh, Scale = BrickScale, Offset = m_Spec.MeshOffset, Start = m_FirstBrick, Count = bricks });

            // the siege's body pools (retired slots until an army is spawned), drawn with the figure and arrow models
            var siegeSpec = SiegeSpec.Default;
            siegeSpec.Scale = BrickScale; siegeSpec.Margin = AvbdGpuConstants.CollisionMargin; siegeSpec.Friction = BrickFriction;
            siegeSpec.UnitMass = UnitMass; siegeSpec.ArrowMass = ArrowMass; siegeSpec.BallCube = ShotCube;
            m_Siege = new SiegeSystem(m_World, siegeSpec, UnitCapacity, ArrowCapacity, ShotCapacity, SiegeParams) { OnSpawned = OnSiegeSpawn, OnRetiring = OnSiegeRetire, Terrain = m_World.Terrain };
            m_SiegeActive = false;
            if (FigureMesh != null)
                m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = FigureMesh, Scale = BrickScale, Offset = siegeSpec.UnitMeshOffset, Start = m_Siege.Units.Start, Count = m_Siege.Units.Capacity });
            if (ArrowMesh != null)
                m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = ArrowMesh, Scale = BrickScale, Offset = siegeSpec.ArrowMeshOffset, Start = m_Siege.Arrows.Start, Count = m_Siege.Arrows.Capacity });
            if (SiegeOnLoad) ToggleSiege();

            // fewer substeps for the big castles: their cannonball crosses a fraction of the wall thickness per step even at one
            m_World.Params.Substeps = n > 20000 ? 1 : n > 8000 ? 2 : 3;
            cameraTarget = new float3(0f, m_Plateau + 1.2f * plan.WallCourses * Brick.BodyHeight * BrickScale, 0f);
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
            if (kb.tKey.wasPressedThisFrame) CycleTerrain();
            if (kb.oKey.wasPressedThisFrame) { Outlying = Outlying >= 8 ? 0 : Outlying + 4; RecreateWorld(); }
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
                (m_SiegeActive ? $"<color=#ffcc88>siege</color> (volleys {(m_Siege.AutoVolleys ? "auto" : "manual")}): {m_Siege.Summary()}\n" : "no siege (U)\n") +
                (m_OutlyingCount > 0 ? $"<color=#c8a8e8>{m_OutlyingCount} outlying copies</color> built asleep around the castle ({m_OutlyingBricks} bricks; the world simulates {m_World.Config.MaxActive} at once)\n" : "") +
                TerrainText(m_Plateau) + "\n\n" +
                StatsText() + "\n\n" +
                "1-0 castle size  , . prev/next  R rebuild  J snap bricks on/off  T terrain  O outlying copies 0/4/8  U siege on/off  V volley  K auto volleys  X retire the dead and spent now  Space pause  N step\n" +
                "F1 contacts  F2 colour mode  F5 joints  F6 collision boxes  F7 shadows  F8 sleep on/off  +/- iterations  [ ] substeps  B/Enter cannonball  G gravity  H hide HUD\n" +
                "LMB drag  RMB orbit  MMB pan  wheel / Q E zoom  W A S D orbit";
        }
    }
}
