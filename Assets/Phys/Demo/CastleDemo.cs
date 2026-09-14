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
    /// <summary>A castle of construction bricks (Assets/Models/ConstructorBlock2x3) on the GPU solver (Castle.unity): every brick is a
    /// box body drawn with the brick model. Bricks are either dry-stacked (friction only) or snapped together with breakable joints.</summary>
    public class CastleDemo : DemoBase
    {
        [Tooltip("The brick model (Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx); the collision boxes are drawn when unset.")]
        public Mesh BrickMesh;
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

        BrickLayout m_Layout;
        BrickSpec m_Spec;
        int m_FirstBrick, m_SnapJoints;
        uint[] m_Tints;
        static readonly Color32 s_Iron = new Color32(70, 72, 78, 255);

        void Reset()
        {
            StartScene = 1;
            MaxBodies = 16384;
        }

        public BrickLayout Layout => m_Layout;
        public int FirstBrick => m_FirstBrick;
        public int BrickCount => m_Layout?.Bricks.Count ?? 0;
        public BrickSpec Spec => m_Spec;

        protected override int SceneCount => CastlePlan.Presets.Length;
        protected override string SceneName(int index) => CastlePlan.Presets[index].Name;
        protected override float HudHeight => 220f;
        protected override float3 ShotSize => ShotCube * BrickScale;
        protected override float ShotDensity => ShotMass / math.pow(ShotCube * BrickScale, 3f);
        /// <summary>Speeds scale with the square root of lengths under the same gravity (dynamic similarity).</summary>
        protected override float ShotSpeed => ShotVelocity * math.sqrt(BrickScale);
        protected override float ShotDistance => 0.5f * BrickScale;
        /// <summary>Same frequency as the reference's 5000 N/m on a 1 kg box.</summary>
        protected override float DragStiffness => 5000f * m_Spec.BrickMass;
        /// <summary>The contact penalties ramp up from their minimum over the first steps and the stacks sink a few centimetres
        /// meanwhile; settle that before showing the castle.</summary>
        protected override int SettleSteps => 240;

        protected override AvbdGpuConfig CreateConfig()
        {
            var cfg = AvbdGpuConfig.ForBodies(MaxBodies);
            cfg.MaxJoints = MaxBodies * 12;                   // four snap joints per brick overlap (about ten per brick) plus drag joints
            cfg.MaxLinks = cfg.MaxJoints * 2 + 4096;
            return cfg;
        }

        protected override void Configure()
        {
            foreach (var arg in System.Environment.GetCommandLineArgs()) if (arg == "-avbd-snap") Snap = true;
            m_World.Params.Substeps = 3;                      // the cannonball and its debris travel a fraction of a brick per substep
            m_Renderer.Shadows = Shadows;
            m_Renderer.DrawJoints = false;
#if UNITY_EDITOR
            if (BrickMesh == null) BrickMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx");
#endif
            if (BrickMesh == null) Debug.LogWarning("CastleDemo: no brick mesh assigned, drawing the collision boxes");
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

            cameraTarget = new float3(0f, 1.3f * plan.WallCourses * Brick.BodyHeight * BrickScale, 0f);
            cameraDistance = 1.7f * plan.Side * Brick.Pitch * BrickScale;
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
            if (m_Tints == null || m_Tints.Length <= body) System.Array.Resize(ref m_Tints, math.max(body + 1, 1024));
            m_Tints[body] = AvbdGpuRenderer.Tint(s_Iron);
            m_Renderer.SetTints(m_Tints, body, 1);
        }

        protected override void HandleSceneKeys()
        {
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb.jKey.wasPressedThisFrame) { Snap = !Snap; Load(m_Scene); }
            if (kb.f6Key.wasPressedThisFrame) m_Renderer.DrawCollisionBoxes = !m_Renderer.DrawCollisionBoxes;
            if (kb.f7Key.wasPressedThisFrame) { Shadows = !Shadows; m_Renderer.Shadows = Shadows; }
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
                (Snap ? $"<color=#88ddff>snapped</color>: {m_SnapJoints} joints; a snap breaks at {SnapFractureLateral:F0} N sideways, {SnapFractureTension:F0} N upward or {m_Spec.SnapBreakDistance * 100f:F1} cm apart  -  cannonball {ShotMass:F0} kg\n\n"
                      : $"dry-stacked (friction only)  -  cannonball {ShotMass:F0} kg\n\n") +
                StatsText() + "\n\n" +
                "1-3 castle size  , . prev/next  R rebuild  J snap bricks on/off  Space pause  N step  F1 contacts  F2 colour mode  F5 joints  F6 collision boxes  F7 shadows\n" +
                "+/- iterations  [ ] substeps  B/Enter shoot a cannonball  G gravity  H hide HUD  LMB drag brick  RMB orbit  MMB pan  wheel / Q E zoom  W A S D orbit";
        }
    }
}
