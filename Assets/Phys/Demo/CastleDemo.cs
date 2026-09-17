using System.Collections.Generic;
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
    /// box body drawn with the brick model. Bricks are either dry-stacked (friction only) or snapped together with breakable joints;
    /// trees and foliage of the construction-piece pack (the brick-assembly documents of Resources/Trees and Resources/Foliage,
    /// <see cref="Vegetation"/>) stand asleep on the ground around it. The siege lives in its own scene (Siege.unity, <see cref="SiegeDemo"/>).</summary>
    public class CastleDemo : DemoBase
    {
        [Tooltip("The brick model (Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx); the collision boxes are drawn when unset.")]
        public Mesh BrickMesh;
        [Tooltip("The 27 piece models of the construction pack in catalog order (PieceCatalog.Pieces; Phys / Assign Castle Meshes fills them in), " +
                 "for the trees and the foliage. Unset pieces are drawn as boxes.")]
        public Mesh[] PieceMeshes;
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
        [Tooltip("Cast the shadows of the bricks and pieces from their collision boxes instead of the models (-avbd-meshshadows turns it off): a " +
                 "shadow map draws every body once per cascade, and the studs do not show in a shadow.")]
        public bool BoxShadows = true;
        [Tooltip("Cannonball: cube size (model metres), mass (kg) and speed (model metres per second).")]
        public float ShotCube = 0.12f;
        public float ShotMass = 30f;
        public float ShotVelocity = 24f;
        [Tooltip("Copies of the castle built asleep around it, up to eight (key O cycles 0 / 4 / 8; -avbd-outlying n): a world with more " +
                 "bricks than it simulates at once. A copy wakes when something hits it and sleeps again once it has settled.")]
        public int Outlying;
        [Tooltip("Bodies reserved for the outlying castles on top of MaxBodies, which stays the number of bricks awake at a time.")]
        public int OutlyingBodies = 131072;
        [Tooltip("Resources folder holding the tree documents (brick assemblies of the construction-piece pack, Tools/generate_trees.py).")]
        public string TreeFolder = "Trees";
        [Tooltip("Trees scattered on the level ground around the castle, up to 30 (key F cycles 0 / 10 / 20 / 30; -avbd-trees n): each a " +
                 "document of the tree folder turned by quarter turns, built asleep as one island, costing nothing until something hits it.")]
        public int Trees = 20;
        [Tooltip("Bodies reserved for the trees on top of MaxBodies (thirty of the largest kind need 44 160).")]
        public int TreeBodies = 49152;
        [Tooltip("Seed of the trees' kinds, places and turns (the foliage draws its own from it).")]
        public uint TreeSeed = 1;
        [Tooltip("Mass of a 2 x 3 brick of a tree (kg), a quarter of the castle's: a crown of a thousand pieces stands on the few contacts of its " +
                 "trunk top, and at the castle's brick mass it sinks its trunk a decimetre and tears the snaps of its lowest courses as it lands " +
                 "(the castle's own brick mass was chosen the same way, for the gatehouse).")]
        public float TreeMass = 0.25f;
        [Tooltip("Resources folder holding the foliage documents (bushes, grasses, rocks, logs and stumps of the construction-piece pack, " +
                 "Tools/generate_foliage.py).")]
        public string FoliageFolder = "Foliage";
        [Tooltip("Clumps of foliage scattered around the castle per tree planted (six by default; -avbd-foliage n): each a document of the " +
                 "foliage folder turned by quarter turns, placed like the trees and clear of them, built asleep as an island of its own. " +
                 "A clump on the hills gets a terrace of its own, so the clumps there stand farther apart than on the level ground.")]
        public int FoliagePerTree = 6;
        [Tooltip("Bodies reserved for the foliage on top of MaxBodies (the twenty kinds average 383 pieces: thirty trees' hundred and eighty " +
                 "clumps take 69 000 on average, 76 000 at the outside).")]
        public int FoliageBodies = 81920;
        [Tooltip("Mass of a 2 x 3 brick of the foliage (kg): the castle's brick mass (a clump rests on the ground with its whole footprint, so " +
                 "nothing in it needs the trees' lightness, and at the trees' quarter kilogram a cannonball turns a stump into a cloud of plates " +
                 "where at a kilogram it blows half of it out and the rest stands).")]
        public float FoliageMass = 0.25f;

        /// <summary>A scripted explosion (<c>-avbd-blasts file</c>): <see cref="AvbdGpuWorld.Blast"/> at a step of the scene, placed in metres
        /// from the castle's centre at ground level (y up from the plateau), with a crater of the pulverize radius dug into the terrain
        /// under it when <see cref="CraterDepth"/> is set (a mining charge: what stood on the crater falls into it).</summary>
        public struct ScriptedBlast
        {
            public int Step;
            public float3 Position;
            public float ImpactRadius, Impulse, Lift, PulverizeRadius, CraterDepth;
        }

        BrickLayout m_Layout;
        BrickSpec m_Spec;
        int m_FirstBrick, m_SnapJoints;
        readonly List<ScriptedBlast> m_Blasts = new List<ScriptedBlast>();
        int m_Step, m_BlastsFired;
        int m_OutlyingCount, m_OutlyingBricks;
        float2[] m_OutlyingCentres = new float2[0];
        Vegetation m_Vegetation;
        float m_Plateau;
        uint[] m_Tints;
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
        /// <summary>Ground height under the castle (the plateau levelled into the terrain; 0 on the flat ground).</summary>
        public float Plateau => m_Plateau;
        /// <summary>The outlying copies built asleep around the castle: how many, their centres, their bricks (contiguous after the
        /// castle's own).</summary>
        public int OutlyingCount => m_OutlyingCount;
        public float2[] OutlyingCentres => m_OutlyingCentres;
        public int OutlyingBricks => m_OutlyingBricks;
        public int FirstOutlyingBrick => m_FirstBrick + BrickCount;
        /// <summary>The trees and the foliage around the castle (their pieces contiguous after the outlying copies' bricks).</summary>
        public Vegetation Vegetation => m_Vegetation;
        /// <summary>The scripted explosions, in step order; <see cref="BlastsFired"/> of them have gone off since the scene was loaded.</summary>
        public IReadOnlyList<ScriptedBlast> Blasts => m_Blasts;
        public int BlastsFired => m_BlastsFired;
        /// <summary>Steps run since the scene was loaded (the settle steps not counted).</summary>
        public int Step => m_Step;

        protected override int SceneCount => CastlePlan.Presets.Length;
        protected override string SceneName(int index) => CastlePlan.Presets[index].Name;
        protected override float HudHeight => 270f;
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

        Vegetation.Settings VegetationSettings => new Vegetation.Settings
        {
            TreeFolder = TreeFolder, FoliageFolder = FoliageFolder, Trees = Trees, TreeBodies = TreeBodies, FoliagePerTree = FoliagePerTree, FoliageBodies = FoliageBodies,
            Seed = TreeSeed, TreeMass = TreeMass, FoliageMass = FoliageMass, Scale = BrickScale, Friction = BrickFriction,
            Snap = Snap, SnapFractureLateral = SnapFractureLateral, SnapFractureTension = SnapFractureTension,
        };

        protected override AvbdGpuConfig CreateConfig()
        {
            // the outlying castles, the trees and the foliage sleep: room for their bodies and joints, the pools stay sized for MaxBodies
            // awake bricks
            int total = MaxBodies + (Outlying > 0 ? OutlyingBodies : 0) + Vegetation.ReservedBodies(VegetationSettings);
            var cfg = AvbdGpuConfig.ForBodies(total, MaxBodies);
            cfg.MaxJoints = total * 12;                       // four snap joints per brick overlap (about ten per brick) plus drag joints
            cfg.MaxLinks = cfg.MaxJoints * 2 + 4096;
            return cfg;
        }

        protected override void ParseArgs(string[] args)
        {
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == "-avbd-snap") Snap = true;
                if (args[i] == "-avbd-outlying" && i + 1 < args.Length && int.TryParse(args[i + 1], out int outlying)) Outlying = outlying;
                if (args[i] == "-avbd-trees" && i + 1 < args.Length && int.TryParse(args[i + 1], out int trees)) Trees = trees;
                if (args[i] == "-avbd-foliage" && i + 1 < args.Length && int.TryParse(args[i + 1], out int foliage)) FoliagePerTree = foliage;
                if (args[i] == "-avbd-noshadows") Shadows = false;
                if (args[i] == "-avbd-meshshadows") BoxShadows = false;
                if (args[i] == "-avbd-blasts" && i + 1 < args.Length) LoadBlasts(args[i + 1]);
            }
        }

        /// <summary>Reads a blast script: one explosion per line as <c>step x y z impactRadius impulse lift pulverizeRadius [craterDepth]</c>
        /// (metres from the castle's centre at ground level, N s, see <see cref="AvbdGpuWorld.Blast"/>; a crater of the pulverize
        /// radius and that depth is dug under the blast when the ninth number is given), <c>#</c> comments; the explosions go off at
        /// their steps after every load of the scene.</summary>
        public void LoadBlasts(string path)
        {
            m_Blasts.Clear();
            var culture = System.Globalization.CultureInfo.InvariantCulture;
            string[] lines;
            try { lines = System.IO.File.ReadAllLines(path); }
            catch (System.Exception e) { Debug.LogError($"CastleDemo: cannot read the blast script {path}: {e.Message}"); return; }
            foreach (string raw in lines)
            {
                string line = raw.Split('#')[0].Trim();
                if (line.Length == 0) continue;
                var f = line.Split(new[] { ' ', '\t', ',' }, System.StringSplitOptions.RemoveEmptyEntries);
                if (f.Length < 8) { Debug.LogWarning($"CastleDemo: blast script line ignored (8 or 9 numbers expected): {raw}"); continue; }
                var v = new float[9];
                bool ok = true;
                for (int k = 0; k < math.min(f.Length, 9); k++) ok &= float.TryParse(f[k], System.Globalization.NumberStyles.Float, culture, out v[k]);
                if (!ok) { Debug.LogWarning($"CastleDemo: blast script line ignored (not a number): {raw}"); continue; }
                AddBlast(new ScriptedBlast { Step = (int)v[0], Position = new float3(v[1], v[2], v[3]), ImpactRadius = v[4], Impulse = v[5], Lift = v[6], PulverizeRadius = v[7], CraterDepth = v[8] });
            }
            Debug.Log($"CastleDemo: {m_Blasts.Count} scripted blasts from {path}");
        }

        /// <summary>Queues a scripted explosion (kept in step order; one queued for a step already run goes off at the next step).</summary>
        public void AddBlast(ScriptedBlast blast)
        {
            int at = m_Blasts.Count;
            while (at > 0 && m_Blasts[at - 1].Step > blast.Step) at--;
            m_Blasts.Insert(at, blast);
            if (at < m_BlastsFired) m_BlastsFired = at;
        }

        protected override void OnStep()
        {
            bool cratered = false;
            while (m_BlastsFired < m_Blasts.Count && m_Blasts[m_BlastsFired].Step <= m_Step)
            {
                var b = m_Blasts[m_BlastsFired++];
                float3 centre = new float3(b.Position.x, m_Plateau + b.Position.y, b.Position.z);
                m_World.Blast(centre, b.ImpactRadius, b.Impulse, b.Lift, b.PulverizeRadius);
                // the crater: the blast wakes everything within the pulverize radius, so the terrain update leaves the rest asleep
                if (b.CraterDepth > 0f && b.PulverizeRadius > 0f && m_World.Terrain != null) { m_World.Terrain.Crater(centre.xz, b.PulverizeRadius, b.CraterDepth); cratered = true; }
                Debug.Log($"CastleDemo: blast {m_BlastsFired} at step {m_Step} ({b.Position.x:F1}, {b.Position.y:F1}, {b.Position.z:F1}) impact {b.ImpactRadius:F1} m x {b.Impulse:F0} N s lift {b.Lift:F2}, pulverize {b.PulverizeRadius:F1} m" +
                    (b.CraterDepth > 0f ? $", crater {b.CraterDepth:F1} m deep" : ""));
            }
            if (cratered)
            {
                m_World.UpdateTerrain(wakeAll: false);
                m_TerrainView.Show(m_World.Terrain, SceneTerrain);
            }
            m_Step++;
        }

        protected override void Configure()
        {
            LoadDocuments();
            m_Renderer.Shadows = Shadows;
            m_Renderer.BoxShadows = BoxShadows;
            m_Renderer.DrawJoints = false;
#if UNITY_EDITOR
            if (BrickMesh == null) BrickMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx");
#endif
            if (BrickMesh == null) Debug.LogWarning("CastleDemo: no brick mesh assigned, drawing the collision boxes");
        }

        /// <summary>Reads the tree and the foliage folders and parses every document once (a rejected document is logged and skipped).</summary>
        public void LoadDocuments()
        {
            m_Vegetation = new Vegetation(VegetationSettings, "CastleDemo");
            m_Vegetation.LoadDocuments();
        }

        protected override void BuildScene(int index, out float3 cameraTarget, out float cameraDistance)
        {
            var plan = CastlePlan.Presets[index];
            m_Step = 0; m_BlastsFired = 0;   // the scripted explosions go off again after a rebuild
            m_Layout = BrickCastle.Generate(plan);
            float2 c = BrickCastle.Center(plan) * Brick.Pitch * BrickScale;
            float volume = Brick.Width * Brick.BodyHeight * Brick.Length * BrickScale * BrickScale * BrickScale;
            int n = m_Layout.Bricks.Count;
            // the outlying copies: as many as fit the reserved bodies (eight at most), on the cells of a 3 x 3 grid around the castle
            // whose pitch keeps every plateau and skirt clear of the next
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
            if (BrickMesh != null)   // one range per castle, so that each copy is culled on its own bounds
                for (int k = 0; k <= m_OutlyingCount; k++)
                    m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = BrickMesh, Scale = BrickScale, Offset = m_Spec.MeshOffset, Start = m_FirstBrick + k * n, Count = n });

            // the trees and the foliage on the ground around the castle, each built asleep as an island: the ground under them is
            // shaped after the castle's plateau (what stands on the level ground stands on it, what stands on the hills gets a terrace
            // of its own) and uploaded once
            var around = ExclusionOf(side, border);
            m_Vegetation.Params = VegetationSettings;
            if (m_Vegetation.Scatter(index, field, around)) m_World.UpdateTerrain();
            m_Vegetation.Build(m_World, m_Renderer, ref m_Tints, piece => PieceMesh(ref PieceMeshes, piece), around);
            m_Renderer.SetTints(m_Tints, 0, m_World.BodyCount);

            // fewer substeps for the big castles: their cannonball crosses a fraction of the wall thickness per step even at one
            m_World.Params.Substeps = n > 20000 ? 1 : n > 8000 ? 2 : 3;
            cameraTarget = new float3(0f, m_Plateau + 1.2f * plan.WallCourses * Brick.BodyHeight * BrickScale, 0f);
            cameraDistance = 1.4f * plan.Side * Brick.Pitch * BrickScale;
        }

        /// <summary>The ground the trees and the foliage keep clear of: the castle's plateau core and the outlying copies' plateaus with
        /// their skirts; the trees go out to the middle of the skirt, the foliage to the foot of the hills (at least 60 m from the
        /// core on a flat ground, so that twenty trees always fit).</summary>
        Vegetation.Exclusion ExclusionOf(float side, float border)
        {
            float half = side * 0.5f + border;   // the castle's plateau core
            return new Vegetation.Exclusion
            {
                CoreHalf = half,
                TreeOuter = half + math.max(TerrainParams.Margin + 0.5f * TerrainParams.Skirt, 60f),
                FoliageOuter = half + math.max(TerrainParams.Margin + TerrainParams.Skirt, 60f),
                OutlyingCore = half + TerrainParams.Skirt,
                OutlyingCentres = m_OutlyingCentres,
            };
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
            if (kb.tKey.wasPressedThisFrame) CycleTerrain();
            if (kb.oKey.wasPressedThisFrame) { Outlying = Outlying >= 8 ? 0 : Outlying + 4; RecreateWorld(); }
            if (kb.fKey.wasPressedThisFrame) { Trees = Trees >= 30 ? 0 : Trees + 10; RecreateWorld(); }
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
                (m_OutlyingCount > 0 ? $"<color=#c8a8e8>{m_OutlyingCount} outlying copies</color> built asleep around the castle ({m_OutlyingBricks} bricks; the world simulates {m_World.Config.MaxActive} at once)\n" : "") +
                m_Vegetation.AroundText() +
                TerrainText(m_Plateau) + "\n\n" +
                StatsText() + "\n\n" +
                "1-0 castle size  , . prev/next  R rebuild  J snap bricks on/off  T terrain  O outlying copies 0/4/8  F trees 0/10/20/30 (and their foliage)  Space pause  N step\n" +
                "F1 contacts  F2 colour mode  F5 joints  F6 collision boxes  F7 shadows  F8 sleep on/off  +/- iterations  [ ] substeps  B/Enter cannonball  G gravity  H hide HUD\n" +
                "LMB drag  RMB orbit  MMB pan  wheel / Q E zoom  W A S D orbit";
        }
    }
}
