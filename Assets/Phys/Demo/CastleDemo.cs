using System.Collections.Generic;
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
    /// A siege (key U) surrounds it with figures that march in and fire volleys at the garrison and the walls; trees and foliage of
    /// the construction-piece pack (the brick-assembly documents of Resources/Trees and Resources/Foliage) stand asleep on the ground
    /// around it.</summary>
    public class CastleDemo : DemoBase
    {
        [Tooltip("The brick model (Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx); the collision boxes are drawn when unset.")]
        public Mesh BrickMesh;
        [Tooltip("The figure model (Assets/Models/ConstructorFigure/ConstructorFigure.fbx) drawn for the units.")]
        public Mesh FigureMesh;
        [Tooltip("The arrow model (Assets/Models/ConstructorArrow/ConstructorArrow.fbx) drawn for arrows and rockets.")]
        public Mesh ArrowMesh;
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
        [Tooltip("Spawn the armies when a castle is loaded (also the -avbd-siege flag).")]
        public bool SiegeOnLoad;
        [Tooltip("Copies of the castle built asleep around it, up to eight (key O cycles 0 / 4 / 8; -avbd-outlying n): a world with more " +
                 "bricks than it simulates at once. A copy wakes when something hits it and sleeps again once it has settled.")]
        public int Outlying;
        [Tooltip("Bodies reserved for the outlying castles on top of MaxBodies, which stays the number of bricks awake at a time.")]
        public int OutlyingBodies = 131072;
        [Tooltip("Resources folder holding the tree documents (brick assemblies of the construction-piece pack, Tools/generate_trees.py).")]
        public string TreeFolder = "Trees";
        [Tooltip("Trees scattered on the level ground around the castle, clear of the armies' bands, up to 30 (key F cycles 0 / 10 / 20 / 30; " +
                 "-avbd-trees n): each a document of the tree folder turned by quarter turns, built asleep as one island, costing nothing until " +
                 "something hits it.")]
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
        [Tooltip("Body pools reserved for the siege: units, arrow-shaped and cube projectiles.")]
        public int UnitCapacity = 512;
        public int ArrowCapacity = 2048;
        public int ShotCapacity = 512;
        public float UnitMass = 1f;
        public float ArrowMass = 0.02f;
        public SiegeSettings SiegeParams = SiegeSettings.Default;

        /// <summary>A tree standing around the castle: which document, where (world xz and the ground height under it), turned by how
        /// many quarter turns, and its bodies (contiguous, one island).</summary>
        public struct TreePlacement
        {
            public int Kind;
            public float2 Centre;
            public float Ground;
            public int Turns;
            public int First, Count;
            /// <summary>How far the crown reaches from the trunk (m).</summary>
            public float Radius;
        }

        /// <summary>A clump of foliage standing around the castle: which document, where (world xz and the ground height under it),
        /// turned by how many quarter turns, and its pieces: the parts [PartStart, PartStart + Count) of its cluster's assembly, which
        /// <see cref="FoliageBody"/> maps to bodies (the clumps of a cluster are built interleaved by piece, so that a cluster is drawn
        /// in a handful of ranges rather than one per clump).</summary>
        public struct FoliagePlacement
        {
            public int Kind;
            public float2 Centre;
            public float Ground;
            public int Turns;
            /// <summary>Half the width of the clump's footprint (m).</summary>
            public float Radius;
            /// <summary>The clump stands on a terrace of its own (the ground under it was not level).</summary>
            public bool Terraced;
            public int Cluster, PartStart, Count;
        }

        /// <summary>The clumps of one grid cell, built as one assembly (its parts in the clumps' order) with the bodies laid out by piece.</summary>
        sealed class FoliageCluster
        {
            public BrickAssembly Assembly;
            public AssemblyBodies Bodies;
        }

        /// <summary>The ground around the castle the trees and the foliage are scattered on: half the width of the plateau core, the
        /// outer wall faces, how far out the armies march and fire along the four faces and half the width of their bands, how far
        /// out the trees and the foliage go, and half the width of an outlying copy's plateau with its skirt.</summary>
        struct Surroundings
        {
            public float Half, Wall, Reach, BandHalf, TreeOuter, FoliageOuter, OutlyingCore;

            /// <summary>A footprint of half width <paramref name="r"/> at <paramref name="c"/> lies off the castle's plateau core, off the
            /// armies' bands (the cross of the four wall faces out to the attackers' starting line) and clear of the outlying copies.</summary>
            public bool Clear(float2 c, float r, float2[] outlying)
            {
                if (math.cmax(math.abs(c)) < Half + r) return false;
                if ((math.abs(c.x) < BandHalf + r && math.abs(c.y) < Wall + Reach + r) || (math.abs(c.y) < BandHalf + r && math.abs(c.x) < Wall + Reach + r)) return false;
                foreach (var oc in outlying) if (math.cmax(math.abs(c - oc)) < OutlyingCore + r) return false;
                return true;
            }
        }

        /// <summary>Half the width of the square levelled exactly under a tree's trunk (m), and how far a terrace blends into the terrain.</summary>
        const float TrunkSquare = 3f, TerraceBlend = 2f;
        /// <summary>Cells per axis of the grid the foliage is clustered by (over the square it is scattered on).</summary>
        public const int FoliageGrid = 4;

        BrickLayout m_Layout;
        BrickSpec m_Spec;
        int m_FirstBrick, m_SnapJoints;
        int m_OutlyingCount, m_OutlyingBricks;
        float2[] m_OutlyingCentres = new float2[0];
        TextAsset[] m_TreeDocuments = new TextAsset[0];
        BrickAssembly[] m_TreeKinds = new BrickAssembly[0];
        BrickAssembly[,] m_TreeTurned;
        readonly List<TreePlacement> m_TreePlacements = new List<TreePlacement>();
        int m_FirstTree, m_TreePieces;
        TextAsset[] m_FoliageDocuments = new TextAsset[0];
        BrickAssembly[] m_FoliageKinds = new BrickAssembly[0];
        readonly List<FoliagePlacement> m_FoliagePlacements = new List<FoliagePlacement>();
        readonly List<FoliageCluster> m_FoliageClusters = new List<FoliageCluster>();
        int m_FirstFoliage, m_FoliagePieces;
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
        /// <summary>The tree documents of the tree folder (by file name) and their parsed assemblies (null where a document was
        /// rejected); the trees standing around the castle, their pieces contiguous after the outlying copies' bricks.</summary>
        public TextAsset[] TreeDocuments => m_TreeDocuments;
        public BrickAssembly[] TreeKinds => m_TreeKinds;
        public IReadOnlyList<TreePlacement> TreePlacements => m_TreePlacements;
        public int TreeCount => m_TreePlacements.Count;
        public int TreePieces => m_TreePieces;
        public int FirstTreePiece => m_FirstTree;
        /// <summary>The foliage documents of the foliage folder (by file name) and their parsed assemblies (null where a document was
        /// rejected); the clumps standing around the castle, their pieces contiguous after the trees', cluster by cluster.</summary>
        public TextAsset[] FoliageDocuments => m_FoliageDocuments;
        public BrickAssembly[] FoliageKinds => m_FoliageKinds;
        public IReadOnlyList<FoliagePlacement> FoliagePlacements => m_FoliagePlacements;
        public int FoliageCount => m_FoliagePlacements.Count;
        public int FoliagePieces => m_FoliagePieces;
        public int FirstFoliagePiece => m_FirstFoliage;
        public int FoliageClusterCount => m_FoliageClusters.Count;
        /// <summary>The bodies of a cluster (their parts in the clumps' order, see <see cref="AssemblyBodies.BodyOfPart"/>).</summary>
        public AssemblyBodies FoliageClusterBodies(int cluster) => m_FoliageClusters[cluster].Bodies;
        /// <summary>The body of the i-th piece of a clump.</summary>
        public int FoliageBody(in FoliagePlacement clump, int i) => m_FoliageClusters[clump.Cluster].Bodies.BodyOfPart[clump.PartStart + i];

        protected override int SceneCount => CastlePlan.Presets.Length;
        protected override string SceneName(int index) => CastlePlan.Presets[index].Name;
        protected override float HudHeight => 286f;
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
            // the outlying castles, the trees and the foliage sleep: room for their bodies and joints, the pools stay sized for MaxBodies
            // awake bricks
            int total = MaxBodies + (Outlying > 0 ? OutlyingBodies : 0) + (Trees > 0 ? TreeBodies : 0) + (Trees > 0 && FoliagePerTree > 0 ? FoliageBodies : 0);
            var cfg = AvbdGpuConfig.ForBodies(total, MaxBodies);
            cfg.MaxJoints = total * 12;                       // four snap joints per brick overlap (about ten per brick) plus drag joints
            cfg.MaxLinks = cfg.MaxJoints * 2 + 4096;
            cfg.MaxSpawns = math.max(cfg.MaxSpawns, UnitCapacity + ArrowCapacity + ShotCapacity);
            return cfg;
        }

        protected override void ParseArgs(string[] args)
        {
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == "-avbd-snap") Snap = true;
                if (args[i] == "-avbd-siege") SiegeOnLoad = true;
                if (args[i] == "-avbd-outlying" && i + 1 < args.Length && int.TryParse(args[i + 1], out int outlying)) Outlying = outlying;
                if (args[i] == "-avbd-trees" && i + 1 < args.Length && int.TryParse(args[i + 1], out int trees)) Trees = trees;
                if (args[i] == "-avbd-foliage" && i + 1 < args.Length && int.TryParse(args[i + 1], out int foliage)) FoliagePerTree = foliage;
                if (args[i] == "-avbd-noshadows") Shadows = false;
                if (args[i] == "-avbd-meshshadows") BoxShadows = false;
            }
        }

        protected override void Configure()
        {
            SiegeParams = SiegeParams.WithDefaults();   // fields a scene was serialised without come out as zero
            LoadDocuments();
            m_Renderer.Shadows = Shadows;
            m_Renderer.BoxShadows = BoxShadows;
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
            if (BrickMesh != null)   // one range per castle, so that each copy is culled on its own bounds
                for (int k = 0; k <= m_OutlyingCount; k++)
                    m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = BrickMesh, Scale = BrickScale, Offset = m_Spec.MeshOffset, Start = m_FirstBrick + k * n, Count = n });

            // the trees and the foliage on the ground around the castle, each built asleep as an island: the ground under them is
            // shaped after the castle's plateau (what stands on the level ground stands on it, what stands on the hills gets a terrace
            // of its own) and uploaded once
            var around = SurroundingsOf(side, border);
            bool cut = ScatterTrees(index, field, around);
            cut |= ScatterFoliage(index, field, around);
            if (cut) m_World.UpdateTerrain();
            BuildTrees();
            BuildFoliage(around);
            m_Renderer.SetTints(m_Tints, 0, m_World.BodyCount);

            // the siege's body pools (retired slots until an army is spawned), drawn with the figure and arrow models
            var siegeSpec = SiegeSpec.Default;
            siegeSpec.Scale = BrickScale; siegeSpec.Margin = AvbdGpuConstants.CollisionMargin; siegeSpec.Friction = BrickFriction;
            siegeSpec.UnitMass = UnitMass; siegeSpec.ArrowMass = ArrowMass; siegeSpec.BallCube = ShotCube;
            m_Siege = new SiegeSystem(m_World, siegeSpec, UnitCapacity, ArrowCapacity, ShotCapacity, SiegeParams) { OnSpawned = OnSiegeSpawn, OnRetiring = OnSiegeRetire, Terrain = m_World.Terrain };
            m_SiegeActive = false;
            if (FigureMesh != null)   // the pools spawn anywhere: world-sized bounds
                m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = FigureMesh, Scale = BrickScale, Offset = siegeSpec.UnitMeshOffset, Start = m_Siege.Units.Start, Count = m_Siege.Units.Capacity, NoCulling = true });
            if (ArrowMesh != null)
                m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = ArrowMesh, Scale = BrickScale, Offset = siegeSpec.ArrowMeshOffset, Start = m_Siege.Arrows.Start, Count = m_Siege.Arrows.Capacity, NoCulling = true });
            if (SiegeOnLoad) ToggleSiege();

            // fewer substeps for the big castles: their cannonball crosses a fraction of the wall thickness per step even at one
            m_World.Params.Substeps = n > 20000 ? 1 : n > 8000 ? 2 : 3;
            cameraTarget = new float3(0f, m_Plateau + 1.2f * plan.WallCourses * Brick.BodyHeight * BrickScale, 0f);
            cameraDistance = 1.4f * plan.Side * Brick.Pitch * BrickScale;
        }

        /// <summary>Reads the tree and the foliage folders (the documents by file name) and parses every document once; a rejected
        /// document is logged and skipped.</summary>
        public void LoadDocuments()
        {
            LoadFolder(TreeFolder, "tree", Trees > 0, out m_TreeDocuments, out m_TreeKinds);
            LoadFolder(FoliageFolder, "foliage", Trees > 0 && FoliagePerTree > 0, out m_FoliageDocuments, out m_FoliageKinds);
            m_TreeTurned = new BrickAssembly[m_TreeKinds.Length, 4];
        }

        static void LoadFolder(string folder, string what, bool wanted, out TextAsset[] documents, out BrickAssembly[] kinds)
        {
            var docs = new List<TextAsset>(string.IsNullOrEmpty(folder) ? new TextAsset[0] : Resources.LoadAll<TextAsset>(folder));
            docs.Sort((a, b) => string.CompareOrdinal(a.name, b.name));
            documents = docs.ToArray();
            kinds = new BrickAssembly[documents.Length];
            for (int i = 0; i < documents.Length; i++)
            {
                try { kinds[i] = BrickAssembly.Parse(documents[i].text); }
                catch (BrickAssemblyException e) { Debug.LogError($"CastleDemo: {documents[i].name}.json rejected: {e.Message}"); }
            }
            if (documents.Length == 0 && wanted) Debug.LogWarning($"CastleDemo: no {what} documents in Resources/{folder}");
        }

        Surroundings SurroundingsOf(float side, float border)
        {
            var siege = SiegeParams;
            float half = side * 0.5f + border;                                                                        // the castle's plateau core
            float reach = siege.AttackDistance + (siege.Ranks - 1) * siege.RankSpacing + siege.MarchDistance + 4f;     // the armies' march
            return new Surroundings
            {
                Half = half,
                Wall = side * 0.5f - BrickCastle.TowerOut * Brick.Pitch * BrickScale,                                  // the outer wall faces
                Reach = reach,
                BandHalf = (siege.ArchersPerRank - 1) * 0.5f * siege.ColumnSpacing + 3f,                              // half the bands' width
                TreeOuter = half + math.max(TerrainParams.Margin + 0.5f * TerrainParams.Skirt, reach + 20f),          // the trees go out to the middle of the skirt
                FoliageOuter = half + math.max(TerrainParams.Margin + TerrainParams.Skirt, reach + 20f),              // the foliage to the foot of the hills
                OutlyingCore = half + TerrainParams.Skirt,                                                            // the copies' plateaus and skirts
            };
        }

        /// <summary>Half the width of every kind's bounding square (m; 0 for a rejected document).</summary>
        static float[] Radii(BrickAssembly[] kinds, float unit)
        {
            var radii = new float[kinds.Length];
            for (int k = 0; k < kinds.Length; k++)
                if (kinds[k] != null) radii[k] = math.max(math.cmax(math.abs(kinds[k].Min.xz)), math.cmax(math.abs(kinds[k].Max.xz))) * unit;
            return radii;
        }

        /// <summary>Scatters up to <see cref="Trees"/> trees around the castle: kinds, places and quarter turns from the seed, each
        /// standing outside the castle's plateau core, outside the bands the armies march and fire in, clear of the other trees
        /// and of the outlying copies, as many as fit <see cref="TreeBodies"/>; then levels the ground under them: a tree stands
        /// on the ground of its trunk, on a terrace levelled into the terrain under its crown. Returns whether the terrain was cut.</summary>
        bool ScatterTrees(int scene, Heightfield field, in Surroundings s)
        {
            m_TreePlacements.Clear();
            int kinds = 0;
            foreach (var k in m_TreeKinds) if (k != null && k.Parts.Count > 0) kinds++;
            if (Trees <= 0 || TreeBodies <= 0 || kinds == 0) return false;
            float unit = PieceCatalog.GridToUnity * BrickScale;
            uint seed = math.max(TreeSeed, 1u) * 0x9E3779B9u ^ (uint)(scene + 1) * 0x85EBCA6Bu;
            var rng = new Unity.Mathematics.Random(seed != 0u ? seed : 1u);
            var radii = Radii(m_TreeKinds, unit);
            int used = 0;
            for (int attempt = 0; attempt < Trees * 400 && m_TreePlacements.Count < Trees; attempt++)
            {
                int kind = rng.NextInt(m_TreeKinds.Length);
                if (m_TreeKinds[kind] == null || m_TreeKinds[kind].Parts.Count == 0) continue;
                float2 c = rng.NextFloat2(-s.TreeOuter, s.TreeOuter);
                float r = radii[kind] + 1f;
                if (!s.Clear(c, r, m_OutlyingCentres)) continue;
                bool clear = true;
                foreach (var p in m_TreePlacements) if (math.distance(c, p.Centre) < p.Radius + r) { clear = false; break; }
                if (!clear) continue;
                var assembly = m_TreeKinds[kind];
                if (used + assembly.Parts.Count > TreeBodies) continue;
                used += assembly.Parts.Count;
                m_TreePlacements.Add(new TreePlacement { Kind = kind, Centre = c, Turns = rng.NextInt(4), Radius = radii[kind] });
            }
            if (m_TreePlacements.Count == 0) return false;

            // the ground under every trunk, then a terrace under every crown blending out over 6 m, then the trunks' own squares
            // levelled again exactly (a neighbour's blend may have reached one); the level ground around the castle is left as it is
            bool cut = false;
            for (int i = 0; i < m_TreePlacements.Count; i++)
            {
                var p = m_TreePlacements[i];
                p.Ground = field != null ? field.MeanHeight(p.Centre - TrunkSquare, p.Centre + TrunkSquare) : 0f;
                m_TreePlacements[i] = p;
            }
            if (field != null)
            {
                foreach (var p in m_TreePlacements)
                {
                    float2 lo = p.Centre - p.Radius - 1f, hi = p.Centre + p.Radius + 1f;
                    if (math.abs(field.MeanHeight(lo, hi) - p.Ground) < 1e-3f && math.abs(field.MaxOver(lo, hi) - p.Ground) < 1e-3f) continue;
                    field.Flatten(lo, hi, p.Ground, 6f);
                    cut = true;
                }
                if (cut) foreach (var p in m_TreePlacements) field.Flatten(p.Centre - TrunkSquare, p.Centre + TrunkSquare, p.Ground, TerraceBlend);
            }
            return cut;
        }

        /// <summary>Builds the trees: one island per tree, snapped like the castle, drawn with the piece models, tinted from the
        /// documents' colours.</summary>
        void BuildTrees()
        {
            m_FirstTree = m_World.BodyCount;
            m_TreePieces = 0;
            float unit = PieceCatalog.GridToUnity * BrickScale;
            for (int i = 0; i < m_TreePlacements.Count; i++)
            {
                var p = m_TreePlacements[i];
                var assembly = m_TreeTurned[p.Kind, p.Turns] ?? (m_TreeTurned[p.Kind, p.Turns] = m_TreeKinds[p.Kind].Turned(p.Turns));
                var spec = new AssemblySpec
                {
                    Scale = BrickScale, Density = TreeMass / (2f * 3f * 1.2f * unit * unit * unit), Friction = BrickFriction, Margin = AvbdGpuConstants.CollisionMargin,
                    Clearance = AssemblySpec.DefaultClearance, Origin = new float3(p.Centre.x, p.Ground, p.Centre.y),
                };
                var bodies = AssemblyBuilder.Build(m_World, assembly, spec);
                if (Snap) AssemblyBuilder.AddSnapJoints(m_World, assembly, bodies, spec, SnapFractureLateral, SnapFractureTension, worldJoints: false);   // a tree stands on the ground by friction
                m_World.SleepRange(bodies.First, bodies.Count);
                p.First = bodies.First; p.Count = bodies.Count;
                m_TreePlacements[i] = p;
                m_TreePieces += bodies.Count;
                if (m_Tints.Length < bodies.First + bodies.Count) System.Array.Resize(ref m_Tints, bodies.First + bodies.Count);
                for (int b = 0; b < bodies.Count; b++)
                {
                    uint rgb = assembly.Parts[bodies.PartOfBody[b]].Rgb;
                    m_Tints[bodies.First + b] = AvbdGpuRenderer.Tint(new Color32((byte)(rgb >> 16), (byte)(rgb >> 8), (byte)rgb, 255));
                }
                foreach (var g in bodies.Groups)
                {
                    var mesh = PieceMesh(ref PieceMeshes, g.Piece);
                    if (mesh != null) m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = mesh, Scale = BrickScale, Offset = g.MeshOffset, Start = g.Start, Count = g.Count });
                }
            }
        }

        /// <summary>Scatters <see cref="FoliagePerTree"/> clumps of foliage per tree planted the way the trees are scattered (a seed of
        /// their own, out to the foot of the hills), each clear of the trees' crowns and of the other clumps by its bounding square,
        /// as many as fit <see cref="FoliageBodies"/>. A clump stands on the ground as it is where that is level within a cell of its
        /// footprint (the samples that shape the surface under it), and on a terrace of its own where it is not: its footprint and a
        /// cell around it levelled at the footprint's mean height, blending out over <see cref="TerraceBlend"/>. A terrace must reach
        /// neither another clump's level cell nor a tree's trunk square, so a clump with one stands two cells and a blend from every
        /// other clump. Returns whether the terrain was cut.</summary>
        bool ScatterFoliage(int scene, Heightfield field, in Surroundings s)
        {
            m_FoliagePlacements.Clear();
            int count = m_TreePlacements.Count * math.max(FoliagePerTree, 0);
            int kinds = 0;
            foreach (var k in m_FoliageKinds) if (k != null && k.Parts.Count > 0) kinds++;
            if (count <= 0 || FoliageBodies <= 0 || kinds == 0) return false;
            float unit = PieceCatalog.GridToUnity * BrickScale;
            float cell = field != null ? math.cmax(field.Cell) : 0f;
            float apart = 2f * cell + TerraceBlend;                       // between the footprints of two clumps when either has a terrace
            uint seed = math.max(TreeSeed, 1u) * 0xC2B2AE35u ^ (uint)(scene + 1) * 0x27D4EB2Fu;
            var rng = new Unity.Mathematics.Random(seed != 0u ? seed : 1u);
            var radii = Radii(m_FoliageKinds, unit);
            int used = 0;
            for (int attempt = 0; attempt < count * 400 && m_FoliagePlacements.Count < count; attempt++)
            {
                int kind = rng.NextInt(m_FoliageKinds.Length);
                if (m_FoliageKinds[kind] == null || m_FoliageKinds[kind].Parts.Count == 0) continue;
                float2 c = rng.NextFloat2(-s.FoliageOuter, s.FoliageOuter);
                float r = radii[kind];
                if (!s.Clear(c, r + 1f, m_OutlyingCentres)) continue;
                bool terraced = false;
                if (field != null) { field.SampleRange(c - r - cell, c + r + cell, out float lo, out float hi); terraced = hi - lo > 1e-3f; }
                bool clear = true;
                foreach (var t in m_TreePlacements)
                {
                    float d = math.cmax(math.abs(c - t.Centre));
                    if (d < t.Radius + r + 1f || (terraced && d < TrunkSquare + r + cell + TerraceBlend)) { clear = false; break; }
                }
                if (clear) foreach (var f in m_FoliagePlacements)
                    if (math.cmax(math.abs(c - f.Centre)) < f.Radius + r + (terraced || f.Terraced ? apart : 1f)) { clear = false; break; }
                if (!clear) continue;
                var assembly = m_FoliageKinds[kind];
                if (used + assembly.Parts.Count > FoliageBodies) continue;
                used += assembly.Parts.Count;
                float ground = field != null ? field.MeanHeight(c - r, c + r) : 0f;
                m_FoliagePlacements.Add(new FoliagePlacement { Kind = kind, Centre = c, Turns = rng.NextInt(4), Radius = r, Terraced = terraced, Ground = ground });
            }

            // the terraces: none reaches the ground under another clump or a tree, so every clump's ground is final
            bool cut = false;
            foreach (var f in m_FoliagePlacements)
            {
                if (!f.Terraced) continue;
                field.Flatten(f.Centre - f.Radius - cell, f.Centre + f.Radius + cell, f.Ground, TerraceBlend);
                cut = true;
            }
            return cut;
        }

        /// <summary>Builds the clumps in clusters, the cells of a <see cref="FoliageGrid"/>-square grid over the ground they are scattered
        /// on: the clumps of a cell go into one assembly whose bodies <see cref="AssemblyBuilder.Build"/> lays out by piece, so that a
        /// cluster is drawn in as many ranges as it has kinds of plate (a hundred and twenty clumps would otherwise be six hundred draws,
        /// each with its shadow proxy) and culled on its own bounds. Every clump sleeps as an island of its own, snapped like the trees.</summary>
        void BuildFoliage(in Surroundings s)
        {
            m_FirstFoliage = m_World.BodyCount;
            m_FoliagePieces = 0;
            m_FoliageClusters.Clear();
            if (m_FoliagePlacements.Count == 0) return;
            float unit = PieceCatalog.GridToUnity * BrickScale;
            var spec = new AssemblySpec
            {
                Scale = BrickScale, Density = FoliageMass / (2f * 3f * 1.2f * unit * unit * unit), Friction = BrickFriction, Margin = AvbdGpuConstants.CollisionMargin,
                Clearance = AssemblySpec.DefaultClearance, Origin = float3.zero,   // every clump's place goes into its parts' positions
            };
            float cellSize = 2f * s.FoliageOuter / FoliageGrid;
            var cells = new List<int>[FoliageGrid * FoliageGrid];
            for (int i = 0; i < m_FoliagePlacements.Count; i++)
            {
                int2 cell = math.clamp((int2)math.floor((m_FoliagePlacements[i].Centre + s.FoliageOuter) / cellSize), 0, FoliageGrid - 1);
                int k = cell.y * FoliageGrid + cell.x;
                (cells[k] ??= new List<int>()).Add(i);
            }
            foreach (var clumps in cells)
            {
                if (clumps == null) continue;
                var assembly = new BrickAssembly { Name = $"foliage cluster {m_FoliageClusters.Count}" };
                foreach (int i in clumps)
                {
                    var f = m_FoliagePlacements[i];
                    var turned = m_FoliageKinds[f.Kind].Turned(f.Turns);
                    float3 at = new float3(f.Centre.x, f.Ground, f.Centre.y) / unit;
                    f.Cluster = m_FoliageClusters.Count; f.PartStart = assembly.Parts.Count; f.Count = turned.Parts.Count;
                    m_FoliagePlacements[i] = f;
                    foreach (var p in turned.Parts)
                        assembly.Parts.Add(new AssemblyPart { Id = i + "/" + p.Id, Piece = p.Piece, Position = p.Position + at, Rotation = p.Rotation, Rgb = p.Rgb });
                }
                var bodies = AssemblyBuilder.Build(m_World, assembly, spec);
                if (Snap) AssemblyBuilder.AddSnapJoints(m_World, assembly, bodies, spec, SnapFractureLateral, SnapFractureTension, worldJoints: false);   // no two clumps overlap, so no snap crosses between them
                foreach (int i in clumps)   // one island per clump, body by body: a clump's bodies lie between its neighbours'
                {
                    var f = m_FoliagePlacements[i];
                    int island = int.MaxValue;
                    for (int j = 0; j < f.Count; j++) island = math.min(island, bodies.BodyOfPart[f.PartStart + j]);
                    for (int j = 0; j < f.Count; j++) m_World.SleepRange(bodies.BodyOfPart[f.PartStart + j], 1, island);
                }
                if (m_Tints.Length < bodies.First + bodies.Count) System.Array.Resize(ref m_Tints, bodies.First + bodies.Count);
                for (int i = 0; i < assembly.Parts.Count; i++)
                {
                    uint rgb = assembly.Parts[i].Rgb;
                    m_Tints[bodies.BodyOfPart[i]] = AvbdGpuRenderer.Tint(new Color32((byte)(rgb >> 16), (byte)(rgb >> 8), (byte)rgb, 255));
                }
                foreach (var g in bodies.Groups)
                {
                    var mesh = PieceMesh(ref PieceMeshes, g.Piece);
                    if (mesh != null) m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = mesh, Scale = BrickScale, Offset = g.MeshOffset, Start = g.Start, Count = g.Count });
                }
                m_FoliageClusters.Add(new FoliageCluster { Assembly = assembly, Bodies = bodies });
                m_FoliagePieces += bodies.Count;
            }
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
            if (kb.fKey.wasPressedThisFrame) { Trees = Trees >= 30 ? 0 : Trees + 10; RecreateWorld(); }
#endif
        }

        /// <summary>The HUD line on the trees and the foliage asleep around the castle.</summary>
        string AroundText()
        {
            string trees = $"<color=#a8d878>{m_TreePlacements.Count} trees</color> of {m_TreeDocuments.Length} kinds";
            if (m_FoliagePlacements.Count == 0)
                return trees + $" asleep around the castle ({m_TreePieces} pieces of the construction pack, {TreeMass:F2} kg per 2 x 3 brick)" +
                       (FoliagePerTree > 0 && m_FoliageDocuments.Length == 0 ? " - no foliage documents" : "") + "\n";
            int terraced = 0;
            foreach (var f in m_FoliagePlacements) if (f.Terraced) terraced++;
            return trees + $" and <color=#c8e090>{m_FoliagePlacements.Count} clumps of foliage</color> of {m_FoliageDocuments.Length} kinds asleep around the castle " +
                   $"({m_TreePieces + m_FoliagePieces} pieces; {m_FoliageClusters.Count} clusters, {terraced} clumps on terraces)\n";
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
                (m_TreePlacements.Count > 0 ? AroundText() : Trees > 0 ? "no trees (none fit, or no documents)\n" : "no trees (F)\n") +
                TerrainText(m_Plateau) + "\n\n" +
                StatsText() + "\n\n" +
                "1-0 castle size  , . prev/next  R rebuild  J snap bricks on/off  T terrain  O outlying copies 0/4/8  F trees 0/10/20/30 (and their foliage)  U siege on/off  V volley  K auto volleys  X retire the dead and spent now  Space pause  N step\n" +
                "F1 contacts  F2 colour mode  F5 joints  F6 collision boxes  F7 shadows  F8 sleep on/off  +/- iterations  [ ] substeps  B/Enter cannonball  G gravity  H hide HUD\n" +
                "LMB drag  RMB orbit  MMB pan  wheel / Q E zoom  W A S D orbit";
        }
    }
}
