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
    /// <summary>A siege the player commands (Siege.unity): a castle from a brick-assembly document on the Castle scene's hills with its
    /// trees and foliage, a garrison of AI defenders on the castle's posts and the player's army formed up on one side, all from a
    /// <see cref="SiegeConfig"/> asset (the number keys choose between the assets of Resources). Left click a unit to select its kind
    /// (green plates under every unit of it), right click a block to attack it or the ground to walk there; the units walk in straight
    /// lines keeping their formation, shoot within their range, and their hits break snaps and blast bricks (the unit configs' hit
    /// effects). Bows, spells and explosions are drawn attached to the bodies (<see cref="AttachmentRenderer"/>).</summary>
    public class SiegeDemo : DemoBase
    {
        [Tooltip("The 27 piece models in catalog order (PieceCatalog.Pieces; Phys / Assign Siege Meshes fills them in). Unset pieces are drawn as boxes.")]
        public Mesh[] PieceMeshes;
        [Tooltip("The siege configs (castle, rosters, side); empty: every SiegeConfig asset of the Resources folder below, by name.")]
        public SiegeConfig[] Configs = new SiegeConfig[0];
        [Tooltip("Resources folder searched for SiegeConfig assets when none are assigned ('' = every Resources folder).")]
        public string ConfigFolder = "";
        [Tooltip("Solver metres per model metre (the pieces and the figures alike).")]
        public float BrickScale = 5f;
        [Tooltip("Mass of a 2 x 3 brick (kg); every piece gets the same density.")]
        public float BrickMass = 1f;
        public float BrickFriction = 0.6f;
        [Tooltip("Clearance of the collision boxes from the pieces' footprints, per side, in model metres.")]
        public float Clearance = AssemblySpec.DefaultClearance;
        [Tooltip("Snap the pieces together with hard ball-socket joints that break under load (-avbd-snap / -avbd-dry).")]
        public bool Snap = true;
        [Tooltip("Sideways force (N) that breaks a snap connection.")]
        public float SnapFractureLateral = 800f;
        [Tooltip("Upward pull (N) that breaks a snap connection.")]
        public float SnapFractureTension = 300f;
        [Tooltip("Snap the pieces on the ground to the world as well.")]
        public bool WorldSnaps = true;
        public bool Shadows = true;
        public bool BoxShadows = true;
        [Tooltip("Cannonball (B / Enter): cube size (model metres), mass (kg) and speed (model metres per second).")]
        public float ShotCube = 0.12f;
        public float ShotMass = 20f;
        public float ShotVelocity = 24f;
        [Tooltip("Trees and foliage around the castle, as in the castle demo (key F cycles the trees 0 / 10 / 20 / 30; -avbd-trees n, -avbd-foliage n).")]
        public string TreeFolder = "Trees";
        public int Trees = 20;
        public int TreeBodies = 49152;
        public uint TreeSeed = 1;
        public float TreeMass = 0.25f;
        public string FoliageFolder = "Foliage";
        public int FoliagePerTree = 6;
        public int FoliageBodies = 81920;
        public float FoliageMass = 1f;
        [Tooltip("Tunables shared by every unit.")]
        public BattleSettings BattleParams = BattleSettings.Default;
        [Tooltip("The plates drawn under the selected units: colour (alpha = opacity) and their size relative to the unit's footprint.")]
        public Color32 SelectionColor = new Color32(60, 220, 90, 150);
        public float PlateMargin = 1.3f;

        /// <summary>A hit effect's visual in flight: which config, where, since when.</summary>
        struct Effect
        {
            public HitEffectConfig Config;
            public float3 Position;
            public int Birth;
            public float Yaw;
        }

        SiegeConfig[] m_Configs = new SiegeConfig[0];
        string m_StartConfig;
        int m_AutoAttackFrame = -1, m_Frame;
        BrickAssembly m_Assembly;
        AssemblyBodies m_Bodies;
        AssemblySpec m_Spec;
        AssemblyOccupancy m_Occupancy;
        AssemblyBuilder.Diagnostics m_Diagnostics;
        string m_Error;
        int m_SnapJoints;
        float m_Plateau;
        uint[] m_Tints;
        Vegetation m_Vegetation;
        Battle m_Battle;
        UnitConfig[] m_Types = new UnitConfig[0];
        AttachmentRenderer m_Attachments;
        Mesh m_Plate;
        readonly List<Effect> m_Effects = new List<Effect>();
        int m_Selected = -1;
        string m_LastOrder = "", m_LastDeath = "";
        Vector2 m_LeftPress, m_RightPress;
        static readonly Color32 s_Iron = new Color32(70, 72, 78, 255);

        void Reset()
        {
            MaxBodies = 40960;
        }

        public Battle Battle => m_Battle;
        public SiegeConfig Config => m_Scene < m_Configs.Length ? m_Configs[m_Scene] : null;
        public SiegeConfig[] LoadedConfigs => m_Configs;
        public BrickAssembly Assembly => m_Assembly;
        public AssemblyBodies Bodies => m_Bodies;
        public AssemblySpec Spec => m_Spec;
        public AssemblyOccupancy Occupancy => m_Occupancy;
        public float Plateau => m_Plateau;
        public Vegetation Vegetation => m_Vegetation;
        public AttachmentRenderer Attachments => m_Attachments;
        /// <summary>The unit configs of the current battle, in the order of <see cref="Battle.Types"/>.</summary>
        public UnitConfig[] UnitTypes => m_Types;
        /// <summary>The selected unit type (index into <see cref="UnitTypes"/>), -1 for none.</summary>
        public int SelectedType => m_Selected;
        public string Error => m_Error;
        public int EffectCount => m_Effects.Count;
        /// <summary>The HUD's text (tests).</summary>
        public string Hud => HudText();

        protected override int SceneCount => math.max(1, m_Configs.Length);
        protected override string SceneName(int index) => index < m_Configs.Length ? ConfigName(m_Configs[index]) : "no config";
        protected override float HudHeight => 330f;
        protected override float3 ShotSize => ShotCube * BrickScale;
        protected override float ShotDensity => ShotMass / math.pow(ShotCube * BrickScale, 3f);
        protected override float ShotSpeed => ShotVelocity * math.sqrt(BrickScale);
        protected override float ShotDistance => 0.5f * BrickScale;
        protected override float DragStiffness => 5000f * math.max(m_Spec.BrickMass, 0.1f);
        protected override float DefaultTileStep => Brick.BodyHeight / 3f * BrickScale;
        protected override int SettleSteps => 180;
        protected override int SettleSubsteps => 1;
        /// <summary>The left button selects, the right one orders: no drag joint.</summary>
        protected override bool MouseDrag => false;

        static string ConfigName(SiegeConfig c) => string.IsNullOrEmpty(c.DisplayName) ? c.name : c.DisplayName;

        Vegetation.Settings VegetationSettings => new Vegetation.Settings
        {
            TreeFolder = TreeFolder, FoliageFolder = FoliageFolder, Trees = Trees, TreeBodies = TreeBodies, FoliagePerTree = FoliagePerTree, FoliageBodies = FoliageBodies,
            Seed = TreeSeed, TreeMass = TreeMass, FoliageMass = FoliageMass, Scale = BrickScale, Friction = BrickFriction,
            Snap = Snap, SnapFractureLateral = SnapFractureLateral, SnapFractureTension = SnapFractureTension,
        };

        /// <summary>Index of the config with the asset or display name, or -1.</summary>
        public int IndexOf(string config)
        {
            for (int i = 0; i < m_Configs.Length; i++) if (m_Configs[i].name == config || m_Configs[i].DisplayName == config) return i;
            return -1;
        }

        void LoadConfigs()
        {
            if (Configs != null && Configs.Length > 0) { m_Configs = System.Array.FindAll(Configs, c => c != null); return; }
            var list = new List<SiegeConfig>(Resources.LoadAll<SiegeConfig>(ConfigFolder ?? ""));
            list.Sort((a, b) => string.CompareOrdinal(a.name, b.name));
            m_Configs = list.ToArray();
            if (m_Configs.Length == 0) Debug.LogWarning($"SiegeDemo: no SiegeConfig assets in Resources/{ConfigFolder}");
        }

        /// <summary>The distinct unit configs of a siege (attackers' order, then the defenders') with the pool sizes: every unit of the
        /// rosters, and six projectiles per unit (at least 64) so that a shot every few seconds never runs the pool dry.</summary>
        static UnitConfig[] Roster(SiegeConfig cfg, out int[] units, out int[] projectiles)
        {
            var types = new List<UnitConfig>();
            var counts = new List<int>();
            void Add(SiegeConfig.Roster[] roster)
            {
                if (roster == null) return;
                foreach (var r in roster)
                {
                    if (r.Unit == null || r.Count <= 0) continue;
                    int i = types.IndexOf(r.Unit);
                    if (i < 0) { types.Add(r.Unit); counts.Add(0); i = types.Count - 1; }
                    counts[i] += r.Count;
                }
            }
            Add(cfg.Attackers); Add(cfg.Defenders);
            units = counts.ToArray();
            projectiles = new int[units.Length];
            for (int i = 0; i < units.Length; i++) projectiles[i] = math.max(64, units[i] * 6);
            return types.ToArray();
        }

        static int PoolBodies(SiegeConfig cfg)
        {
            Roster(cfg, out var units, out var projectiles);
            int n = 0;
            for (int i = 0; i < units.Length; i++) n += units[i] + projectiles[i];
            return n;
        }

        protected override AvbdGpuConfig CreateConfig()
        {
            if (m_Configs.Length == 0) LoadConfigs();
            int pools = 0;
            foreach (var c in m_Configs) pools = math.max(pools, PoolBodies(c));
            pools = math.max(pools, 1024);
            // the castle and the armies awake, the trees and the foliage asleep beyond
            int total = MaxBodies + pools + Vegetation.ReservedBodies(VegetationSettings);
            var cfg = AvbdGpuConfig.ForBodies(total, MaxBodies + pools);
            cfg.MaxJoints = total * 12;
            cfg.MaxLinks = cfg.MaxJoints * 2 + 4096;
            cfg.MaxSpawns = math.max(cfg.MaxSpawns, pools);
            return cfg;
        }

        protected override void ParseArgs(string[] args)
        {
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == "-avbd-snap") Snap = true;
                if (args[i] == "-avbd-dry") Snap = false;
                if (args[i] == "-avbd-config" && i + 1 < args.Length) m_StartConfig = args[i + 1];
                if (args[i] == "-avbd-attack" && i + 1 < args.Length && int.TryParse(args[i + 1], out int attackFrame)) m_AutoAttackFrame = attackFrame;
                if (args[i] == "-avbd-trees" && i + 1 < args.Length && int.TryParse(args[i + 1], out int trees)) Trees = trees;
                if (args[i] == "-avbd-foliage" && i + 1 < args.Length && int.TryParse(args[i + 1], out int foliage)) FoliagePerTree = foliage;
                if (args[i] == "-avbd-noshadows") Shadows = false;
                if (args[i] == "-avbd-meshshadows") BoxShadows = false;
            }
        }

        protected override void Configure()
        {
            BattleParams = BattleParams.WithDefaults();
            if (m_Configs.Length == 0) LoadConfigs();
            m_Renderer.Shadows = Shadows;
            m_Renderer.BoxShadows = BoxShadows;
            m_Renderer.DrawJoints = false;
            m_Vegetation = new Vegetation(VegetationSettings, "SiegeDemo");
            m_Vegetation.LoadDocuments();
            m_Attachments?.Dispose();
            m_Attachments = new AttachmentRenderer(m_World) { Shadows = Shadows };
            if (m_Plate == null) m_Plate = AttachmentRenderer.BuildPlate();
            if (m_StartConfig != null)
            {
                int index = IndexOf(m_StartConfig);
                if (index >= 0) StartScene = index;
                else Debug.LogWarning($"SiegeDemo: no SiegeConfig '{m_StartConfig}'");
                m_StartConfig = null;
            }
        }

        void OnDestroy()
        {
            m_Attachments?.Dispose(); m_Attachments = null;
            if (m_Plate != null) Destroy(m_Plate);
        }

        // ------------------------------------------------------------------------------------------------ the scene

        protected override void BuildScene(int index, out float3 cameraTarget, out float cameraDistance)
        {
            m_Assembly = null; m_Bodies = null; m_Occupancy = null; m_Error = null; m_SnapJoints = 0; m_Diagnostics = default; m_Plateau = 0f;
            m_Battle = null; m_Types = new UnitConfig[0]; m_Effects.Clear(); m_Selected = -1; m_LastOrder = "";
            cameraTarget = new float3(0f, 2f * BrickScale, 0f);
            cameraDistance = 20f * BrickScale;
            if (m_Tints == null || m_Tints.Length < 1024) m_Tints = new uint[1024];
            var cfg = index < m_Configs.Length ? m_Configs[index] : null;
            if (cfg == null) m_Error = "no SiegeConfig assets in Resources";
            else if (cfg.Castle == null) m_Error = $"{cfg.name}: no castle document";
            else
            {
                try { m_Assembly = BrickAssembly.Parse(cfg.Castle.text); }
                catch (BrickAssemblyException e) { m_Error = e.Message; Debug.LogError($"SiegeDemo: {cfg.Castle.name}.json rejected: {e.Message}"); }
            }

            // the terrain with a plateau under the castle's footprint and the level margin around it (the armies stand on the flat)
            float unit = PieceCatalog.GridToUnity * BrickScale;
            float2 half = m_Assembly != null ? m_Assembly.Extent.xz * 0.5f * unit : new float2(4f * BrickScale);
            var field = CreateTerrain();
            m_Plateau = AddGround(field, BrickFriction, half, 2f * unit, out int groundBody);
            m_Tints[groundBody] = AvbdGpuRenderer.Tint(new Color32(78, 112, 58, 255));
            float3 origin = new float3(0f, m_Plateau, 0f);
            if (m_Assembly != null)
            {
                m_Occupancy = m_Assembly.OccupancyOrDerived();
                if (!m_Occupancy.ExplicitPosts) m_Occupancy.DerivePosts(AssemblyOccupancy.PostRules.Default);
                m_Diagnostics = AssemblyBuilder.Diagnose(m_Assembly);
                if (!m_Diagnostics.Clean)
                    Debug.LogWarning($"SiegeDemo: {cfg.Castle.name}.json: {m_Diagnostics.Floating} floating, {m_Diagnostics.PoorlySupported} poorly supported, {m_Diagnostics.Intersections} intersecting pieces ({m_Diagnostics.Sample})");
                float3 centre = (m_Assembly.Min + m_Assembly.Max) * 0.5f;
                m_Spec = new AssemblySpec
                {
                    Scale = BrickScale, Density = BrickMass / (2f * 3f * 1.2f * unit * unit * unit), Friction = BrickFriction, Margin = AvbdGpuConstants.CollisionMargin,
                    Clearance = Clearance, Origin = new float3(-centre.x * unit, m_Plateau, -centre.z * unit),
                };
                origin = m_Spec.Origin;
                m_Bodies = AssemblyBuilder.Build(m_World, m_Assembly, m_Spec);
                if (Snap) m_SnapJoints = AssemblyBuilder.AddSnapJoints(m_World, m_Assembly, m_Bodies, m_Spec, SnapFractureLateral, SnapFractureTension, WorldSnaps);
                int n = m_Bodies.Count;
                if (m_Tints.Length < m_Bodies.First + n) System.Array.Resize(ref m_Tints, m_Bodies.First + n);
                for (int i = 0; i < n; i++)
                {
                    uint rgb = m_Assembly.Parts[m_Bodies.PartOfBody[i]].Rgb;
                    m_Tints[m_Bodies.First + i] = AvbdGpuRenderer.Tint(new Color32((byte)(rgb >> 16), (byte)(rgb >> 8), (byte)rgb, 255));
                }
                foreach (var g in m_Bodies.Groups)
                {
                    var mesh = PieceMesh(ref PieceMeshes, g.Piece);
                    if (mesh != null) m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = mesh, Scale = BrickScale, Offset = g.MeshOffset, Start = g.Start, Count = g.Count });
                }
                m_World.Params.Substeps = n > 20000 ? 1 : n > 8000 ? 2 : 3;
            }

            // the attacked side and the formation's ground, kept clear of trees
            float3 outward = cfg != null ? (float3)cfg.Outward : new float3(0, 0, -1);
            float faceDistance = math.abs(outward.x) > 0.5f ? half.x : half.y;
            var ranks = new List<Formations.Rank>();
            UnitArchetype[] archetypes = new UnitArchetype[0];
            int[] unitCaps = new int[0], projectileCaps = new int[0];
            if (cfg != null)
            {
                m_Types = Roster(cfg, out unitCaps, out projectileCaps);
                archetypes = new UnitArchetype[m_Types.Length];
                for (int t = 0; t < m_Types.Length; t++) archetypes[t] = m_Types[t].ToArchetype(BrickScale, AvbdGpuConstants.CollisionMargin);
                foreach (var r in cfg.Attackers)
                {
                    if (r.Unit == null || r.Count <= 0) continue;
                    int t = System.Array.IndexOf(m_Types, r.Unit);
                    ranks.Add(new Formations.Rank { Type = t, Count = r.Count, Width = archetypes[t].BoxSize.x, Depth = archetypes[t].BoxSize.z, Range = archetypes[t].Range });
                }
            }
            float formationDepth = cfg != null ? Formations.Depth(ranks, cfg.RankSpacing, math.max(cfg.MaxColumns, 1)) : 0f;
            float formationDistance = cfg != null ? cfg.FormationDistance : 30f;
            if (field != null && formationDistance + formationDepth + 8f > TerrainParams.Margin)
                Debug.LogWarning($"SiegeDemo: the formation ({formationDistance:F0} m out, {formationDepth:F0} m deep) reaches past the level margin of {TerrainParams.Margin:F0} m: the rear ranks stand on the slope");
            var around = ExclusionOf(half, unit, outward, faceDistance, formationDistance, formationDepth, cfg);
            m_Vegetation.Params = VegetationSettings;
            if (m_Vegetation.Scatter(index, field, around)) m_World.UpdateTerrain();
            m_Vegetation.Build(m_World, m_Renderer, ref m_Tints, piece => PieceMesh(ref PieceMeshes, piece), around);

            // the battle: pools after everything asleep, drawn with the unit and projectile models wherever they go
            if (cfg != null && m_Types.Length > 0)
            {
                m_Battle = new Battle(m_World, archetypes, unitCaps, projectileCaps, BattleParams)
                {
                    Terrain = m_World.Terrain, Occupancy = m_Occupancy, OccupancyOrigin = origin, OccupancyUnit = unit,
                    OnUnitSpawned = OnUnitSpawned, OnProjectileSpawned = OnProjectileSpawned, OnRetiring = OnRetiring, OnImpact = OnImpact, OnUnitKilled = OnUnitKilled,
                };
                for (int t = 0; t < m_Types.Length; t++)
                {
                    var a = archetypes[t];
                    var pool = m_Battle.UnitPools[t];
                    if (pool != null && m_Types[t].BodyMesh != null)
                        m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = m_Types[t].BodyMesh, Scale = BrickScale, Offset = a.MeshOffset, Start = pool.Start, Count = pool.Capacity, NoCulling = true });
                    var pools = m_Battle.ProjectilePools[t];
                    var meshes = m_Types[t].ProjectileMeshes;
                    for (int v = 0; v < pools.Length; v++)
                    {
                        var mesh = meshes != null && meshes.Length > 0 ? meshes[math.min(v, meshes.Length - 1)] : null;
                        if (pools[v] != null && mesh != null)
                            m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = mesh, Scale = BrickScale, Offset = a.ProjectileMeshOffset, Start = pools[v].Start, Count = pools[v].Capacity, NoCulling = true });
                    }
                }
                SpawnDefenders(cfg, origin, unit);
                SpawnAttackers(cfg, ranks, outward, faceDistance, formationDistance);
            }
            if (m_Tints.Length < m_World.BodyCount) System.Array.Resize(ref m_Tints, m_World.BodyCount);   // the pools' retired slots included
            m_Renderer.SetTints(m_Tints, 0, m_World.BodyCount);

            // the camera behind the attackers, looking over them at the castle: the target midway between the castle and the formation
            float3 extent = m_Assembly != null ? m_Assembly.Extent * unit : new float3(8f * BrickScale);
            float reach = faceDistance + formationDistance + formationDepth;
            cameraTarget = outward * (0.5f * (faceDistance + formationDistance + 0.5f * formationDepth)) + new float3(0f, m_Plateau + 0.25f * extent.y, 0f);
            cameraDistance = 1.1f * reach + 10f;
            if (m_Camera != null) { m_Camera.Yaw = cfg != null ? (cfg.AttackSide == 1 ? -90f : cfg.AttackSide == 2 ? 180f : cfg.AttackSide == 3 ? 90f : 0f) : 0f; m_Camera.Pitch = 30f; }
        }

        /// <summary>The ground the trees keep clear of: the castle's plateau core and the attackers' band from the face out past the
        /// formation; the trees go out to the middle of the skirt, the foliage to the foot of the hills (at least 60 m).</summary>
        Vegetation.Exclusion ExclusionOf(float2 half, float unit, float3 outward, float faceDistance, float formationDistance, float formationDepth, SiegeConfig cfg)
        {
            float core = math.cmax(half) + 2f * unit;
            float across = cfg != null ? ((math.max(cfg.MaxColumns, 1) - 1) * 0.5f * cfg.ColumnSpacing + 8f) : 20f;
            float2 o = outward.xz, along = new float2(outward.z, -outward.x);
            float2 p1 = o * (faceDistance - 2f), p2 = o * (faceDistance + formationDistance + formationDepth + 8f);
            float2 w = math.abs(along) * across;
            var band = new float4(math.min(p1, p2) - w, math.max(p1, p2) + w);
            return new Vegetation.Exclusion
            {
                CoreHalf = core,
                Bands = new List<float4> { band },
                TreeOuter = core + math.max(TerrainParams.Margin + 0.5f * TerrainParams.Skirt, 60f),
                FoliageOuter = core + math.max(TerrainParams.Margin + TerrainParams.Skirt, 60f),
            };
        }

        /// <summary>The garrison on the castle's posts, spread over them (the wall posts first), each facing its post's way; without a
        /// castle, in a small square about the origin.</summary>
        void SpawnDefenders(SiegeConfig cfg, float3 origin, float unit)
        {
            int total = cfg.DefenderCount;
            if (total == 0) return;
            var posts = m_Occupancy?.Posts;
            // the wall posts (first in the list) taken first, spread evenly; the ground posts only for the rest
            int walls = 0;
            if (posts != null) foreach (var p in posts) if (p.OnWall) walls++;
            var assignment = new int[total];
            if (posts != null && posts.Count > 0)
            {
                int onWalls = math.min(total, walls);
                var wallPicks = Formations.AssignPosts(walls, onWalls);
                var groundPicks = Formations.AssignPosts(posts.Count - walls, total - onWalls);
                for (int i = 0; i < total; i++) assignment[i] = i < onWalls ? wallPicks[i] : posts.Count - walls > 0 ? walls + groundPicks[i - onWalls] : wallPicks.Length > 0 ? wallPicks[i % wallPicks.Length] : 0;
            }
            int k = 0;
            foreach (var r in cfg.Defenders)
            {
                if (r.Unit == null) continue;
                int t = System.Array.IndexOf(m_Types, r.Unit);
                for (int i = 0; i < r.Count; i++, k++)
                {
                    float3 foot; float yaw;
                    if (posts != null && posts.Count > 0)
                    {
                        var post = posts[assignment[k]];
                        foot = origin + post.Position * unit; yaw = post.Yaw;
                    }
                    else
                    {
                        float2 xz = new float2((k % 6 - 2.5f) * 2.5f, (k / 6) * 2.5f);
                        foot = new float3(xz.x, m_Battle.GroundHeight(xz), xz.y); yaw = 0f;
                    }
                    if (m_Battle.SpawnUnit(t, Battle.Defenders, foot, yaw) < 0) Debug.LogWarning($"SiegeDemo: the {m_Types[t].name} pool is full");
                }
            }
        }

        /// <summary>The player's army in ranks on the attacked side, facing the castle, standing on the ground wherever it is.</summary>
        void SpawnAttackers(SiegeConfig cfg, List<Formations.Rank> ranks, float3 outward, float faceDistance, float formationDistance)
        {
            var placed = Formations.Ranks(ranks, outward, faceDistance, formationDistance, cfg.ColumnSpacing, cfg.RankSpacing, math.max(cfg.MaxColumns, 1));
            foreach (var (type, position, yaw) in placed)
            {
                float3 foot = new float3(position.x, m_Battle.GroundHeight(position.xz), position.z);
                if (m_Battle.SpawnUnit(type, Battle.Attackers, foot, yaw) < 0) Debug.LogWarning($"SiegeDemo: the {m_Types[type].name} pool is full");
            }
        }

        void SetTint(int body, Color32 color)
        {
            if (m_Tints == null || m_Tints.Length <= body) System.Array.Resize(ref m_Tints, math.max(body + 1, 1024));
            m_Tints[body] = AvbdGpuRenderer.Tint(color);
            m_Renderer.SetTints(m_Tints, body, 1);
        }

        void OnUnitSpawned(int body, int type)
        {
            var cfg = Config;
            int unit = m_Battle.UnitOfBody(body);
            bool defender = unit >= 0 && m_Battle.Units[unit].Team == Battle.Defenders;
            SetTint(body, defender && cfg != null && cfg.DefenderTint.a > 0 ? cfg.DefenderTint : m_Types[type].Tint);
        }

        void OnProjectileSpawned(int body, int type) => SetTint(body, m_Types[type].ProjectileColor);

        void OnRetiring(int body)
        {
            if (DragBody == body) ReleaseDrag();
        }

        void OnImpact(int type, float3 at)
        {
            var effect = m_Types[type].HitEffect;
            if (effect == null || effect.Mesh == null) return;
            m_Effects.Add(new Effect { Config = effect, Position = at, Birth = m_World.StepIndex, Yaw = (m_Effects.Count * 137) % 360 });
        }

        void OnUnitKilled(int unit, string cause)
        {
            var u = m_Battle.Units[unit];
            m_LastDeath = $"{(u.Team == Battle.Defenders ? "defender" : "attacker")} {m_Types[u.Type].name} ({cause}) at step {m_World.StepIndex}";
            Debug.Log("SiegeDemo: " + m_LastDeath);
        }

        protected override void OnShot(int body) => SetTint(body, s_Iron);

        /// <summary>Every attacker kind attacks the castle's highest piece (the -avbd-attack flag, for screenshots and benches).</summary>
        public void AttackTheCastle()
        {
            if (m_Battle == null || m_Bodies == null || m_World.ReadCount == 0 || Config == null) return;
            // the highest piece standing on the attacked face's line (not a brick blown into the air somewhere)
            float unit = PieceCatalog.GridToUnity * BrickScale;
            float3 outward = Config.Outward;
            float face = math.abs(outward.x) > 0.5f ? m_Assembly.Extent.x * 0.5f * unit : m_Assembly.Extent.z * 0.5f * unit;
            int target = -1; float top = float.NegativeInfinity;
            for (int i = 0; i < m_Bodies.Count && m_Bodies.First + i < m_World.ReadCount; i++)
            {
                float3 p = m_World.ReadPositions[m_Bodies.First + i].xyz;
                if (math.abs(math.dot(p, outward) - face) > 3f * unit || p.y > m_Plateau + 40f) continue;
                if (p.y > top) { top = p.y; target = m_Bodies.First + i; }
            }
            if (target < 0) return;
            for (int t = 0; t < m_Types.Length; t++) m_Battle.Attack(m_Battle.Select(Battle.Attackers, t), target);
            m_LastOrder = $"attack the top of the facing wall (body {target})";
            if (m_Selected < 0) SelectUnitType(0);   // the plates show in the screenshot
        }

        protected override void OnStep()
        {
            m_Frame++;
            if (m_AutoAttackFrame >= 0 && m_Frame == m_AutoAttackFrame) AttackTheCastle();
            m_Battle?.Tick(m_World.ReadPositions, m_World.ReadCount, m_World.ReadStep);
            for (int i = m_Effects.Count - 1; i >= 0; i--)
                if (m_World.StepIndex - m_Effects[i].Birth >= math.max(m_Effects[i].Config.LifeSteps, 1)) m_Effects.RemoveAt(i);
        }

        // ------------------------------------------------------------------------------------------------ the player

        /// <summary>Selects a unit type (all the live attackers of it get their plates); -1 clears the selection.</summary>
        public void SelectUnitType(int type)
        {
            m_Selected = m_Battle != null && type >= 0 && type < m_Battle.Types.Length ? type : -1;
        }

        /// <summary>The live attackers of the selected type (indices into <see cref="Battle.Units"/>).</summary>
        public List<int> SelectedUnits() => m_Battle != null && m_Selected >= 0 ? m_Battle.Select(Battle.Attackers, m_Selected) : new List<int>();

        /// <summary>What the left button does at a ray: a live attacker under it selects its type, anything else clears the selection.
        /// Returns the selected type.</summary>
        public int SelectAt(Ray ray)
        {
            int body = m_World.Pick(ray.origin, ray.direction, out _, out float dist);
            int unit = body >= 0 ? m_Battle?.UnitOfBody(body) ?? -1 : -1;
            if (unit >= 0 && m_Battle.Units[unit].Team == Battle.Attackers && m_Battle.Units[unit].State != UnitState.Dead && dist < RayTerrain(ray))
                SelectUnitType(m_Battle.Units[unit].Type);
            else SelectUnitType(-1);
            return m_Selected;
        }

        /// <summary>What the right button does at a ray for the selected units: a body under it (nearer than the ground) is attacked - a
        /// live attacker of the player's is walked to instead - and a ground point is walked to. Returns the order issued.</summary>
        public OrderKind OrderAt(Ray ray)
        {
            var units = SelectedUnits();
            if (m_Battle == null || units.Count == 0) return OrderKind.None;
            int body = m_World.Pick(ray.origin, ray.direction, out _, out float dist);
            float ground = RayTerrain(ray);
            if (body >= 0 && dist < ground)
            {
                int unit = m_Battle.UnitOfBody(body);
                if (unit >= 0 && m_Battle.Units[unit].Team == Battle.Attackers)
                {
                    float3 under = (float3)ray.GetPoint(dist);
                    m_Battle.Move(units, new float3(under.x, m_Battle.GroundHeight(under.xz), under.z));
                    m_LastOrder = $"move to the {m_Types[m_Battle.Units[unit].Type].name}";
                    return OrderKind.Move;
                }
                m_Battle.Attack(units, body);
                m_LastOrder = $"attack body {body}" + (unit >= 0 ? $" (a {m_Types[m_Battle.Units[unit].Type].name})" : m_Bodies != null && body >= m_Bodies.First && body < m_Bodies.First + m_Bodies.Count ? $" ({m_Assembly.Parts[m_Bodies.PartOfBody[body - m_Bodies.First]].Id})" : "");
                return OrderKind.Attack;
            }
            if (float.IsInfinity(ground)) return OrderKind.None;
            float3 point = (float3)ray.GetPoint(ground);
            m_Battle.Move(units, point);
            m_LastOrder = $"move to ({point.x:F0}, {point.z:F0})";
            return OrderKind.Move;
        }

        /// <summary>Distance along the ray to the ground (the heightfield marched in half cells and bisected, or the flat ground), or
        /// +infinity when the ray never meets it.</summary>
        public float RayTerrain(Ray ray)
        {
            float3 o = ray.origin, d = math.normalize((float3)ray.direction);
            var field = m_World.Terrain;
            if (field == null)
            {
                if (d.y >= -1e-6f) return o.y <= m_Plateau ? 0f : float.PositiveInfinity;
                return (m_Plateau - o.y) / d.y;
            }
            float step = 0.5f * math.cmin(field.Cell);
            float t = o.y > field.MaxHeight && d.y < 0f ? (field.MaxHeight - o.y) / d.y : 0f;
            float3 p = o + d * t;
            if (p.y <= field.Height(p.xz)) return t;
            for (float end = t + 4000f; t < end; )
            {
                float next = t + step;
                float3 q = o + d * next;
                if (q.y <= field.Height(q.xz))
                {
                    float lo = t, hi = next;
                    for (int i = 0; i < 12; i++)
                    {
                        float mid = 0.5f * (lo + hi);
                        float3 m = o + d * mid;
                        if (m.y <= field.Height(m.xz)) hi = mid; else lo = mid;
                    }
                    return hi;
                }
                if (d.y >= 0f && q.y > field.MaxHeight) break;   // climbing away above everything
                t = next;
            }
            return float.PositiveInfinity;
        }

        protected override void HandleSceneMouse()
        {
#if ENABLE_INPUT_SYSTEM
            var mouse = Mouse.current;
            var cam = Camera.main;
            if (mouse == null || cam == null || m_Battle == null) return;
            Vector2 mp = mouse.position.ReadValue();
            const float clickPixels = 5f;
            if (mouse.leftButton.wasPressedThisFrame) m_LeftPress = mp;
            if (mouse.leftButton.wasReleasedThisFrame && Vector2.Distance(mp, m_LeftPress) <= clickPixels) SelectAt(cam.ScreenPointToRay(mp));
            if (mouse.rightButton.wasPressedThisFrame) m_RightPress = mp;
            if (mouse.rightButton.wasReleasedThisFrame && Vector2.Distance(mp, m_RightPress) <= clickPixels) OrderAt(cam.ScreenPointToRay(mp));
#endif
        }

        protected override void HandleSceneKeys()
        {
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb.escapeKey.wasPressedThisFrame) SelectUnitType(-1);
            if (kb.jKey.wasPressedThisFrame) { Snap = !Snap; Load(m_Scene); }
            if (kb.f6Key.wasPressedThisFrame) m_Renderer.DrawCollisionBoxes = !m_Renderer.DrawCollisionBoxes;
            if (kb.f7Key.wasPressedThisFrame) { Shadows = !Shadows; m_Renderer.Shadows = Shadows; if (m_Attachments != null) m_Attachments.Shadows = Shadows; }
            if (kb.tKey.wasPressedThisFrame) CycleTerrain();
            if (kb.fKey.wasPressedThisFrame) { Trees = Trees >= 30 ? 0 : Trees + 10; RecreateWorld(); }
            if (kb.xKey.wasPressedThisFrame && m_Battle != null) { ReleaseDrag(); m_Battle.Purge(); }
#endif
        }

        // ------------------------------------------------------------------------------------------------ drawing

        /// <summary>The weapons in the units' hands (swinging after a shot), the spells on the projectiles' tips, the plates under the
        /// selected units and the hit effects in flight.</summary>
        protected override void OnRender(Camera camera)
        {
            if (m_Attachments == null || m_Battle == null) return;
            m_Attachments.Begin();
            int step = m_World.StepIndex;
            for (int i = 0; i < m_Battle.Units.Count; i++)
            {
                var u = m_Battle.Units[i];
                var cfg = m_Types[u.Type];
                var a = m_Battle.Types[u.Type];
                if (cfg.Weapon != null && cfg.Weapon.Length > 0)
                {
                    // the weapon rests cocked (the swing angle); a shot swings it to the model's pose over SwingSteps (the projectile
                    // leaves at the launch delay, ideally then) and it cocks again over ResetSteps
                    float angle = cfg.SwingDegrees;
                    if (cfg.SwingDegrees != 0f && u.LastShotStep >= 0)
                    {
                        int age = step - u.LastShotStep;
                        int swing = math.max(cfg.SwingSteps, 1), reset = math.max(cfg.ResetSteps, 1);
                        if (age < swing) angle = cfg.SwingDegrees * (1f - (float)age / swing);
                        else if (age < swing + reset) angle = cfg.SwingDegrees * (float)(age - swing) / reset;
                    }
                    quaternion swingRot = angle != 0f ? quaternion.AxisAngle(math.normalizesafe((float3)cfg.SwingAxis, new float3(1, 0, 0)), math.radians(angle)) : quaternion.identity;
                    foreach (var part in cfg.Weapon)
                    {
                        if (part.Mesh == null) continue;
                        float3 local = (float3)cfg.SwingPivot + math.mul(swingRot, (float3)part.LocalPosition - (float3)cfg.SwingPivot);   // model metres
                        quaternion rot = math.mul(swingRot, quaternion.Euler(math.radians((float3)part.LocalEuler)));
                        float scale = (part.Scale > 0f ? part.Scale : 1f) * BrickScale;
                        m_Attachments.Add(part.Mesh, AttachmentRenderer.Attached(u.Body, local * BrickScale + a.MeshOffset, rot, scale, u.Team == Battle.Defenders && Config != null && Config.DefenderTint.a > 0 ? Config.DefenderTint : cfg.Tint));
                    }
                }
                if (u.Team == Battle.Attackers && u.Type == m_Selected && u.State != UnitState.Dead)
                    m_Attachments.Add(m_Plate, AttachmentRenderer.Attached(u.Body, new float3(0f, -a.BoxSize.y * 0.5f + AvbdGpuConstants.CollisionMargin + 0.03f, 0f), quaternion.identity,
                        new float3(a.BoxSize.x * PlateMargin, 1f, math.max(a.BoxSize.z * PlateMargin, a.BoxSize.x * 0.6f)), SelectionColor), transparent: true);
            }
            foreach (var p in m_Battle.Projectiles)
            {
                if (p.Spent) continue;
                var cfg = m_Types[p.Type];
                if (cfg.TipMesh == null) continue;
                var a = m_Battle.Types[p.Type];
                m_Attachments.Add(cfg.TipMesh, AttachmentRenderer.Attached(p.Body, (float3)cfg.TipLocalPosition * BrickScale + a.ProjectileMeshOffset, quaternion.identity, cfg.TipScale * BrickScale, cfg.TipColor, cfg.TipUnlit));
            }
            foreach (var e in m_Effects)
            {
                var c = e.Config;
                float t = math.saturate((step - e.Birth) / (float)math.max(c.LifeSteps, 1));
                float scale = math.lerp(c.StartScale, c.EndScale, t);
                var color = c.Color; color.a = (byte)math.round(c.Color.a * (1f - t));
                var rot = quaternion.RotateY(math.radians(e.Yaw + c.SpinDegPerSec * (step - e.Birth) / 60f));
                m_Attachments.Add(c.Mesh, AttachmentRenderer.Free(e.Position, rot, scale, color, c.Unlit), transparent: true);
            }
            m_Attachments.Render(camera);
        }

        // ------------------------------------------------------------------------------------------------ HUD

        string ArmiesText()
        {
            if (m_Battle == null) return "";
            var sb = new System.Text.StringBuilder();
            for (int team = 0; team < 2; team++)
            {
                sb.Append(team == Battle.Attackers ? "<color=#ffcc88>attackers</color>: " : "<color=#88bbff>defenders</color>: ");
                bool any = false;
                for (int t = 0; t < m_Types.Length; t++)
                {
                    int total = 0;
                    foreach (var r in team == Battle.Attackers ? Config.Attackers : Config.Defenders) if (r.Unit == m_Types[t]) total += r.Count;
                    if (total == 0) continue;
                    sb.Append(any ? ", " : "").Append(m_Battle.AliveOf(team, t)).Append('/').Append(total).Append(' ').Append(m_Types[t].name);
                    any = true;
                }
                sb.Append(team == 0 ? "   " : "\n");
            }
            return sb.ToString();
        }

        protected override string HudText()
        {
            var cfg = Config;
            string title = cfg != null ? $"<b>[{m_Scene + 1}/{m_Configs.Length}] {ConfigName(cfg)}</b> ({cfg.name}.asset, {(cfg.Castle != null ? cfg.Castle.name + ".json" : "no castle")})" : "<b>no SiegeConfig assets</b>";
            string castle;
            if (m_Assembly == null) castle = $"<color=#ff5555>{m_Error}</color>\n";
            else
            {
                float unit = PieceCatalog.GridToUnity * BrickScale;
                float3 e = m_Assembly.Extent;
                int walls = 0; foreach (var p in m_Occupancy.Posts) if (p.OnWall) walls++;
                castle = $"{m_Assembly.Parts.Count} pieces, {e.x:F0} x {e.z:F0} studs, {e.y:F1} tall = {e.x * unit:F1} x {e.z * unit:F1} x {e.y * unit:F1} m; {m_Occupancy.Posts.Count} posts ({walls} on the walls{(m_Occupancy.ExplicitPosts ? ", from the document" : ", derived")})  -  " +
                         (Snap ? $"<color=#88ddff>snapped</color>: {m_SnapJoints} joints ({SnapFractureLateral:F0} / {SnapFractureTension:F0} N)" : "dry-stacked") +
                         (m_Diagnostics.Clean ? "" : $"  <color=#ffcc55>{m_Diagnostics.Floating} floating, {m_Diagnostics.PoorlySupported} poorly supported, {m_Diagnostics.Intersections} intersecting</color>") + "\n";
            }
            string deaths = m_LastDeath.Length > 0 ? $"  -  last death: {m_LastDeath}" : "";
            string selection = m_Battle == null ? "" : m_Selected >= 0
                ? $"<color=#80ff90>selected: {m_Types[m_Selected].name}</color> x {m_Battle.AliveOf(Battle.Attackers, m_Selected)} (range {m_Battle.Types[m_Selected].Range:F0} m)  -  RMB on a block: attack, on the ground: move" + (m_LastOrder.Length > 0 ? $"  -  last order: {m_LastOrder}" : "") + "\n"
                : "nothing selected: LMB on one of your units selects its kind\n";
            return
                $"{title}{PausedText}\n" + castle + ArmiesText() + selection +
                (m_Battle != null ? m_Battle.Summary() + $"; {m_Effects.Count} effects, {(m_Attachments != null ? m_Attachments.LastInstances : 0)} attachments{deaths}\n" : "") +
                m_Vegetation.AroundText() + TerrainText(m_Plateau) + "\n\n" +
                StatsText() + "\n\n" +
                "1-0 siege config  , . prev/next  R rebuild  Esc deselect  X retire the dead now  J snap on/off  T terrain  F trees 0/10/20/30  Space pause  N step\n" +
                "F1 contacts  F2 colour mode  F5 joints  F6 collision boxes  F7 shadows  F8 sleep on/off  +/- iterations  [ ] substeps  B/Enter cannonball  G gravity  H hide HUD\n" +
                "LMB select unit kind  RMB click order (drag orbits)  MMB pan  wheel / Q E zoom  W A S D orbit";
        }
    }
}
