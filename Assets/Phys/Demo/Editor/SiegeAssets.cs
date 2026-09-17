using System.IO;
using System.Linq;
using Phys.AvbdGpu.Siege;
using UnityEditor;
using UnityEngine;

namespace Phys.Demo.Editor
{
    /// <summary>The default siege configs (Phys / Create Siege Configs, also -executeMethod Phys.Demo.Editor.SiegeAssets.CreateSiegeConfigs):
    /// the hit effects and unit kinds built on the ConstructorFantasy models (their bounds read from the meshes, the trebuchet's part
    /// pivots from its manifest) and two sieges, the Outpost and the Emerald Crown citadel, as SiegeConfig1 / SiegeConfig2 in
    /// Resources. Idempotent: existing assets are overwritten in place (their GUIDs kept).</summary>
    public static class SiegeAssets
    {
        const string Fantasy = "Assets/Models/ConstructorFantasy";
        const string Figure = "Assets/Models/ConstructorFigure/ConstructorFigure.fbx";
        const string UnitsDir = "Assets/Resources/Siege/Units", EffectsDir = "Assets/Resources/Siege/HitEffects", ConfigsDir = "Assets/Resources";

        [MenuItem("Phys/Create Siege Configs")]
        public static void CreateSiegeConfigs()
        {
            bool ok = true;
            Mesh Load(string file) { var m = AssetDatabase.LoadAssetAtPath<Mesh>(file); if (m == null) { Debug.LogError($"SiegeAssets: no mesh in {file}"); ok = false; } return m; }
            Mesh Part(string file, string part)
            {
                var meshes = AssetDatabase.LoadAllAssetsAtPath(file).OfType<Mesh>().ToArray();
                var m = meshes.FirstOrDefault(x => x.name == part) ?? meshes.FirstOrDefault(x => x.name.StartsWith(part) || x.name.EndsWith(part));
                if (m == null) { Debug.LogError($"SiegeAssets: no mesh '{part}' in {file} (found: {string.Join(", ", meshes.Select(x => x.name))})"); ok = false; }
                return m;
            }
            Directory.CreateDirectory(UnitsDir); Directory.CreateDirectory(EffectsDir);

            var figure = Load(Figure);
            var mage = Load($"{Fantasy}/ConstructorMage.fbx");
            var bow = Load($"{Fantasy}/ConstructorBow.fbx");
            var arrow = Load($"{Fantasy}/ConstructorBowArrow.fbx");
            var heavyArrow = Load($"{Fantasy}/ConstructorBowHeavyArrow.fbx");
            var spell = Load($"{Fantasy}/ConstructorSpellProjectile.fbx");
            var explosion1 = Load($"{Fantasy}/ConstructorSpellExplosion01.fbx");
            var explosion2 = Load($"{Fantasy}/ConstructorSpellExplosion02.fbx");
            var explosion3 = Load($"{Fantasy}/ConstructorSpellExplosion03.fbx");
            var rocks = new[] { Load($"{Fantasy}/ConstructorRock01.fbx"), Load($"{Fantasy}/ConstructorRock02.fbx"), Load($"{Fantasy}/ConstructorRock03.fbx"), Load($"{Fantasy}/ConstructorRock04.fbx") };
            string trebuchet = $"{Fantasy}/ConstructorTrebuchet.fbx";
            var trebBase = Part(trebuchet, "Base");
            var beam = Part(trebuchet, "ActiveBeam");
            var load = Part(trebuchet, "HangingLoad");
            var basin = Part(trebuchet, "ProjectileBasin");
            if (!ok) { if (Application.isBatchMode) EditorApplication.Exit(1); return; }

            // ---- hit effects
            var explosive = Effect("Explosive", 1.2f, 3f, 6f, 0.4f, explosion3, new Color32(255, 150, 60, 220), 2f, 8f, 36, 60f);
            var rock = Effect("Rock", 0.8f, 2.5f, 8f, 0.5f, explosion3, new Color32(150, 140, 120, 200), 2f, 7f, 30, 40f);
            var fire = Effect("Fire", 2f, 5f, 12f, 0.4f, explosion1, new Color32(255, 90, 30, 230), 3f, 12f, 45, 120f);
            var frost = Effect("Frost", 0.8f, 6f, 16f, 0.7f, explosion2, new Color32(130, 200, 255, 220), 3f, 14f, 45, 30f);
            var arcane = Effect("Arcane", 3f, 3f, 6f, 0.3f, explosion3, new Color32(190, 110, 255, 230), 3f, 10f, 45, 200f);

            // ---- units
            var archer = Unit("Archer", u =>
            {
                Body(u, figure, new Color32(200, 60, 50, 255));
                u.Weapon = new[] { new UnitConfig.WeaponPart { Mesh = bow, LocalPosition = new Vector3(0.148f, 0.197f, 0.008f), Scale = 1f } };
                Projectile(u, new[] { arrow }, 0.02f, true, new Color32(222, 200, 150, 255));
                u.Trajectory = Trajectory.Elevation; u.ElevationDeg = 55f; u.MaxSpeed = 40f; u.Spread = 0.03f;
                u.LaunchPoint = new Vector3(0.15f, 0.36f, 0.06f); u.LaunchOffset = 2.6f;
                u.Range = 35f; u.CooldownSteps = 150; u.AutoEngage = true; u.ProjectileRetireDelay = 120; u.HitEffect = null;
            });
            var explosiveArcher = Unit("ExplosiveArcher", u =>
            {
                Body(u, figure, new Color32(230, 130, 40, 255));
                u.Weapon = new[] { new UnitConfig.WeaponPart { Mesh = bow, LocalPosition = new Vector3(0.148f, 0.197f, 0.008f), Scale = 1f } };
                Projectile(u, new[] { heavyArrow }, 0.05f, true, new Color32(120, 60, 40, 255));
                u.TipMesh = spell; u.TipLocalPosition = new Vector3(0f, 0f, 0.12f); u.TipScale = 0.45f; u.TipColor = new Color32(255, 140, 40, 255); u.TipUnlit = 1f;
                u.Trajectory = Trajectory.Elevation; u.ElevationDeg = 50f; u.MaxSpeed = 40f; u.Spread = 0.03f;
                u.LaunchPoint = new Vector3(0.15f, 0.36f, 0.06f); u.LaunchOffset = 2.6f;
                u.Range = 35f; u.CooldownSteps = 240; u.AutoEngage = true; u.ProjectileRetireDelay = 0; u.HitEffect = explosive;
            });
            var trebuchetUnit = Unit("Trebuchet", u =>
            {
                Body(u, trebBase, new Color32(134, 96, 58, 255));
                u.Mass = 30f; u.Speed = 1.2f; u.Force = 120f; u.HitPoints = 5;
                // the manifest's pivots: the beam on its axle, the load and the basin hanging from the beam (assembled metres)
                u.Weapon = new[]
                {
                    new UnitConfig.WeaponPart { Mesh = beam, LocalPosition = new Vector3(0f, 0.65f, 0f), Scale = 1f },
                    new UnitConfig.WeaponPart { Mesh = load, LocalPosition = new Vector3(0f, 0.47f, -0.28f), Scale = 1f },
                    new UnitConfig.WeaponPart { Mesh = basin, LocalPosition = new Vector3(0f, 1.05f, 0.65f), Scale = 1f },
                };
                // rests cocked (the basin down behind, the counterweight raised), swings over the top to the model's pose and re-cocks
                u.SwingPivot = new Vector3(0f, 0.65f, 0f); u.SwingAxis = Vector3.right; u.SwingDegrees = -210f; u.SwingSteps = 30; u.ResetSteps = 180;
                Projectile(u, rocks, 6f, false, new Color32(110, 105, 100, 255));
                u.Trajectory = Trajectory.HighArc; u.LaunchSpeed = 32f; u.Spread = 0.02f;
                u.LaunchPoint = new Vector3(0f, 1.05f, 0.65f); u.LaunchOffset = 1f; u.LaunchDelaySteps = 30;
                u.Range = 70f; u.CooldownSteps = 420; u.AutoEngage = false; u.ProjectileRetireDelay = 600; u.HitEffect = rock;
            });
            UnitConfig Mage(string name, Color32 tint, Color32 spellColor, HitEffectConfig effect) => Unit(name, u =>
            {
                Body(u, mage, tint);
                Projectile(u, new[] { spell }, 0.15f, true, spellColor);
                u.ProjectileUnlit = 1f;
                u.Trajectory = Trajectory.Homing; u.LaunchSpeed = 22f; u.Thrust = 8f; u.Spread = 0f;
                u.LaunchPoint = new Vector3(0.1f, 0.5f, 0.1f); u.LaunchOffset = 2.6f;
                u.Range = 40f; u.CooldownSteps = 300; u.AutoEngage = true; u.ProjectileRetireDelay = 0; u.HitEffect = effect;
            });
            var mageFire = Mage("MageFire", new Color32(200, 70, 40, 255), new Color32(255, 120, 40, 255), fire);
            var mageFrost = Mage("MageFrost", new Color32(80, 140, 220, 255), new Color32(150, 220, 255, 255), frost);
            var mageArcane = Mage("MageArcane", new Color32(150, 80, 200, 255), new Color32(210, 140, 255, 255), arcane);

            // ---- sieges
            var outpost = AssetDatabase.LoadAssetAtPath<TextAsset>("Assets/Resources/Castles/outpost.json");
            var citadel = AssetDatabase.LoadAssetAtPath<TextAsset>("Assets/Resources/Castles/emerald_crown_citadel.json");
            if (outpost == null || citadel == null) { Debug.LogError("SiegeAssets: the castle documents are missing"); if (Application.isBatchMode) EditorApplication.Exit(1); return; }
            Siege("SiegeConfig1", "Siege of the Outpost", outpost, 30f,
                new[] { Roster(archer, 12), Roster(explosiveArcher, 4), Roster(trebuchetUnit, 2), Roster(mageFire, 2) },
                new[] { Roster(archer, 12), Roster(mageFrost, 2) });
            Siege("SiegeConfig2", "Siege of the Emerald Crown", citadel, 30f,
                new[] { Roster(archer, 24), Roster(explosiveArcher, 8), Roster(trebuchetUnit, 4), Roster(mageFire, 3), Roster(mageFrost, 3), Roster(mageArcane, 3) },
                new[] { Roster(archer, 24), Roster(mageArcane, 4), Roster(mageFrost, 2) });
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("SiegeAssets: 5 hit effects, 6 units, 2 siege configs written");
        }

        static SiegeConfig.Roster Roster(UnitConfig unit, int count) => new SiegeConfig.Roster { Unit = unit, Count = count };

        static void Body(UnitConfig u, Mesh mesh, Color32 tint)
        {
            u.BodyMesh = mesh; u.Tint = tint;
            u.BodyBoundsCenter = mesh.bounds.center; u.BodyBoundsSize = mesh.bounds.size;
        }

        static void Projectile(UnitConfig u, Mesh[] meshes, float mass, bool align, Color32 color)
        {
            u.ProjectileMeshes = meshes; u.ProjectileMass = mass; u.AlignToVelocity = align; u.ProjectileColor = color;
            // the variants share one box: the largest bounds
            var b = meshes[0].bounds;
            foreach (var m in meshes) b.Encapsulate(m.bounds);
            u.ProjectileBoundsCenter = b.center; u.ProjectileBoundsSize = b.size;
        }

        static HitEffectConfig Effect(string name, float pulverize, float impact, float impulse, float lift, Mesh mesh, Color32 color, float start, float end, int life, float spin)
        {
            var e = LoadOrCreate<HitEffectConfig>($"{EffectsDir}/{name}.asset");
            e.PulverizeRadius = pulverize; e.ImpactRadius = impact; e.Impulse = impulse; e.Lift = lift; e.KillUnits = true;
            e.Mesh = mesh; e.Color = color; e.StartScale = start; e.EndScale = end; e.LifeSteps = life; e.SpinDegPerSec = spin; e.Unlit = 1f;
            EditorUtility.SetDirty(e);
            return e;
        }

        static UnitConfig Unit(string name, System.Action<UnitConfig> fill)
        {
            var u = LoadOrCreate<UnitConfig>($"{UnitsDir}/{name}.asset");
            u.DisplayName = name;
            u.Mass = 1f; u.Friction = 0.6f; u.Speed = 3f; u.Force = 12f; u.HitPoints = 1;
            u.Weapon = new UnitConfig.WeaponPart[0]; u.SwingDegrees = 0f; u.TipMesh = null; u.Thrust = 0f; u.LaunchDelaySteps = 0; u.ProjectileUnlit = 0f;
            fill(u);
            EditorUtility.SetDirty(u);
            return u;
        }

        static void Siege(string file, string title, TextAsset castle, float formationDistance, SiegeConfig.Roster[] attackers, SiegeConfig.Roster[] defenders)
        {
            var s = LoadOrCreate<SiegeConfig>($"{ConfigsDir}/{file}.asset");
            s.DisplayName = title; s.Castle = castle; s.Attackers = attackers; s.Defenders = defenders;
            s.AttackSide = 0; s.FormationDistance = formationDistance; s.RankSpacing = 3f; s.ColumnSpacing = 2.5f; s.MaxColumns = 12;
            s.DefenderTint = new Color32(70, 110, 200, 255);
            EditorUtility.SetDirty(s);
        }

        static T LoadOrCreate<T>(string path) where T : ScriptableObject
        {
            var asset = AssetDatabase.LoadAssetAtPath<T>(path);
            if (asset != null) return asset;
            asset = ScriptableObject.CreateInstance<T>();
            AssetDatabase.CreateAsset(asset, path);
            return asset;
        }

        /// <summary>Assigns the 27 piece meshes to the SiegeDemo of Siege.unity and saves the scene.</summary>
        [MenuItem("Phys/Assign Siege Meshes")]
        public static void AssignSiegeMeshes()
        {
            var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene("Assets/Phys/Demo/Siege.unity");
            var demo = Object.FindFirstObjectByType<SiegeDemo>();
            if (demo == null) { Debug.LogError("Siege.unity has no SiegeDemo"); if (Application.isBatchMode) EditorApplication.Exit(1); return; }
            var pieces = Phys.AvbdGpu.Scenes.PieceCatalog.Pieces;
            demo.PieceMeshes = new Mesh[pieces.Length];
            int missing = 0;
            for (int i = 0; i < pieces.Length; i++)
            {
                demo.PieceMeshes[i] = AssetDatabase.LoadAssetAtPath<Mesh>($"Assets/Models/construction_pieces/{pieces[i].Id}.fbx");
                if (demo.PieceMeshes[i] == null) { missing++; Debug.LogError($"AssignSiegeMeshes: no mesh for {pieces[i].Id}"); }
            }
            EditorUtility.SetDirty(demo);
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(scene);
            UnityEditor.SceneManagement.EditorSceneManager.SaveScene(scene);
            Debug.Log($"AssignSiegeMeshes: {pieces.Length - missing} of {pieces.Length} piece meshes assigned");
            if (missing > 0 && Application.isBatchMode) EditorApplication.Exit(1);
        }
    }
}
