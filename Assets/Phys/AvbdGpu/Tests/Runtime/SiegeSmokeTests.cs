using System.Collections;
using NUnit.Framework;
using Phys.AvbdGpu.Siege;
using Phys.Demo;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.TestTools;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The siege demo end to end on the Outpost document with in-memory configs: the armies spawn (the garrison on the walls),
    /// the player's orders walk and attack, shots fly, impacts break snaps and draw effects, the selection draws its plates, nothing
    /// overflows.</summary>
    public class SiegeSmokeTests
    {
        GameObject m_Root;
        SiegeDemo m_Demo;
        UnitConfig m_Archer, m_Explosive, m_Trebuchet;
        SiegeConfig m_Config;

        [UnitySetUp]
        public IEnumerator SetUp()
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("Compute shaders are not supported on this device");
            m_Root = new GameObject("SiegeSmoke");
            var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
            camGo.transform.SetParent(m_Root.transform);
            camGo.AddComponent<Camera>();
            camGo.AddComponent<DemoCamera>();
            var light = new GameObject("Light").AddComponent<Light>();
            light.type = LightType.Directional;
            light.shadows = LightShadows.Soft;
            light.transform.SetParent(m_Root.transform);
            light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            MakeConfigs();
            m_Demo = m_Root.AddComponent<SiegeDemo>();
            m_Demo.StartScene = 0;
            m_Demo.MaxBodies = 40960;
            m_Demo.Trees = 0;
            m_Demo.FoliagePerTree = 0;
            m_Demo.Configs = new[] { m_Config };
            yield return null;   // Start: the world, the castle with its settle steps, the armies
            for (int f = 0; f < 30 && m_Demo.World.ReadStep < 0; f++) yield return null;   // the first pose readback (orders and picking need it)
            Assert.GreaterOrEqual(m_Demo.World.ReadStep, 0, "poses read back");
        }

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            if (m_Root != null) Object.Destroy(m_Root);
            foreach (var o in new ScriptableObject[] { m_Archer, m_Explosive, m_Trebuchet, m_Config }) if (o != null) Object.Destroy(o);
            yield return null;
        }

        static Mesh LoadMesh(string path)
        {
#if UNITY_EDITOR
            return UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>(path);
#else
            return null;
#endif
        }

        void MakeConfigs()
        {
            var figure = LoadMesh("Assets/Models/ConstructorFigure/ConstructorFigure.fbx");
            var arrow = LoadMesh("Assets/Models/ConstructorFantasy/ConstructorBowArrow.fbx");
            var bow = LoadMesh("Assets/Models/ConstructorFantasy/ConstructorBow.fbx");
            var spell = LoadMesh("Assets/Models/ConstructorFantasy/ConstructorSpellProjectile.fbx");
            var explosion = LoadMesh("Assets/Models/ConstructorFantasy/ConstructorSpellExplosion03.fbx");
            var rock = LoadMesh("Assets/Models/ConstructorFantasy/ConstructorRock01.fbx");

            var blast = ScriptableObject.CreateInstance<HitEffectConfig>();
            blast.PulverizeRadius = 2.5f; blast.ImpactRadius = 4f; blast.Impulse = 6f; blast.Lift = 0.4f; blast.KillRadius = 3f;   // big enough to reach the wall's snaps from an arrow stopped by a defender standing on it
            blast.Mesh = explosion; blast.LifeSteps = 40;

            m_Archer = ScriptableObject.CreateInstance<UnitConfig>();
            m_Archer.name = "Archer"; m_Archer.DisplayName = "Archer";
            m_Archer.BodyMesh = figure;
            if (figure != null) { m_Archer.BodyBoundsCenter = figure.bounds.center; m_Archer.BodyBoundsSize = figure.bounds.size; }
            m_Archer.Weapon = new[] { new UnitConfig.WeaponPart { Mesh = bow, LocalPosition = new Vector3(0.148f, 0.197f, 0.008f), Scale = 1f } };
            m_Archer.ProjectileMeshes = new[] { arrow };
            if (arrow != null) { m_Archer.ProjectileBoundsCenter = arrow.bounds.center; m_Archer.ProjectileBoundsSize = arrow.bounds.size; }
            m_Archer.Range = 35f; m_Archer.CooldownSteps = 120; m_Archer.Spread = 0.02f; m_Archer.AutoEngage = false;   // nothing flies until ordered: the attachment counts stay exact

            m_Explosive = ScriptableObject.CreateInstance<UnitConfig>();
            m_Explosive.name = "ExplosiveArcher"; m_Explosive.DisplayName = "ExplosiveArcher";
            m_Explosive.BodyMesh = figure;
            if (figure != null) { m_Explosive.BodyBoundsCenter = figure.bounds.center; m_Explosive.BodyBoundsSize = figure.bounds.size; }
            m_Explosive.ProjectileMeshes = new[] { arrow };
            if (arrow != null) { m_Explosive.ProjectileBoundsCenter = arrow.bounds.center; m_Explosive.ProjectileBoundsSize = arrow.bounds.size; }
            m_Explosive.ProjectileMass = 0.05f; m_Explosive.TipMesh = spell; m_Explosive.TipScale = 0.45f;
            m_Explosive.Range = 35f; m_Explosive.CooldownSteps = 120; m_Explosive.Spread = 0.02f; m_Explosive.ProjectileRetireDelay = 0; m_Explosive.HitEffect = blast; m_Explosive.AutoEngage = false;

            m_Trebuchet = ScriptableObject.CreateInstance<UnitConfig>();
            m_Trebuchet.name = "Trebuchet"; m_Trebuchet.DisplayName = "Trebuchet";
            m_Trebuchet.BodyBoundsCenter = new Vector3(0f, 0.33f, 0.2f); m_Trebuchet.BodyBoundsSize = new Vector3(0.726f, 0.66f, 1.325f);
            m_Trebuchet.Mass = 30f; m_Trebuchet.Speed = 1.2f; m_Trebuchet.Force = 120f; m_Trebuchet.HitPoints = 5;
            m_Trebuchet.ProjectileMeshes = new[] { rock };
            m_Trebuchet.ProjectileBoundsSize = new Vector3(0.1f, 0.1f, 0.1f); m_Trebuchet.ProjectileMass = 6f; m_Trebuchet.AlignToVelocity = false;
            m_Trebuchet.Trajectory = Trajectory.HighArc; m_Trebuchet.LaunchSpeed = 32f; m_Trebuchet.LaunchPoint = new Vector3(0f, 1.05f, 0.65f); m_Trebuchet.LaunchOffset = 1f; m_Trebuchet.LaunchDelaySteps = 30;
            m_Trebuchet.Range = 70f; m_Trebuchet.CooldownSteps = 300; m_Trebuchet.AutoEngage = false; m_Trebuchet.ProjectileRetireDelay = 600; m_Trebuchet.HitEffect = blast;

            m_Config = ScriptableObject.CreateInstance<SiegeConfig>();
            m_Config.name = "SmokeSiege"; m_Config.DisplayName = "Smoke siege";
            m_Config.Castle = Resources.Load<TextAsset>("Castles/outpost");
            m_Config.Attackers = new[] { new SiegeConfig.Roster { Unit = m_Archer, Count = 6 }, new SiegeConfig.Roster { Unit = m_Explosive, Count = 3 }, new SiegeConfig.Roster { Unit = m_Trebuchet, Count = 1 } };
            m_Config.Defenders = new[] { new SiegeConfig.Roster { Unit = m_Archer, Count = 6 } };
            m_Config.AttackSide = 0; m_Config.FormationDistance = 24f;
        }

        int TypeOf(UnitConfig cfg) => System.Array.IndexOf(m_Demo.UnitTypes, cfg);

        [UnityTest]
        public IEnumerator ArmiesSpawnAndTheGarrisonHoldsTheWalls()
        {
            Assert.IsNull(m_Demo.Error, m_Demo.Error);
            Assert.IsNotNull(m_Demo.Battle);
            Assert.IsNotNull(m_Demo.Assembly);
            var battle = m_Demo.Battle;
            Assert.AreEqual(10, battle.Alive[Battle.Attackers], "six archers, three explosive archers, a trebuchet");
            Assert.AreEqual(6, battle.Alive[Battle.Defenders]);
            Assert.AreEqual(3, m_Demo.UnitTypes.Length, "the distinct unit configs of both rosters");
            int pools = 0;
            for (int t = 0; t < battle.Types.Length; t++)
            {
                if (battle.UnitPools[t] != null && m_Demo.UnitTypes[t].BodyMesh != null) pools++;
                foreach (var p in battle.ProjectilePools[t]) if (p != null && m_Demo.UnitTypes[t].ProjectileMeshes.Length > 0 && m_Demo.UnitTypes[t].ProjectileMeshes[0] != null) pools++;
            }
            Assert.AreEqual(m_Demo.Bodies.Groups.Count + pools, m_Demo.Renderer.MeshRanges.Count, "a range per piece group, per unit pool with a model and per projectile pool with a model");
            // the garrison stands on the posts: on the walls above the plateau
            m_Demo.World.GetPosesSync(out var pos, out var rot);
            int onWalls = 0;
            float plateau = m_Demo.Plateau;
            foreach (var u in battle.Units)
            {
                float3 p = pos[u.Body].xyz;
                Assert.IsTrue(math.all(math.isfinite(p)), $"unit {u.Body} position");
                float3 up = math.mul(new quaternion(rot[u.Body]), new float3(0, 1, 0));
                Assert.Greater(up.y, 0.99f, $"unit {u.Body} stands upright");
                if (u.Team == Battle.Defenders && p.y > plateau + 3f) onWalls++;
                if (u.Team == Battle.Attackers) Assert.Less(p.z, -m_Demo.Assembly.Extent.z * 0.25f, "the attackers formed up south of the castle");
            }
            Assert.GreaterOrEqual(onWalls, 4, "most of the six defenders stand on the wall tops");
            Assert.IsTrue(m_Demo.Occupancy.ExplicitPosts, "the document carries its posts");
            Assert.IsNotEmpty(m_Demo.Hud);
            var stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags);
            yield return null;
        }

        [UnityTest]
        public IEnumerator OrdersWalkTheArmyAndBreakTheWalls()
        {
            var battle = m_Demo.Battle;
            // a move order: the archers walk toward a point 12 m nearer the castle, keeping their formation
            int archers = TypeOf(m_Archer);
            m_Demo.SelectUnitType(archers);
            Assert.AreEqual(archers, m_Demo.SelectedType);
            var selected = m_Demo.SelectedUnits();
            Assert.AreEqual(6, selected.Count);
            m_Demo.World.GetPosesSync(out var start, out _);
            float3 centroid = float3.zero;
            foreach (int i in selected) centroid += start[battle.Units[i].Body].xyz;
            centroid /= selected.Count;
            float3 goal = centroid + new float3(0f, 0f, 12f);
            var order = m_Demo.OrderAt(new Ray(goal + new float3(0f, 60f, 0f), Vector3.down));
            Assert.AreEqual(OrderKind.Move, order, "the ground under the ray: a move");
            foreach (int i in selected) Assert.AreEqual(UnitState.Moving, battle.Units[i].State);
            for (int f = 0; f < 360; f++) yield return null;
            m_Demo.World.GetPosesSync(out var later, out _);
            foreach (int i in selected)
            {
                var u = battle.Units[i];
                Assert.Greater(later[u.Body].z, start[u.Body].z + 8f, $"archer {i} walked toward the castle");
                Assert.Less(math.abs(later[u.Body].x - start[u.Body].x), 1.5f, $"archer {i} kept its column");
            }

            // an attack order on a wall brick: the explosive archers walk into range and their hits break snaps
            int explosive = TypeOf(m_Explosive);
            m_Demo.SelectUnitType(explosive);
            var bombers = m_Demo.SelectedUnits();
            Assert.AreEqual(3, bombers.Count);
            // a brick of the south wall: the highest castle body whose z lies in the front quarter
            int target = -1; float best = float.NegativeInfinity;
            for (int i = 0; i < m_Demo.Bodies.Count; i++)
            {
                int b = m_Demo.Bodies.First + i;
                float3 p = later[b].xyz;
                if (p.z < -m_Demo.Assembly.Extent.z * 0.5f * 0.1f * m_Demo.BrickScale * 0.6f && p.y > best) { best = p.y; target = b; }
            }
            Assert.GreaterOrEqual(target, 0);
            float3 brick = later[target].xyz;
            var impacts = new System.Collections.Generic.List<float3>();
            var onImpact = battle.OnImpact;
            battle.OnImpact = (type, at) => { onImpact?.Invoke(type, at); impacts.Add(at); };
            order = m_Demo.OrderAt(new Ray(brick + new float3(0f, 60f, 0f), Vector3.down));
            Assert.AreEqual(OrderKind.Attack, order, "a brick under the ray: an attack");
            foreach (int i in bombers) Assert.AreEqual(OrderKind.Attack, battle.Units[i].Order);
            int impactsBefore = battle.Impacts;
            var jointsBefore = m_Demo.World.GetJointStatesSync();
            int brokenBefore = 0; foreach (var j in jointsBefore) if (j.Broken != 0) brokenBefore++;
            bool drewEffect = false;
            for (int f = 0; f < 1200 && (battle.Impacts - impactsBefore < 2 || !drewEffect); f++)
            {
                yield return null;
                if (m_Demo.EffectCount > 0 && m_Demo.Attachments.LastInstances > 0) drewEffect = true;
                if (f % 120 == 0) Assert.AreEqual(0, m_Demo.World.GetStatsSync().OverflowFlags, $"no overflow at frame {f}");
            }
            Assert.GreaterOrEqual(battle.Impacts - impactsBefore, 2, "the explosive arrows hit and went off");
            Assert.IsTrue(drewEffect, "an explosion was drawn");
            var joints = m_Demo.World.GetJointStatesSync();
            int broken = 0; foreach (var j in joints) if (j.Broken != 0) broken++;
            var where = new System.Text.StringBuilder($"target {target} at {brick}; impacts:");
            foreach (var at in impacts) where.Append(' ').Append(at);
            Debug.Log($"siege smoke: {where}");
            Assert.Greater(broken, brokenBefore, "snaps of the wall broke: " + where);
            Debug.Log($"siege smoke: {battle.Dead[Battle.Defenders]} defenders killed by the blasts");
            Assert.Greater(battle.ShotsFired, 0);
            foreach (var u in battle.Units) if (u.Team == Battle.Attackers && u.Type == explosive) Assert.AreNotEqual(UnitState.Idle, u.State, "the bombers keep their attack order");
            m_Demo.World.GetPosesSync(out var end, out _);
            for (int i = 0; i < m_Demo.World.BodyCount; i++) Assert.IsTrue(math.all(math.isfinite(end[i])), $"body {i} position finite");
            Debug.Log($"siege smoke: {battle.Summary()}; {broken} snaps broken; {m_Demo.Renderer.MeshRanges.Count} ranges, {m_Demo.Attachments.LastDraws} attachment draws");
        }

        [UnityTest]
        public IEnumerator SelectionDrawsPlatesAndTheTrebuchetLobs()
        {
            var battle = m_Demo.Battle;
            Assert.AreEqual(0, m_Demo.Attachments.LastInstances - CountWeapons(), "nothing selected: only the weapons are drawn");
            m_Demo.SelectUnitType(TypeOf(m_Archer));
            yield return null;
            Assert.AreEqual(6, m_Demo.Attachments.LastInstances - CountWeapons(), "a plate under every archer");
            m_Demo.SelectUnitType(-1);
            yield return null;
            Assert.AreEqual(0, m_Demo.Attachments.LastInstances - CountWeapons());

            // the trebuchet attacks the castle's centre body: a rock leaves after the swing
            int trebuchet = TypeOf(m_Trebuchet);
            m_Demo.SelectUnitType(trebuchet);
            m_Demo.World.GetPosesSync(out var pos, out _);
            int target = m_Demo.Bodies.First;
            var order = m_Demo.OrderAt(new Ray(pos[target].xyz + new float3(0f, 80f, 0f), Vector3.down));
            Assert.AreEqual(OrderKind.Attack, order);
            int shots = battle.ShotsFired;
            for (int f = 0; f < 600 && battle.ShotsFired == shots; f++) yield return null;
            Assert.Greater(battle.ShotsFired, shots, "the trebuchet fired");
            bool rockFlying = false;
            foreach (var p in battle.Projectiles) if (p.Type == trebuchet && !p.Spent) rockFlying = true;
            Assert.IsTrue(rockFlying || battle.Impacts > 0, "a rock in flight or already landed");
            Assert.AreEqual(0, m_Demo.World.GetStatsSync().OverflowFlags);
        }

        int CountWeapons()
        {
            int n = 0;
            foreach (var u in m_Demo.Battle.Units) n += m_Demo.UnitTypes[u.Type].Weapon?.Length ?? 0;
            // spell tips on the explosive arrows in flight and the effects count too
            foreach (var p in m_Demo.Battle.Projectiles) if (!p.Spent && m_Demo.UnitTypes[p.Type].TipMesh != null) n++;
            return n + m_Demo.EffectCount;
        }
    }
}
