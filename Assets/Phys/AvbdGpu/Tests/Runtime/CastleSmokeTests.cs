using System.Collections;
using NUnit.Framework;
using Phys.AvbdGpu;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdGpu.Siege;
using Phys.Demo;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.TestTools;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>PlayMode: the castle demo (brick bodies drawn with the brick model) runs, the dry-stacked castle stands still, and the
    /// trees planted around it and the foliage scattered between them sleep until a cannonball wakes one.</summary>
    public class CastleSmokeTests
    {
        GameObject m_Root;
        CastleDemo m_Demo;

        [UnitySetUp]
        public IEnumerator SetUp()
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("no compute");
            CreateDemo(null);
            yield return null;
        }

        /// <summary>The demo with its camera and light; <paramref name="configure"/> sets fields before Start runs (next frame).</summary>
        void CreateDemo(System.Action<CastleDemo> configure)
        {
            m_Root = new GameObject("CastleSmoke");
            var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
            camGo.transform.SetParent(m_Root.transform);
            camGo.AddComponent<Camera>();
            camGo.AddComponent<DemoCamera>();
            var light = new GameObject("Light").AddComponent<Light>();
            light.type = LightType.Directional;
            light.shadows = LightShadows.Soft;
            light.transform.SetParent(m_Root.transform);
            light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            m_Demo = m_Root.AddComponent<CastleDemo>();
            m_Demo.StartScene = 0;
            m_Demo.MaxBodies = 40960;
            m_Demo.Trees = 0;   // the castle alone unless a test plants them (and no foliage, whose debris-woken clumps would count as awake in the trees' test)
            m_Demo.FoliagePerTree = 0;
#if UNITY_EDITOR
            m_Demo.BrickMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx");
#endif
            configure?.Invoke(m_Demo);
        }

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            if (m_Root != null) Object.Destroy(m_Root);
            yield return null;
        }

        [UnityTest]
        public IEnumerator DryStackedCastleStands()
        {
            Assert.IsNotNull(m_Demo.BrickMesh, "brick mesh");
            Assert.AreEqual(3, m_Demo.Renderer.MeshRanges.Count, "the bricks are drawn with the brick model, the siege pools with the figure and arrow models");
            Assert.AreEqual(m_Demo.BrickCount, m_Demo.Renderer.MeshRanges[0].Count);
            for (int f = 0; f < 8; f++) yield return null;   // the range bounds read back
            Assert.AreEqual(1, m_Demo.Renderer.BoundedDraws, "the castle is drawn with its own bounds, the pools with the world's");
            Assert.IsTrue(m_Demo.Renderer.TryGetRangeBounds(m_Demo.FirstBrick, m_Demo.BrickCount, out var castleBounds));
            float side = CastlePlan.Presets[0].Side * Brick.Pitch * m_Demo.BrickScale, margin = m_Demo.Renderer.BoundsMargin;
            Assert.Less(castleBounds.size.x, side + 2f * margin + 4f, "the castle's bounds are its footprint plus the margin and a brick's radius");
            Assert.Less(castleBounds.size.z, side + 2f * margin + 4f);
            Assert.Less(math.abs(castleBounds.center.x), 2f); Assert.Less(math.abs(castleBounds.center.z), 2f);
            m_Demo.World.GetPosesSync(out var start, out _);
            float maxMove = 0f;
            int worst = m_Demo.FirstBrick;   // stays the first brick when the sleeping castle does not move at all
            var log = new System.Text.StringBuilder();
            for (int f = 1; f <= 180; f++)   // three seconds; the displacement is logged along the way to tell settling from creep
            {
                yield return null;
                if (f % 30 != 0) continue;
                m_Demo.World.GetPosesSync(out var pos, out var rot);
                maxMove = 0f;
                float meanMove = 0f;
                for (int i = m_Demo.FirstBrick; i < m_Demo.FirstBrick + m_Demo.BrickCount; i++)
                {
                    Assert.IsTrue(math.all(math.isfinite(pos[i])) && math.all(math.isfinite(rot[i])), $"brick {i} pose");
                    float d = math.distance(pos[i].xyz, start[i].xyz);
                    meanMove += d;
                    if (d > maxMove) { maxMove = d; worst = i; }
                }
                log.Append($"  frame {f}: max {maxMove * 1000f:F1} mm mean {meanMove / m_Demo.BrickCount * 1000f:F2} mm");
            }
            var stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags, $"capacity overflow {stats.OverflowFlags}");
            var b = m_Demo.Layout.Bricks[worst - m_Demo.FirstBrick];
            Debug.Log($"castle of {m_Demo.BrickCount} bricks ({Brick.BodyHeight * m_Demo.Spec.Scale * 1000f:F0} mm tall), worst brick at ({b.X}, {b.Z}) course {b.Layer}, " +
                $"contacts {stats.Contacts}, colours {stats.ColorsUsed}, step {stats.AvgStepMs:F2} ms\n{log}");
            Assert.Less(maxMove, 0.25f * Brick.BodyHeight * m_Demo.Spec.Scale, "no brick moved more than a quarter of its height: the castle stands");
        }

        [UnityTest]
        public IEnumerator LargestCastleStands()
        {
            m_Demo.Load(CastlePlan.Presets.Length - 1);
            Assert.Greater(m_Demo.BrickCount, 30000, "the largest preset");
            m_Demo.World.GetPosesSync(out var start, out _);
            float maxMove = 0f, maxDown = 0f, maxSide = 0f;
            int worst = m_Demo.FirstBrick;
            var log = new System.Text.StringBuilder();
            AvbdGpuStats stats = default;
            for (int f = 1; f <= 240; f++)
            {
                yield return null;
                if (f % 60 != 0) continue;
                m_Demo.World.GetPosesSync(out var pos, out _);
                stats = m_Demo.World.GetStatsSync();
                Assert.AreEqual(0, stats.OverflowFlags, $"capacity overflow {stats.OverflowFlags}");
                maxMove = maxDown = maxSide = 0f;
                for (int i = m_Demo.FirstBrick; i < m_Demo.FirstBrick + m_Demo.BrickCount; i++)
                {
                    Assert.IsTrue(math.all(math.isfinite(pos[i])), $"brick {i} position");
                    float3 d = pos[i].xyz - start[i].xyz;
                    maxDown = math.max(maxDown, -d.y);
                    maxSide = math.max(maxSide, math.length(d.xz));
                    if (math.length(d) > maxMove) { maxMove = math.length(d); worst = i; }
                }
                log.Append($"  frame {f}: max {maxMove * 1000f:F0} mm (down {maxDown * 1000f:F0}, sideways {maxSide * 1000f:F0})");
            }
            var b = m_Demo.Layout.Bricks[worst - m_Demo.FirstBrick];
            Debug.Log($"largest castle: {m_Demo.BrickCount} bricks, {stats.Contacts} contacts, {stats.ColorsUsed} colours, step {stats.AvgStepMs:F2} ms; " +
                $"worst brick at ({b.X}, {b.Z}) course {b.Layer}\n{log}");
            Assert.Less(maxSide, 0.5f * Brick.BodyHeight * m_Demo.Spec.Scale, "nothing slid or toppled: the largest castle stands");
        }

        [UnityTest]
        public IEnumerator CannonballKnocksBricksOff()
        {
            for (int f = 0; f < 10; f++) yield return null;
            m_Demo.World.GetPosesSync(out var start, out _);
            // a heavy cube fired at the middle of the front wall from outside
            var plan = CastlePlan.Presets[0];
            float s = m_Demo.Spec.Scale;
            float side = plan.Side * Brick.Pitch * s;
            float3 target = new float3(side * 0.15f, 3f * Brick.BodyHeight * s, -side * 0.5f + BrickCastle.TowerOut * Brick.Pitch * s);
            float3 from = target + new float3(0f, 0.2f * s, -1.5f * s);
            float cube = 0.15f * s;
            int ball = m_Demo.World.AddBody(new float3(cube), m_Demo.ShotMass / (cube * cube * cube), 0.5f, from, quaternion.identity, math.normalize(target - from) * m_Demo.ShotVelocity * math.sqrt(s));
            for (int f = 0; f < 120; f++) yield return null;
            m_Demo.World.GetPosesSync(out var pos, out _);
            int moved = 0;
            for (int i = m_Demo.FirstBrick; i < m_Demo.FirstBrick + m_Demo.BrickCount; i++)
                if (math.distance(pos[i].xyz, start[i].xyz) > Brick.BodyHeight * s) moved++;
            Debug.Log($"cannonball: {moved} bricks displaced by more than a brick height, ball at {pos[ball].xyz}");
            Assert.Greater(moved, 0, "the cannonball displaced some bricks");
            Assert.Less(moved, m_Demo.BrickCount / 2, "the castle did not collapse entirely");
        }

        [UnityTest]
        public IEnumerator SnappedCastleHoldsAndBreaksUnderTheCannonball()
        {
            m_Demo.Snap = true;
            m_Demo.Load(0);
            Assert.Greater(m_Demo.World.JointCount, m_Demo.BrickCount, "at least one snap joint per brick");
            for (int f = 0; f < 60; f++) yield return null;
            var stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags, $"capacity overflow {stats.OverflowFlags}");
            Assert.AreEqual(0, BrokenJoints(), "the snapped castle holds together under its own weight: " + BrokenSummary());

            // the cannonball into the gatehouse pier breaks the snaps it hits and nothing else comes apart
            var plan = CastlePlan.Presets[0];
            float s = m_Demo.Spec.Scale;
            float side = plan.Side * Brick.Pitch * s;
            float3 target = new float3(side * 0.15f, 3f * Brick.BodyHeight * s, -side * 0.5f + BrickCastle.TowerOut * Brick.Pitch * s);
            float3 from = target + new float3(0f, 0.2f * s, -1.5f * s);
            float cube = m_Demo.ShotCube * s;
            m_Demo.World.AddBody(new float3(cube), m_Demo.ShotMass / (cube * cube * cube), 0.5f, from, quaternion.identity, math.normalize(target - from) * m_Demo.ShotVelocity * math.sqrt(s));
            for (int f = 0; f < 120; f++) yield return null;
            int broken = BrokenJoints();
            Debug.Log($"cannonball into the snapped castle: {broken} of {m_Demo.World.JointCount} joints broke");
            Assert.Greater(broken, 0, "the impact breaks some snaps");
            Assert.Less(broken, m_Demo.World.JointCount / 2, "most of the castle stays snapped");
            m_Demo.World.GetPosesSync(out var pos, out _);
            for (int i = 0; i < m_Demo.World.BodyCount; i++) Assert.IsTrue(math.all(math.isfinite(pos[i])), $"body {i} position");
        }

        /// <summary>The Outpost on hills: a plateau is levelled under it, the bricks rest on the plateau, the castle stands, and the
        /// armies of its siege spawn on the surface around it and march in over the slopes.</summary>
        [UnityTest]
        public IEnumerator CastleStandsOnHillsAndTheArmiesWalkThem()
        {
            m_Demo.TerrainParams.Preset = TerrainPreset.Hills;
            m_Demo.TerrainParams.Seed = 3;
            m_Demo.Load(0);
            var field = m_Demo.World.Terrain;
            Assert.IsNotNull(field, "the demo set a terrain");
            Assert.IsTrue(m_Demo.TerrainView.Visible, "and draws it");
            Assert.Greater(field.MaxHeight - field.MinHeight, 4f, "hills");
            float plateau = m_Demo.Plateau;
            Assert.AreEqual(plateau, m_Demo.Spec.Origin.y, "the castle stands on the plateau");
            float side = CastlePlan.Presets[0].Side * Brick.Pitch * m_Demo.Spec.Scale;
            Assert.AreEqual(plateau, field.Height(new float2(0.4f * side, -0.4f * side)), 1e-4f, "the plateau is level across the footprint");
            Assert.GreaterOrEqual(field.MaxOver(new float2(-0.45f * side), new float2(0.45f * side)), plateau - 1e-3f, "the mip bounds it (its 8 m blocks reach into the skirt)");
            Assert.Greater(m_Demo.World.TerrainManifoldsSync(), 100, "the ground course rests on the terrain (asleep by now: in the cold store)");

            m_Demo.World.GetPosesSync(out var start, out _);
            float maxMove = 0f;
            for (int f = 1; f <= 120; f++)
            {
                yield return null;
                if (f % 30 != 0) continue;
                m_Demo.World.GetPosesSync(out var pos, out _);
                maxMove = 0f;
                for (int i = m_Demo.FirstBrick; i < m_Demo.FirstBrick + m_Demo.BrickCount; i++)
                {
                    Assert.IsTrue(math.all(math.isfinite(pos[i])), $"brick {i} position");
                    maxMove = math.max(maxMove, math.distance(pos[i].xyz, start[i].xyz));
                }
            }
            Assert.Less(maxMove, 0.25f * Brick.BodyHeight * m_Demo.Spec.Scale, "the castle stands on its plateau");

            // the siege: units spawn standing on the surface wherever it is, and the attackers walk in over the slopes
            m_Demo.ToggleSiege();
            var siege = m_Demo.Siege;
            Assert.AreSame(field, siege.Terrain);
            yield return null;
            yield return null;
            m_Demo.World.GetPosesSync(out var spawned, out _);
            float meanDistance0 = 0f;
            int attackers = 0;
            foreach (var u in siege.UnitList)
            {
                float3 p = spawned[u.Body].xyz;
                Assert.AreEqual(siege.StandHeight(p.xz), p.y, 0.3f, $"unit {u.Body} ({u.Kind}, team {u.Team}) stands on the ground at {p}");
                if (u.Team == SiegeSystem.Attackers) { meanDistance0 += math.length(p.xz); attackers++; }
            }
            meanDistance0 /= attackers;
            for (int f = 0; f < 240; f++) yield return null;
            m_Demo.World.GetPosesSync(out var later, out _);
            float meanDistance1 = 0f;
            foreach (var u in siege.UnitList)
            {
                float3 p = later[u.Body].xyz;
                Assert.IsTrue(math.all(math.isfinite(p)), $"unit {u.Body} position");
                Assert.Greater(p.y, siege.GroundHeight(p.xz) - 1f, $"unit {u.Body} stays above the terrain at {p}");
                if (u.Team == SiegeSystem.Attackers) meanDistance1 += math.length(p.xz);
            }
            meanDistance1 /= attackers;
            Debug.Log($"siege on hills: plateau {plateau:F2} m, terrain {field.MinHeight:F1} .. {field.MaxHeight:F1} m, attackers {meanDistance0:F1} -> {meanDistance1:F1} m from the centre after 4 s");
            Assert.Less(meanDistance1, meanDistance0 - 2f, "the attackers marched in over the slopes");
            var stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags, $"capacity overflow {stats.OverflowFlags}");

            // back to the flat ground: the terrain goes away
            m_Demo.TerrainParams.Preset = TerrainPreset.None;
            m_Demo.Load(0);
            Assert.IsNull(m_Demo.World.Terrain);
            Assert.IsFalse(m_Demo.TerrainView.Visible);
            Assert.AreEqual(0f, m_Demo.Plateau);
        }

        /// <summary>Seven Strongholds built asleep around an eighth: 137k bricks in a world that simulates 40 960 at a time. They cost
        /// nothing until a cannonball hits one, which wakes it whole (an island), settles it and puts it back to sleep; nothing
        /// overflows and the centre castle stands throughout.</summary>
        [UnityTest]
        public IEnumerator OutlyingCastlesSleepBeyondTheActiveCapacity()
        {
            Object.Destroy(m_Root);
            yield return null;
            CreateDemo(d => { d.StartScene = 7; d.Outlying = 7; d.OutlyingBodies = 131072; d.TerrainParams.Preset = TerrainPreset.Hills; });
            yield return null;   // Start: the world with room for the copies, the scene with its 180 settle steps
            Assert.AreEqual(7, m_Demo.OutlyingCount, "seven copies fit the reserved bodies");
            int n = m_Demo.BrickCount;
            Assert.Greater(n * 8, m_Demo.World.Config.MaxActive, "more bricks than the world simulates at once");
            int drawnBricks = 0;
            for (int k = 0; k < 8; k++) { Assert.AreEqual(n, m_Demo.Renderer.MeshRanges[k].Count, "one range per castle, culled on its own bounds"); drawnBricks += m_Demo.Renderer.MeshRanges[k].Count; }
            Assert.AreEqual(n * 8, drawnBricks, "the copies are drawn with the brick model too");
            var stats = m_Demo.World.GetStatsSync();
            {
                var words = m_Demo.World.GetSleepSync();
                var sb = new System.Text.StringBuilder();
                for (int k = 0; k < 7; k++)
                {
                    int awake = 0, inGrid = 0;
                    for (int b = m_Demo.FirstOutlyingBrick + k * n; b < m_Demo.FirstOutlyingBrick + (k + 1) * n; b++) { if (!GpuBodySleep.IsAsleep(words[b])) awake++; if (GpuBodySleep.IsInGrid(words[b])) inGrid++; }
                    sb.Append($" copy {k} at {m_Demo.OutlyingCentres[k]}: {awake} awake, {inGrid} in the grid;");
                }
                Debug.Log($"after the settle steps: {m_Demo.World.BodyCount} bodies, active {stats.Active}, hot {stats.Hot}, asleep {stats.Sleeping}, grid {stats.SleepGrid}, flags {stats.OverflowFlags}, rebuilds {stats.Rebuilds};{sb}");
            }
            Assert.AreEqual(0, stats.OverflowFlags, $"no overflow after the settle steps (flags {stats.OverflowFlags})");
            Assert.GreaterOrEqual(stats.SleepGrid, 7 * n, "the copies are in the sleeping grid");
            Assert.LessOrEqual(stats.Active, n, "at most the centre castle is awake");
            Assert.LessOrEqual(stats.Hot, n + 1 + 16, "the hot list holds the centre castle at most");
            m_Demo.World.GetPosesSync(out var start, out _);
            for (int f = 0; f < 60; f++) yield return null;
            stats = m_Demo.World.GetStatsSync();
            Debug.Log($"outlying castles: {m_Demo.World.BodyCount} bodies, {stats.Sleeping} asleep, {stats.Hot} hot, {stats.Active} active, grid {stats.SleepGrid}, cold {stats.ColdManifolds}, step {stats.AvgStepMs:F2} ms");
            Assert.AreEqual(0, stats.OverflowFlags);

            // a cannonball into the first copy wakes it (and it alone), which settles and sleeps again
            float2 centre = m_Demo.OutlyingCentres[0];
            float s = m_Demo.Spec.Scale;
            float3 target = new float3(centre.x, m_Demo.Plateau + 2f * s, centre.y);
            float3 from = target + math.normalize(new float3(-centre.x, 0f, -centre.y)) * 12f * s + new float3(0f, 0.5f * s, 0f);
            float cube = m_Demo.ShotCube * s;
            m_Demo.World.AddBody(new float3(cube), m_Demo.ShotMass / (cube * cube * cube), 0.5f, from, quaternion.identity, math.normalize(target - from) * m_Demo.ShotVelocity * math.sqrt(s));
            int first = m_Demo.FirstOutlyingBrick;
            bool woke = false;
            for (int f = 0; f < 180 && !woke; f++)
            {
                yield return null;
                var words = m_Demo.World.GetSleepSync();
                int awake = 0;
                for (int b = first; b < first + n; b++) if (!GpuBodySleep.IsAsleep(words[b])) awake++;
                if (awake == 0) continue;
                woke = true;
                Assert.Greater(awake, n / 2, "the hit copy woke as an island");
                for (int b = first + n; b < first + 7 * n; b++) Assert.IsTrue(GpuBodySleep.IsAsleep(words[b]), $"brick {b} of another copy sleeps on");
                stats = m_Demo.World.GetStatsSync();
                Assert.AreEqual(0, stats.OverflowFlags, "the woken copy fits the pools");
                Assert.LessOrEqual(stats.Active, 2 * n + 1, "one castle awake at most on top of the centre one");
            }
            Assert.IsTrue(woke, "the cannonball hit the copy");
            for (int f = 0; f < 300; f++) yield return null;
            m_Demo.World.GetPosesSync(out var pos, out _);
            for (int i = m_Demo.FirstBrick; i < m_Demo.FirstBrick + n; i++) Assert.IsTrue(math.all(math.isfinite(pos[i])), $"brick {i} position");
            float maxMove = 0f;
            for (int b = first + n; b < first + 7 * n; b++) maxMove = math.max(maxMove, math.distance(pos[b].xyz, start[b].xyz));
            Assert.AreEqual(0f, maxMove, "the other copies never moved");
            stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags);
            Debug.Log($"after the shot: {stats.Sleeping} asleep, {stats.Active} active, hot {stats.Hot}, cold {stats.ColdManifolds} (dead {stats.ColdDead}), step {stats.AvgStepMs:F2} ms");
        }

        /// <summary>Twenty trees of the six documents planted around the Castle on the hills: every tree outside the castle's plateau
        /// core and the armies' bands, clear of the others, on its own ground; all of them asleep and in the sleeping grid from the
        /// first step, drawn with the piece models; a cannonball into one wakes that tree alone and throws its crown to pieces (a tree
        /// weighs a quarter of the castle's bricks), the rest are at most nudged by the debris, and nothing but the hit tree and its
        /// debris is awake afterwards.</summary>
        [UnityTest]
        public IEnumerator TreesSleepAroundTheCastleUntilHit()
        {
            Object.Destroy(m_Root);
            yield return null;
            CreateDemo(d => { d.StartScene = 3; d.Trees = 20; d.Snap = true; d.BrickMass = 1f; d.SnapFractureLateral = 800f; d.SnapFractureTension = 300f; d.ShotMass = 20f; d.TerrainParams.Preset = TerrainPreset.Hills; });   // the scene's configuration
            yield return null;   // Start: the world with room for the trees, the scene with its 180 settle steps
            Assert.AreEqual(6, m_Demo.TreeDocuments.Length, "the six tree documents of Resources/Trees");
            foreach (var kind in m_Demo.TreeKinds) { Assert.IsNotNull(kind); Assert.That(kind.Parts.Count, Is.InRange(1000, 3000), $"{kind.Name}: a thousand to three thousand pieces"); }
            Assert.AreEqual(20, m_Demo.TreeCount, "twenty trees were placed");
            Assert.AreEqual(m_Demo.FirstOutlyingBrick, m_Demo.FirstTreePiece, "the trees come right after the castle");
            var plan = CastlePlan.Presets[3];
            float side = plan.Side * Brick.Pitch * m_Demo.BrickScale, half = side * 0.5f + 4f * Brick.Pitch * m_Demo.BrickScale;
            var sp = m_Demo.SiegeParams;
            float reach = side * 0.5f - BrickCastle.TowerOut * Brick.Pitch * m_Demo.BrickScale + sp.AttackDistance + (sp.Ranks - 1) * sp.RankSpacing + sp.MarchDistance;
            float bandHalf = (sp.ArchersPerRank - 1) * 0.5f * sp.ColumnSpacing;
            var placements = m_Demo.TreePlacements;
            int pieces = 0;
            var kinds = new System.Collections.Generic.HashSet<int>();
            for (int i = 0; i < placements.Count; i++)
            {
                var p = placements[i];
                pieces += p.Count;
                kinds.Add(p.Kind);
                Assert.AreEqual(m_Demo.TreeKinds[p.Kind].Parts.Count, p.Count, "every piece of the document became a body");
                Assert.Greater(math.cmax(math.abs(p.Centre)), half + p.Radius, $"tree {i} clears the castle's plateau core");
                Assert.IsFalse((math.abs(p.Centre.x) < bandHalf + p.Radius && math.abs(p.Centre.y) < reach + p.Radius) || (math.abs(p.Centre.y) < bandHalf + p.Radius && math.abs(p.Centre.x) < reach + p.Radius), $"tree {i} is off the armies' bands");
                for (int j = 0; j < i; j++) Assert.Greater(math.distance(p.Centre, placements[j].Centre), p.Radius + placements[j].Radius, $"trees {i} and {j} do not touch");
                Assert.AreEqual(m_Demo.World.Terrain.Height(p.Centre), p.Ground, 0.05f, $"tree {i} stands on its ground");
            }
            Assert.AreEqual(pieces, m_Demo.TreePieces);
            Assert.GreaterOrEqual(kinds.Count, 4, "several kinds of tree");
            int drawn = 0;
            foreach (var r in m_Demo.Renderer.MeshRanges) { Assert.IsNotNull(r.Mesh); if (r.Start >= m_Demo.FirstTreePiece && r.Start < m_Demo.FirstTreePiece + pieces) drawn += r.Count; }
            Assert.AreEqual(pieces, drawn, "every piece of every tree is drawn with its model");
            Assert.Greater(m_Demo.World.JointCount, pieces, "the trees are snapped like the castle");

            var stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags, $"no overflow after the settle steps (flags {stats.OverflowFlags})");
            var words = m_Demo.World.GetSleepSync();
            int first = m_Demo.FirstTreePiece;
            for (int b = first; b < first + pieces; b++) { Assert.IsTrue(GpuBodySleep.IsAsleep(words[b]), $"tree piece {b} sleeps"); Assert.IsTrue(GpuBodySleep.IsInGrid(words[b]), $"tree piece {b} is in the sleeping grid"); }
            Assert.LessOrEqual(stats.Active, m_Demo.BrickCount, "at most the castle is awake");
            m_Demo.World.GetPosesSync(out var start, out _);
            for (int f = 0; f < 60; f++) yield return null;
            stats = m_Demo.World.GetStatsSync();
            Debug.Log($"trees: {m_Demo.TreeCount} trees, {pieces} pieces, {m_Demo.World.BodyCount} bodies, {stats.Sleeping} asleep, {stats.Hot} hot, {stats.Active} active, grid {stats.SleepGrid}, cold {stats.ColdManifolds}, step {stats.AvgStepMs:F2} ms");
            Assert.AreEqual(0, stats.OverflowFlags);

            // every range but the pools is drawn with bounds read back from the GPU; a tree's ranges lie within its placement
            var renderer = m_Demo.Renderer;
            int culled = 0;
            foreach (var r in renderer.MeshRanges) if (!r.NoCulling) culled++;
            Assert.AreEqual(culled, renderer.BoundedDraws, "every range but the siege pools is drawn with its own bounds");
            var tree0 = placements[0];
            float3 tree0Extent = m_Demo.TreeKinds[tree0.Kind].Extent * PieceCatalog.GridToUnity * m_Demo.Spec.Scale;
            float tree0Reach = tree0.Radius + renderer.BoundsMargin + 1f;
            int treeRanges = 0;
            foreach (var r in renderer.MeshRanges)
            {
                if (r.Start < tree0.First || r.Start >= tree0.First + tree0.Count) continue;
                treeRanges++;
                Assert.IsTrue(renderer.TryGetRangeBounds(r.Start, r.Count, out var rb), "the range's bounds are in");
                Assert.Less(math.abs(rb.center.x - tree0.Centre.x), tree0Reach, "bounds around the tree");
                Assert.Less(math.abs(rb.center.z - tree0.Centre.y), tree0Reach);
                Assert.Less(math.max(rb.size.x, rb.size.z), 2f * tree0Reach + 4f, "bounds no wider than the crown plus the margin and a piece's radius");
                Assert.Greater(rb.min.y, tree0.Ground - renderer.BoundsMargin - 2f);
                Assert.Less(rb.max.y, tree0.Ground + tree0Extent.y + renderer.BoundsMargin + 2f);
            }
            Assert.Greater(treeRanges, 0);

            // a cannonball into the crown of the first tree wakes it (and it alone)
            var hit = placements[0];
            float s = m_Demo.Spec.Scale;
            float3 extent = m_Demo.TreeKinds[hit.Kind].Extent * PieceCatalog.GridToUnity * s;
            float3 target = new float3(hit.Centre.x, hit.Ground + 0.6f * extent.y, hit.Centre.y);
            float3 from = target + new float3(0.3f * s, 10f * s, 0f);   // from high above: nothing else in the way
            float cube = m_Demo.ShotCube * s;
            m_Demo.World.AddBody(new float3(cube), m_Demo.ShotMass / (cube * cube * cube), 0.5f, from, quaternion.identity, math.normalize(target - from) * m_Demo.ShotVelocity * math.sqrt(s));
            bool woke = false;
            for (int f = 0; f < 180 && !woke; f++)
            {
                yield return null;
                words = m_Demo.World.GetSleepSync();
                int awake = 0;
                for (int b = hit.First; b < hit.First + hit.Count; b++) if (!GpuBodySleep.IsAsleep(words[b])) awake++;
                if (awake == 0) continue;
                woke = true;
                Assert.Greater(awake, hit.Count / 2, "the hit tree woke as an island");
                for (int i = 1; i < placements.Count; i++)
                    for (int b = placements[i].First; b < placements[i].First + placements[i].Count; b++) Assert.IsTrue(GpuBodySleep.IsAsleep(words[b]), $"piece {b} of tree {i} sleeps on");
                stats = m_Demo.World.GetStatsSync();
                Assert.AreEqual(0, stats.OverflowFlags, "the woken tree fits the pools");
            }
            Assert.IsTrue(woke, "the cannonball hit the tree");
            for (int f = 0; f < 300; f++) yield return null;
            m_Demo.World.GetPosesSync(out var pos, out _);
            float maxMove = 0f;
            for (int i = 1; i < placements.Count; i++)
                for (int b = placements[i].First; b < placements[i].First + placements[i].Count; b++) maxMove = math.max(maxMove, math.distance(pos[b].xyz, start[b].xyz));
            Assert.Less(maxMove, 0.5f * 1.2f * PieceCatalog.GridToUnity * s, $"the other trees were nudged by debris at most ({maxMove * 1000f:F0} mm), none knocked down");
            int fell = 0, brokenTree = 0;
            float meanMove = 0f;
            for (int b = hit.First; b < hit.First + hit.Count; b++)
            {
                Assert.IsTrue(math.all(math.isfinite(pos[b])), $"piece {b} position");
                float dist = math.distance(pos[b].xyz, start[b].xyz);
                meanMove += dist / hit.Count;
                if (dist > 1.2f * PieceCatalog.GridToUnity * s) fell++;
            }
            var jointStates = m_Demo.World.GetJointStatesSync();
            for (int j = 0; j < m_Demo.World.JointCount; j++)
            {
                var d = m_Demo.World.GetJointDef(j);
                if (jointStates[j].Broken != 0 && d.BodyB >= hit.First && d.BodyB < hit.First + hit.Count) brokenTree++;
            }
            stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags);
            Debug.Log($"after the shot: tree 0 ({m_Demo.TreeKinds[hit.Kind].Name}) lost {fell} of {hit.Count} pieces (a brick height away), moved {meanMove * 1000f:F0} mm on average, {brokenTree} snaps broken; {stats.Sleeping} asleep, {stats.Active} active, hot {stats.Hot}, cold {stats.ColdManifolds}, step {stats.AvgStepMs:F2} ms");
            Assert.Greater(fell, 0, "the cannonball took pieces off the tree");
            Assert.Greater(brokenTree, 0, "snaps of the tree broke");
            Assert.LessOrEqual(stats.Active, hit.Count + 1, "nothing but the hit tree, its debris and the ball is awake");
        }

        /// <summary>Six clumps of foliage per tree - a hundred and twenty of the twenty documents - scattered around the Castle on the
        /// hills with the twenty trees: every clump off the castle's plateau core and the armies' bands, clear of the trees' crowns and
        /// of the other clumps by its bounding square (two cells and a blend apart where either stands on a terrace), on level ground
        /// over its whole footprint; the clumps of a grid cell built as one cluster drawn in a range per kind of plate, all of them
        /// asleep and in the sleeping grid from the first step; a cannonball into a clump wakes that clump alone although its bodies
        /// lie between its cluster-mates', and the rest of the foliage is at most nudged by the debris.</summary>
        [UnityTest]
        public IEnumerator FoliageSleepsAroundTheCastleUntilHit()
        {
            Object.Destroy(m_Root);
            yield return null;
            CreateDemo(d => { d.StartScene = 3; d.Trees = 20; d.Snap = true; d.BrickMass = 1f; d.SnapFractureLateral = 800f; d.SnapFractureTension = 300f; d.ShotMass = 20f; d.FoliagePerTree = 6; d.FoliageMass = 1f; d.TerrainParams = TerrainSettings.CastleScene; });   // the scene's configuration, terrain included (the level ground around the castle, 3 m cells)
            yield return null;
            Assert.AreEqual(20, m_Demo.FoliageDocuments.Length, "the twenty foliage documents of Resources/Foliage");
            foreach (var kind in m_Demo.FoliageKinds) { Assert.IsNotNull(kind); Assert.That(kind.Parts.Count, Is.InRange(100, 600), $"{kind.Name}: a hundred to six hundred pieces"); }
            Assert.AreEqual(20, m_Demo.TreeCount, "twenty trees were placed");
            Assert.AreEqual(120, m_Demo.FoliageCount, "six clumps per tree were placed");
            Assert.AreEqual(m_Demo.FirstTreePiece + m_Demo.TreePieces, m_Demo.FirstFoliagePiece, "the foliage comes right after the trees");
            var plan = CastlePlan.Presets[3];
            float side = plan.Side * Brick.Pitch * m_Demo.BrickScale, half = side * 0.5f + 4f * Brick.Pitch * m_Demo.BrickScale;
            var sp = m_Demo.SiegeParams;
            float reach = side * 0.5f - BrickCastle.TowerOut * Brick.Pitch * m_Demo.BrickScale + sp.AttackDistance + (sp.Ranks - 1) * sp.RankSpacing + sp.MarchDistance;
            float bandHalf = (sp.ArchersPerRank - 1) * 0.5f * sp.ColumnSpacing;
            float cell = math.cmax(m_Demo.World.Terrain.Cell);
            var trees = m_Demo.TreePlacements;
            var clumps = m_Demo.FoliagePlacements;
            int pieces = 0, terraced = 0;
            var kinds = new System.Collections.Generic.HashSet<int>();
            var clusters = new System.Collections.Generic.HashSet<int>();
            for (int i = 0; i < clumps.Count; i++)
            {
                var f = clumps[i];
                pieces += f.Count;
                kinds.Add(f.Kind);
                clusters.Add(f.Cluster);
                if (f.Terraced) terraced++;
                Assert.AreEqual(m_Demo.FoliageKinds[f.Kind].Parts.Count, f.Count, "every piece of the document became a body");
                Assert.Greater(math.cmax(math.abs(f.Centre)), half + f.Radius, $"clump {i} clears the castle's plateau core");
                Assert.IsFalse((math.abs(f.Centre.x) < bandHalf + f.Radius && math.abs(f.Centre.y) < reach + f.Radius) || (math.abs(f.Centre.y) < bandHalf + f.Radius && math.abs(f.Centre.x) < reach + f.Radius), $"clump {i} is off the armies' bands");
                foreach (var t in trees) Assert.Greater(math.cmax(math.abs(f.Centre - t.Centre)), t.Radius + f.Radius, $"clump {i} is clear of the trees");
                for (int j = 0; j < i; j++)
                {
                    float apart = f.Terraced || clumps[j].Terraced ? 2f * cell + 1f : 0.5f;
                    Assert.Greater(math.cmax(math.abs(f.Centre - clumps[j].Centre)), f.Radius + clumps[j].Radius + apart, $"clumps {i} and {j} stand apart");
                }
                // level ground under the whole footprint (a terrace where the hills were), so that every plate of the ground course rests on it
                foreach (var corner in new[] { new float2(-1f, -1f), new float2(1f, -1f), new float2(-1f, 1f), new float2(1f, 1f), float2.zero })
                    Assert.AreEqual(f.Ground, m_Demo.World.Terrain.Height(f.Centre + corner * f.Radius), 0.01f, $"clump {i} ({m_Demo.FoliageKinds[f.Kind].Name}) stands on level ground");
                for (int k = 1; k < f.Count; k++) Assert.AreNotEqual(m_Demo.FoliageBody(f, 0), m_Demo.FoliageBody(f, k));
            }
            Assert.AreEqual(pieces, m_Demo.FoliagePieces);
            Assert.GreaterOrEqual(kinds.Count, 12, "many kinds of foliage");
            Assert.Greater(terraced, 0, "some clumps stand on the hills, on terraces of their own");
            Assert.Less(terraced, clumps.Count, "some clumps stand on the level ground around the castle");
            Assert.AreEqual(clusters.Count, m_Demo.FoliageClusterCount, "every cluster holds clumps");
            Assert.That(m_Demo.FoliageClusterCount, Is.InRange(4, CastleDemo.FoliageGrid * CastleDemo.FoliageGrid), "the clumps are clustered by grid cell");
            int drawn = 0, foliageRanges = 0;
            foreach (var r in m_Demo.Renderer.MeshRanges)
            {
                Assert.IsNotNull(r.Mesh);
                if (r.Start < m_Demo.FirstFoliagePiece || r.Start >= m_Demo.FirstFoliagePiece + pieces) continue;
                drawn += r.Count;
                foliageRanges++;
            }
            Assert.AreEqual(pieces, drawn, "every piece of every clump is drawn with its model");
            Assert.LessOrEqual(foliageRanges, 5 * m_Demo.FoliageClusterCount, "a cluster is drawn in a range per kind of plate, not one per clump");
            Debug.Log($"foliage: {clumps.Count} clumps of {kinds.Count} kinds, {pieces} pieces, {terraced} on terraces, {m_Demo.FoliageClusterCount} clusters drawn in {foliageRanges} ranges ({m_Demo.Renderer.MeshRanges.Count} ranges in all)");

            var stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags, $"no overflow after the settle steps (flags {stats.OverflowFlags})");
            var words = m_Demo.World.GetSleepSync();
            int first = m_Demo.FirstFoliagePiece;
            for (int b = first; b < first + pieces; b++) { Assert.IsTrue(GpuBodySleep.IsAsleep(words[b]), $"foliage piece {b} sleeps"); Assert.IsTrue(GpuBodySleep.IsInGrid(words[b]), $"foliage piece {b} is in the sleeping grid"); }
            Assert.LessOrEqual(stats.Active, m_Demo.BrickCount, "at most the castle is awake");
            m_Demo.World.GetPosesSync(out var start, out _);
            for (int f = 0; f < 60; f++) yield return null;
            stats = m_Demo.World.GetStatsSync();
            Debug.Log($"foliage: {m_Demo.World.BodyCount} bodies, {stats.Sleeping} asleep, {stats.Hot} hot, {stats.Active} active, grid {stats.SleepGrid}, cold {stats.ColdManifolds}, step {stats.AvgStepMs:F2} ms");
            Assert.AreEqual(0, stats.OverflowFlags);
            int culled = 0;
            foreach (var r in m_Demo.Renderer.MeshRanges) if (!r.NoCulling) culled++;
            Assert.AreEqual(culled, m_Demo.Renderer.BoundedDraws, "every range but the siege pools is drawn with its own bounds");

            // a cannonball into the clump farthest from everything else wakes it, and it alone, although its bodies lie between its
            // cluster-mates' (one island per clump)
            int target = 0;
            float bestGap = -1f;
            for (int i = 0; i < clumps.Count; i++)
            {
                float gap = float.PositiveInfinity;
                foreach (var t in trees) gap = math.min(gap, math.cmax(math.abs(clumps[i].Centre - t.Centre)) - t.Radius - clumps[i].Radius);
                for (int j = 0; j < clumps.Count; j++) if (j != i) gap = math.min(gap, math.cmax(math.abs(clumps[i].Centre - clumps[j].Centre)) - clumps[j].Radius - clumps[i].Radius);
                if (gap > bestGap) { bestGap = gap; target = i; }
            }
            var hit = clumps[target];
            float s = m_Demo.Spec.Scale;
            float3 extent = m_Demo.FoliageKinds[hit.Kind].Extent * PieceCatalog.GridToUnity * s;
            float3 aim = new float3(hit.Centre.x, hit.Ground + 0.5f * extent.y, hit.Centre.y);
            float3 from = aim + new float3(0.3f * s, 10f * s, 0f);   // from high above: nothing else in the way
            float cube = m_Demo.ShotCube * s;
            m_Demo.World.AddBody(new float3(cube), m_Demo.ShotMass / (cube * cube * cube), 0.5f, from, quaternion.identity, math.normalize(aim - from) * m_Demo.ShotVelocity * math.sqrt(s));
            bool woke = false;
            for (int f = 0; f < 180 && !woke; f++)
            {
                yield return null;
                words = m_Demo.World.GetSleepSync();
                int awake = 0;
                for (int k = 0; k < hit.Count; k++) if (!GpuBodySleep.IsAsleep(words[m_Demo.FoliageBody(hit, k)])) awake++;
                if (awake == 0) continue;
                woke = true;
                Assert.Greater(awake, hit.Count / 2, "the hit clump woke as an island");
                for (int i = 0; i < clumps.Count; i++)
                {
                    if (i == target) continue;
                    for (int k = 0; k < clumps[i].Count; k++) Assert.IsTrue(GpuBodySleep.IsAsleep(words[m_Demo.FoliageBody(clumps[i], k)]), $"piece {k} of clump {i} sleeps on");
                }
                foreach (var t in trees) for (int b = t.First; b < t.First + t.Count; b++) Assert.IsTrue(GpuBodySleep.IsAsleep(words[b]), $"tree piece {b} sleeps on");
                stats = m_Demo.World.GetStatsSync();
                Assert.AreEqual(0, stats.OverflowFlags, "the woken clump fits the pools");
            }
            Assert.IsTrue(woke, "the cannonball hit the clump");
            for (int f = 0; f < 300; f++) yield return null;
            m_Demo.World.GetPosesSync(out var pos, out _);
            float maxMove = 0f;
            for (int i = 0; i < clumps.Count; i++)
            {
                if (i == target) continue;
                for (int k = 0; k < clumps[i].Count; k++) { int b = m_Demo.FoliageBody(clumps[i], k); maxMove = math.max(maxMove, math.distance(pos[b].xyz, start[b].xyz)); }
            }
            Assert.Less(maxMove, 0.5f * 1.2f * PieceCatalog.GridToUnity * s, $"the other clumps were nudged by debris at most ({maxMove * 1000f:F0} mm), none knocked over");
            int fell = 0;
            float meanMove = 0f;
            for (int k = 0; k < hit.Count; k++)
            {
                int b = m_Demo.FoliageBody(hit, k);
                Assert.IsTrue(math.all(math.isfinite(pos[b])), $"piece {b} position");
                float dist = math.distance(pos[b].xyz, start[b].xyz);
                meanMove += dist / hit.Count;
                if (dist > 1.2f * PieceCatalog.GridToUnity * s) fell++;
            }
            stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags);
            Debug.Log($"after the shot: clump {target} ({m_Demo.FoliageKinds[hit.Kind].Name}, {hit.Count} pieces, {bestGap:F1} m clear) lost {fell} pieces (a brick height away), moved {meanMove * 1000f:F0} mm on average; {stats.Sleeping} asleep, {stats.Active} active, hot {stats.Hot}, cold {stats.ColdManifolds}, step {stats.AvgStepMs:F2} ms");
            Assert.Greater(fell, 0, "the cannonball took pieces off the clump");
        }

        int BrokenJoints()
        {
            var states = m_Demo.World.GetJointStatesSync();
            int broken = 0;
            for (int j = 0; j < m_Demo.World.JointCount; j++) if (states[j].Broken != 0) broken++;
            return broken;
        }

        /// <summary>The bricks whose joints broke, with their grid position and course (diagnostics).</summary>
        string BrokenSummary()
        {
            var states = m_Demo.World.GetJointStatesSync();
            var sb = new System.Text.StringBuilder();
            int listed = 0;
            for (int j = 0; j < m_Demo.World.JointCount && listed < 40; j++)
            {
                if (states[j].Broken == 0) continue;
                var d = m_Demo.World.GetJointDef(j);
                var b = m_Demo.Layout.Bricks[d.BodyB - m_Demo.FirstBrick];
                string below = d.BodyA < 0 ? "world" : $"brick {d.BodyA - m_Demo.FirstBrick}";
                sb.Append($" [joint {j}: brick at ({b.X},{b.Z}) course {b.Layer} {b.Tone} {(b.Rotated ? "rot" : "")} on {below}]");
                listed++;
            }
            return sb.ToString();
        }
    }
}
