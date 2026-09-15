using System.Collections;
using NUnit.Framework;
using Phys.AvbdGpu;
using Phys.AvbdGpu.Scenes;
using Phys.Demo;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.TestTools;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>PlayMode: the preview demo loads the castles of Resources/Castles and draws every piece with its model; the Outpost
    /// exported from the castle planner stands dry-stacked and snapped and loses bricks to a cannonball; the Emerald Crown Citadel
    /// (an agent's design re-bonded by Tools/rebond_castle.py) stands dry-stacked and holds snapped.</summary>
    public class PreviewSmokeTests
    {
        GameObject m_Root;
        PreviewDemo m_Demo;
        Moves m_Moves;

        [UnitySetUp]
        public IEnumerator SetUp()
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("no compute");
            m_Root = new GameObject("PreviewSmoke");
            var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
            camGo.transform.SetParent(m_Root.transform);
            camGo.AddComponent<Camera>();
            camGo.AddComponent<DemoCamera>();
            var light = new GameObject("Light").AddComponent<Light>();
            light.type = LightType.Directional;
            light.shadows = LightShadows.Soft;
            light.transform.SetParent(m_Root.transform);
            light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            m_Demo = m_Root.AddComponent<PreviewDemo>();
            m_Demo.StartScene = 0;
            m_Demo.MaxBodies = 40960;   // the script's defaults otherwise: 0.25 kg bricks, snaps of 300 / 50 N, like the castle smoke tests
            yield return null;
        }

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            if (m_Root != null) Object.Destroy(m_Root);
            yield return null;
        }

        void LoadCastle(string name)
        {
            int index = m_Demo.IndexOf(name);
            Assert.GreaterOrEqual(index, 0, $"{name}.json in Resources/Castles");
            m_Demo.Load(index);   // always: the snap setting may have changed since the demo built the scene
            Assert.IsNull(m_Demo.Error, m_Demo.Error);
            Assert.AreEqual(m_Demo.Bodies.Groups.Count, m_Demo.Renderer.MeshRanges.Count, "every group of pieces is drawn with its model");
            int drawn = 0;
            foreach (var r in m_Demo.Renderer.MeshRanges) { Assert.IsNotNull(r.Mesh); drawn += r.Count; }
            Assert.AreEqual(m_Demo.PartCount, drawn, "every piece is drawn with a model");
        }

        float BrickHeight => 1.2f * PieceCatalog.GridToUnity * m_Demo.Spec.Scale;

        [UnityTest]
        public IEnumerator OutpostStandsDryStacked()
        {
            LoadCastle("outpost");
            Assert.AreEqual(1001, m_Demo.PartCount);
            Assert.IsTrue(m_Demo.Diagnostics.Clean);
            yield return Run(180);
            Assert.Less(m_Moves.max, 0.25f * BrickHeight, "no brick moved more than a quarter of its height: the castle stands");
        }

        [UnityTest]
        public IEnumerator OutpostHoldsSnapped()
        {
            m_Demo.Snap = true;
            LoadCastle("outpost");
            Assert.Greater(m_Demo.SnapJoints, m_Demo.PartCount, "at least one snap joint per brick");
            Assert.AreEqual(m_Demo.SnapJoints, m_Demo.World.JointCount);
            yield return Run(120);
            Assert.Less(m_Moves.max, 0.25f * BrickHeight, "the snapped castle stands");
            Assert.AreEqual(0, BrokenJoints(), "no snap broke under the castle's own weight: " + BrokenSummary());
        }

        /// <summary>The pieces whose joints broke (diagnostics).</summary>
        string BrokenSummary()
        {
            var states = m_Demo.World.GetJointStatesSync();
            var sb = new System.Text.StringBuilder();
            int listed = 0;
            for (int j = 0; j < m_Demo.World.JointCount && listed < 40; j++)
            {
                if (states[j].Broken == 0) continue;
                var d = m_Demo.World.GetJointDef(j);
                var b = m_Demo.Assembly.Parts[m_Demo.Bodies.PartOfBody[d.BodyB - m_Demo.Bodies.First]];
                string below = d.BodyA < 0 ? "world" : m_Demo.Assembly.Parts[m_Demo.Bodies.PartOfBody[d.BodyA - m_Demo.Bodies.First]].Id;
                sb.Append($" [joint {j}: '{b.Id}' {PieceCatalog.Pieces[b.Piece].Id} at {b.Position} on {below}; lambda {states[j].LambdaLin}]");
                listed++;
            }
            return sb.ToString();
        }

        [UnityTest]
        public IEnumerator CannonballKnocksBricksOffTheOutpost()
        {
            LoadCastle("outpost");
            for (int f = 0; f < 10; f++) yield return null;
            m_Demo.World.GetPosesSync(out var start, out _);
            // a heavy cube fired at the front (-z) wall from outside
            float s = m_Demo.Spec.Scale;
            float3 extent = m_Demo.Assembly.Extent * PieceCatalog.GridToUnity * s;
            float3 target = new float3(0.15f * extent.x, 3f * BrickHeight, -0.5f * extent.z + 0.3f * s);
            float3 from = target + new float3(0f, 0.2f * s, -1.5f * s);
            float cube = m_Demo.ShotCube * s;
            int ball = m_Demo.World.AddBody(new float3(cube), m_Demo.ShotMass / (cube * cube * cube), 0.5f, from, quaternion.identity, math.normalize(target - from) * m_Demo.ShotVelocity * math.sqrt(s));
            for (int f = 0; f < 120; f++) yield return null;
            m_Demo.World.GetPosesSync(out var pos, out _);
            int moved = 0;
            for (int i = m_Demo.Bodies.First; i < m_Demo.Bodies.First + m_Demo.Bodies.Count; i++)
                if (math.distance(pos[i].xyz, start[i].xyz) > BrickHeight) moved++;
            Debug.Log($"cannonball: {moved} bricks displaced by more than a brick height, ball at {pos[ball].xyz}");
            Assert.Greater(moved, 0, "the cannonball displaced some bricks");
            Assert.Less(moved, m_Demo.PartCount / 2, "the castle did not collapse entirely");
        }

        [UnityTest]
        public IEnumerator CitadelStandsDryStacked()
        {
            LoadCastle("emerald_crown_citadel");
            Assert.Greater(m_Demo.PartCount, 2000);
            var d = m_Demo.Diagnostics;
            Assert.AreEqual(0, d.Intersections);
            Assert.AreEqual(7, d.Floating, "the portcullis teeth and the trees' outer foliage: " + d.Sample);
            yield return Run(180);
            var stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags);
            Debug.Log($"dry citadel: max {m_Moves.max * 1000f:F0} mm, mean {m_Moves.mean * 1000f:F1} mm, still {m_Moves.still}/{m_Demo.PartCount}");
            Assert.Greater(m_Moves.still, m_Demo.PartCount * 9 / 10, "nine of ten pieces stay within a quarter of a brick height");
            Assert.Greater(m_Moves.groundStill, 0.95f, "the pieces on the ground stay put");
        }

        [UnityTest]
        public IEnumerator CitadelHoldsSnapped()
        {
            m_Demo.Snap = true;
            LoadCastle("emerald_crown_citadel");
            Assert.Greater(m_Demo.SnapJoints, 8000);
            yield return Run(120);
            Debug.Log($"snapped citadel: max {m_Moves.max * 1000f:F0} mm, mean {m_Moves.mean * 1000f:F1} mm, still {m_Moves.still}/{m_Demo.PartCount}, broken {BrokenJoints()}: {BrokenSummary()}");
            Assert.Greater(m_Moves.still, m_Demo.PartCount - 8, "everything but the hanging teeth stays within a quarter of a brick height");
            Assert.AreEqual(0, BrokenJoints(), "no snap broke under the castle's own weight: " + BrokenSummary());
        }

        /// <summary>The Outpost document on a ridge: the plateau under it, the pieces resting on the terrain, the castle standing.</summary>
        [UnityTest]
        public IEnumerator OutpostStandsOnARidge()
        {
            m_Demo.TerrainParams.Preset = TerrainPreset.Ridge;
            LoadCastle("outpost");
            var field = m_Demo.World.Terrain;
            Assert.IsNotNull(field, "the demo set a terrain");
            Assert.IsTrue(m_Demo.TerrainView.Visible);
            Assert.AreEqual(m_Demo.Plateau, m_Demo.Spec.Origin.y, "the castle stands on the plateau");
            Assert.Greater(m_Demo.Plateau, 1f, "the ridge runs under the castle");
            float3 extent = m_Demo.Assembly.Extent * PieceCatalog.GridToUnity * m_Demo.Spec.Scale;
            Assert.AreEqual(m_Demo.Plateau, field.Height(new float2(0.45f * extent.x, 0.45f * extent.z)), 1e-4f, "level across the footprint");
            Assert.Greater(m_Demo.World.GetStatsSync().TerrainManifolds, 100, "the ground course rests on the terrain");
            yield return Run(180);
            Assert.Less(m_Moves.max, 0.25f * BrickHeight, "no brick moved more than a quarter of its height: the castle stands on the ridge");
            Assert.Greater(m_Moves.groundStill, 0.95f, "the pieces on the plateau stay put");
        }

        struct Moves { public float max, mean; public int still; public float groundStill; }

        /// <summary>Runs the demo for some frames, logging how far the pieces move; the result is in m_Moves.</summary>
        IEnumerator Run(int frames)
        {
            var result = new Moves();
            m_Demo.World.GetPosesSync(out var start, out _);
            int worst = -1;
            var log = new System.Text.StringBuilder();
            int first = m_Demo.Bodies.First, count = m_Demo.Bodies.Count;
            var byPrefix = new System.Collections.Generic.Dictionary<string, (float sum, int n)>();
            for (int f = 1; f <= frames; f++)
            {
                yield return null;
                if (f % 30 != 0) continue;
                m_Demo.World.GetPosesSync(out var pos, out var rot);
                result = new Moves();
                int ground = 0, groundStill = 0;
                byPrefix.Clear();
                for (int i = first; i < first + count; i++)
                {
                    Assert.IsTrue(math.all(math.isfinite(pos[i])) && math.all(math.isfinite(rot[i])), $"piece {i} pose");
                    float dist = math.distance(pos[i].xyz, start[i].xyz);
                    result.mean += dist;
                    if (dist > result.max) { result.max = dist; worst = i; }
                    if (dist < 0.25f * BrickHeight) result.still++;
                    var part = m_Demo.Assembly.Parts[m_Demo.Bodies.PartOfBody[i - first]];
                    if (part.Position.y < 1e-3f) { ground++; if (dist < 0.05f * m_Demo.Spec.Scale) groundStill++; }
                    string prefix = part.Id.Split('/')[0];
                    byPrefix.TryGetValue(prefix, out var acc);
                    byPrefix[prefix] = (acc.sum + dist, acc.n + 1);
                }
                result.mean /= count;
                result.groundStill = ground > 0 ? groundStill / (float)ground : 1f;
                log.Append($"  frame {f}: max {result.max * 1000f:F0} mm mean {result.mean * 1000f:F1} mm still {result.still}/{count}{(m_Demo.World.JointCount > 0 ? $" broken joints {BrokenJoints()}" : "")}");
            }
            var stats = m_Demo.World.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags, $"capacity overflow {stats.OverflowFlags}");
            var worstPart = m_Demo.Assembly.Parts[m_Demo.Bodies.PartOfBody[worst - first]];
            var movers = new System.Collections.Generic.List<(string prefix, float mean, int n)>();
            foreach (var kv in byPrefix) movers.Add((kv.Key, kv.Value.sum / kv.Value.n, kv.Value.n));
            movers.Sort((a, b) => b.mean.CompareTo(a.mean));
            var top = new System.Text.StringBuilder();
            for (int i = 0; i < movers.Count && i < 6; i++) top.Append($"  {movers[i].prefix}: {movers[i].mean * 1000f:F0} mm over {movers[i].n} pieces");
            Debug.Log($"{m_Demo.Assembly.Name}: {count} pieces, worst '{worstPart.Id}' ({PieceCatalog.Pieces[worstPart.Piece].Id} at {worstPart.Position}), " +
                $"contacts {stats.Contacts}, colours {stats.ColorsUsed}, step {stats.AvgStepMs:F2} ms\n{log}\n  moved most:{top}");
            m_Moves = result;
        }

        int BrokenJoints()
        {
            var states = m_Demo.World.GetJointStatesSync();
            int broken = 0;
            for (int j = 0; j < m_Demo.World.JointCount; j++) if (states[j].Broken != 0) broken++;
            return broken;
        }
    }
}
