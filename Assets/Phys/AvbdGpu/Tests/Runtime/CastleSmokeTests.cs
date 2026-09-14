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
    /// <summary>PlayMode: the castle demo (brick bodies drawn with the brick model) runs, and the dry-stacked castle stands still.</summary>
    public class CastleSmokeTests
    {
        GameObject m_Root;
        CastleDemo m_Demo;

        [UnitySetUp]
        public IEnumerator SetUp()
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("no compute");
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
            m_Demo.MaxBodies = 8192;
#if UNITY_EDITOR
            m_Demo.BrickMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx");
#endif
            yield return null;
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
            Assert.AreEqual(1, m_Demo.Renderer.MeshRanges.Count, "the bricks are drawn with the brick model");
            Assert.AreEqual(m_Demo.BrickCount, m_Demo.Renderer.MeshRanges[0].Count);
            m_Demo.World.GetPosesSync(out var start, out _);
            float maxMove = 0f;
            int worst = -1;
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
