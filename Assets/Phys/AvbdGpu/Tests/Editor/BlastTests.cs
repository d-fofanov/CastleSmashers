using NUnit.Framework;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Blasts (<see cref="AvbdGpuWorld.Blast"/>): the radial impulse with its linear falloff, the exclusions, the joint
    /// pulverization, waking sleepers and determinism.</summary>
    public class BlastTests
    {
        [Test]
        public void ImpulseFallsOffLinearlyAndSkipsTheExcludedAndStatic()
        {
            using var world = GpuTestUtil.NewWorld(1024);
            world.Params.Gravity = float3.zero;
            int near = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0.5f, 0, 0), quaternion.identity, float3.zero);     // d = 0.5
            int mid = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(2.5f, 0, 0), quaternion.identity, float3.zero);      // d = 2.5
            int heavy = world.AddBody(new float3(1, 1, 1), 2f, 0.5f, new float3(-2.5f, 0, 0), quaternion.identity, float3.zero);   // d = 2.5, 2 kg
            int far = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(5f, 0, 0), quaternion.identity, float3.zero);        // outside
            int excluded = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 0, 2f), quaternion.identity, float3.zero);
            int wall = world.AddBody(new float3(1, 1, 1), 0f, 0.5f, new float3(0, 0, -2f), quaternion.identity, float3.zero);      // static
            world.Blast(float3.zero, 4f, 8f, 0f, 0f, excluded);
            world.Step();
            world.GetVelocitiesSync(out var vel, out _);
            Assert.AreEqual(8f * (1f - 0.5f / 4f), vel[near].x, 1e-3f, "near: J (1 - d/R) / m along +x");
            Assert.AreEqual(0f, math.length(vel[near].yz), 1e-3f, "near: radial");
            Assert.AreEqual(8f * (1f - 2.5f / 4f), vel[mid].x, 1e-3f, "mid");
            Assert.AreEqual(-8f * (1f - 2.5f / 4f) / 2f, vel[heavy].x, 1e-3f, "heavy: half the velocity change, along -x");
            Assert.AreEqual(0f, math.length(vel[far].xyz), 1e-4f, "outside the radius: untouched");
            Assert.AreEqual(0f, math.length(vel[excluded].xyz), 1e-4f, "excluded: untouched");
            Assert.AreEqual(0f, math.length(vel[wall].xyz), 1e-6f, "static: untouched");
            GpuTestUtil.AssertFinite(world);
        }

        [Test]
        public void LiftBiasesTheDirectionUpward()
        {
            using var world = GpuTestUtil.NewWorld(1024);
            world.Params.Gravity = float3.zero;
            int b = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(2f, 0, 0), quaternion.identity, float3.zero);
            world.Blast(float3.zero, 4f, 4f, 1f, 0f);     // lift 1: (1, 0, 0) + (0, 1, 0), normalised
            world.Step();
            world.GetVelocitiesSync(out var vel, out _);
            float expected = 4f * 0.5f / math.sqrt(2f);
            Assert.AreEqual(expected, vel[b].x, 1e-3f);
            Assert.AreEqual(expected, vel[b].y, 1e-3f);
        }

        [Test]
        public void PulverizeBreaksJointsWithinRadiusOnlyAndTheImpulseTossesThePieces()
        {
            using var world = GpuTestUtil.NewWorld(1024);
            var joints = new int[2][];
            var tops = new int[2];
            for (int s = 0; s < 2; s++) joints[s] = Stack(world, s * 6f, out tops[s]);
            for (int i = 0; i < 30; i++) world.Step();
            world.GetPosesSync(out var before, out _);
            // off centre, so the freed pieces are pushed sideways rather than lifted and dropped back onto their base
            world.Blast(new float3(-1f, 1f, 0), 2.5f, 8f, 0.5f, 2f);
            world.Step();
            var states = world.GetJointStatesSync();
            foreach (int j in joints[0]) Assert.AreNotEqual(0u, states[j].Broken, $"stack 0 joint {j} within the pulverize radius breaks");
            foreach (int j in joints[1]) Assert.AreEqual(0u, states[j].Broken, $"stack 1 joint {j} 6 m away holds");
            for (int i = 0; i < 90; i++) world.Step();
            world.GetPosesSync(out var after, out _);
            float flown = math.max(math.distance(after[tops[0]].xyz, before[tops[0]].xyz), math.distance(after[tops[0] - 1].xyz, before[tops[0] - 1].xyz));
            Assert.Greater(flown, 1f, "a piece of the pulverized stack flew off");
            Assert.Less(math.distance(after[tops[1]].xyz, before[tops[1]].xyz), 0.05f, "the other stack stands");
            GpuTestUtil.AssertFinite(world);
        }

        [Test]
        public void RemovedJointsAreLeftAlone()
        {
            using var world = GpuTestUtil.NewWorld(1024);
            var joints = Stack(world, 0f, out _);
            world.Step();
            world.RemoveJoint(joints[1]);
            world.Blast(new float3(0, 1f, 0), 0f, 0f, 0f, 3f);
            world.Step();
            var states = world.GetJointStatesSync();
            Assert.AreNotEqual(0u, states[joints[0]].Broken, "the live joint breaks");
            Assert.AreEqual(0u, states[joints[1]].Broken, "the removed joint keeps its zero state");
        }

        [Test]
        public void BlastWakesSleepersWithinReachAndLeavesTheOthersAsleep()
        {
            using var world = GpuTestUtil.NewWorld(2048);
            world.Params.Sleep = true;
            world.AddBody(new float3(60, 1, 60), 0f, 0.6f, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);
            const int n = 10;
            var bodies = new int[n * n];
            for (int x = 0; x < n; x++)
                for (int z = 0; z < n; z++)
                    bodies[x * n + z] = world.AddBody(new float3(1, 1, 1), 1f, 0.6f, new float3((x - 4.5f) * 1.5f, 0.5f, (z - 4.5f) * 1.5f), quaternion.identity, float3.zero);
            for (int i = 0; i < 150; i++) world.Step();
            var sleep = world.GetSleepSync();
            int asleep = 0, inGrid = 0;
            foreach (int b in bodies) { if (GpuBodySleep.IsAsleep(sleep[b])) asleep++; if (GpuBodySleep.IsInGrid(sleep[b])) inGrid++; }
            Assert.AreEqual(bodies.Length, asleep, "the grid of cubes fell asleep");
            Assert.Greater(inGrid, 0, "the sleepers entered the sleeping grid (their wake goes through the stale count)");
            world.GetPosesSync(out var before, out _);
            const float radius = 4f;
            world.Blast(new float3(0, 0.5f, 0), radius, 6f, 0.3f, 0f);
            world.Step();
            sleep = world.GetSleepSync();
            int woken = 0, expected = 0;
            foreach (int b in bodies)
            {
                bool inside = math.length(before[b].xyz - new float3(0, 0.5f, 0)) < radius;
                if (inside) expected++;
                if (inside) Assert.IsFalse(GpuBodySleep.IsAsleep(sleep[b]), $"body {b} within the blast woke");
                else Assert.IsTrue(GpuBodySleep.IsAsleep(sleep[b]), $"body {b} outside the blast sleeps on");
                if (!GpuBodySleep.IsAsleep(sleep[b])) woken++;
            }
            Assert.Greater(expected, 4, "several cubes lie within the radius");
            Assert.AreEqual(expected, woken);
            for (int i = 0; i < 30; i++) world.Step();
            world.GetPosesSync(out var after, out _);
            float moved = 0f;
            foreach (int b in bodies) moved = math.max(moved, math.distance(after[b].xyz, before[b].xyz));
            Assert.Greater(moved, 0.5f, "the woken cubes moved");
            GpuTestUtil.AssertFinite(world);
        }

        [Test]
        public void BlastIsBitwiseDeterministic()
        {
            float4[] a = Run(), b = Run();
            Assert.AreEqual(a.Length, b.Length);
            for (int i = 0; i < a.Length; i++) Assert.IsTrue(math.all(a[i] == b[i]), $"body {i}: {a[i]} vs {b[i]}");
        }

        static float4[] Run()
        {
            using var world = GpuTestUtil.NewWorld(1024);
            for (int s = 0; s < 3; s++) Stack(world, s * 4f, out _);
            for (int i = 0; i < 120; i++)
            {
                if (i == 10) world.Blast(new float3(2f, 1f, 0), 6f, 4f, 0.5f, 3f);
                world.Step();
            }
            world.GetPosesSync(out var pos, out _);
            return pos;
        }

        /// <summary>A static base cube at (x, 0.5, 0) carrying two 1 kg cubes on unbreakable snap joints; returns the joint
        /// indices, <paramref name="top"/> the top cube.</summary>
        static int[] Stack(AvbdGpuWorld world, float x, out int top)
        {
            int a = world.AddBody(new float3(1, 1, 1), 0f, 0.6f, new float3(x, 0.5f, 0), quaternion.identity, float3.zero);
            int b = world.AddBody(new float3(1, 1, 1), 1f, 0.6f, new float3(x, 1.5f, 0), quaternion.identity, float3.zero);
            int c = world.AddBody(new float3(1, 1, 1), 1f, 0.6f, new float3(x, 2.5f, 0), quaternion.identity, float3.zero);
            top = c;
            float inf = float.PositiveInfinity;
            return new[]
            {
                world.AddJointIndexed(a, b, new float3(0, 0.5f, 0), new float3(0, -0.5f, 0), inf, inf, inf, inf, inf, inf, 2),
                world.AddJointIndexed(b, c, new float3(0, 0.5f, 0), new float3(0, -0.5f, 0), inf, inf, inf, inf, inf, inf, 2),
            };
        }
    }
}
