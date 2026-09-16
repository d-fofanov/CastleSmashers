using System.Collections.Generic;
using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Sleeping islands: resting islands freeze and cost no pairs, a touch wakes a whole island in the same step, an
    /// island sleeps only when all of it rests, gameplay changes wake, debris leaves its island, and it is all deterministic.</summary>
    public class SleepTests
    {
        static AvbdGpuWorld SleepWorld(int bodies = 4096)
        {
            var world = GpuTestUtil.NewWorld(bodies);
            world.Params.Sleep = true;
            return world;
        }

        static int Ground(AvbdGpuWorld world, float friction = 0.5f) =>
            world.AddBody(new float3(100, 1, 100), 0f, friction, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);

        static int DynamicCount(AvbdGpuWorld world)
        {
            int n = 0;
            for (int i = 0; i < world.BodyCount; i++) if (!world.GetBodyDef(i).IsStatic) n++;
            return n;
        }

        /// <summary>Steps until every dynamic body is asleep (checking every <paramref name="every"/> steps); returns the step count.
        /// With <paramref name="trace"/> the progress is logged: bodies asleep, bodies resting for the sleep time, the fastest body.</summary>
        static int StepUntilAsleep(AvbdGpuWorld world, int maxSteps, int every = 10, bool trace = false)
        {
            int dynamic = DynamicCount(world);
            int sleepSteps = (int)math.ceil(world.Params.SleepTime / (world.Params.Dt / math.max(1, world.Params.Substeps)));
            for (int step = 0; step < maxSteps; step += every)
            {
                for (int i = 0; i < every; i++) world.Step();
                int sleeping = world.GetStatsSync().Sleeping;
                if (trace && (step + every) % 60 == 0)
                {
                    var words = world.GetSleepSync();
                    int resting = 0;
                    for (int i = 0; i < world.BodyCount; i++) if (!world.GetBodyDef(i).IsStatic && GpuBodySleep.RestSteps(words[i]) >= sleepSteps) resting++;
                    Debug.Log($"step {step + every}: asleep {sleeping}, resting {resting} of {dynamic}, max speed {GpuTestUtil.MaxSpeed(world):E2} m/s");
                }
                if (sleeping == dynamic) return step + every;
            }
            Assert.Fail($"not every body fell asleep within {maxSteps} steps ({world.GetStatsSync().Sleeping} of {dynamic})");
            return maxSteps;
        }

        static bool Asleep(uint[] words, int body) => GpuBodySleep.IsAsleep(words[body]);

        [Test]
        public void SettledPyramidSleepsAndFreezes()
        {
            using var world = SleepWorld();
            world.BuildScene(AvbdScenes.Pyramid);
            int steps = StepUntilAsleep(world, 1200, trace: true);
            var stats = world.GetStatsSync();
            Debug.Log($"pyramid: asleep after {steps} steps; {stats.Manifolds} manifolds, {stats.CarriedManifolds} carried, {stats.Pairs} pairs");
            Assert.AreEqual(136, stats.Sleeping, "every box of the pyramid sleeps");
            Assert.AreEqual(0, stats.Pairs, "sleeping bodies pair with nothing");
            Assert.Greater(stats.Manifolds, 100, "the manifolds of the sleeping pyramid are kept");
            Assert.AreEqual(stats.Manifolds, stats.CarriedManifolds, "and every one of them is carried over");

            var labels = world.GetLabelsSync();
            var islands = new HashSet<uint>();
            for (int i = 1; i < world.BodyCount; i++) islands.Add(labels[i]);
            Assert.AreEqual(1, islands.Count, "the pyramid is one island");

            world.GetPosesSync(out var pos0, out var rot0);
            for (int i = 0; i < 60; i++) world.Step();
            world.GetPosesSync(out var pos1, out var rot1);
            world.GetVelocitiesSync(out var vel, out var ang);
            for (int i = 0; i < world.BodyCount; i++)
            {
                Assert.IsTrue(math.all(pos0[i] == pos1[i]) && math.all(rot0[i] == rot1[i]), $"body {i} moved while asleep");
                Assert.IsTrue(math.all(vel[i] == 0f) && math.all(ang[i] == 0f), $"body {i} has a velocity while asleep");
            }
            Assert.AreEqual(0, world.GetStatsSync().OverflowFlags);
        }

        /// <summary>The GPU island labels of the sleeping Outpost (snapped: joints connect the bricks, side gaps keep them out of contact)
        /// cover the connected components of the carried manifolds and joints computed on the CPU: every component lies within one
        /// label. Labels only merge until a relabel, and sleeping bodies keep theirs, so an island may still be the union of
        /// components that touched while settling; the count is logged.</summary>
        [Test]
        public void IslandLabelsCoverTheConnectedComponents()
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("no compute");
            var config = AvbdGpuConfig.ForBodies(8192);
            config.MaxJoints = 16384; config.MaxLinks = 40960;   // four snap joints per brick overlap
            using var world = new AvbdGpuWorld(config);
            world.AddBody(new float3(400, 1, 400), 0f, 0.6f, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);
            var plan = CastlePlan.Presets[0];
            var layout = BrickCastle.Generate(plan);
            const float scale = 5f;
            float2 c = BrickCastle.Center(plan) * Brick.Pitch * scale;
            float volume = Brick.Width * Brick.BodyHeight * Brick.Length * scale * scale * scale;
            var spec = new BrickSpec { Scale = scale, Density = 0.25f / volume, Friction = 0.6f, Margin = AvbdGpuConstants.CollisionMargin, Origin = new float3(-c.x, 0, -c.y) };
            int first = BrickCastle.Build(world, layout, spec);
            BrickCastle.AddSnapJoints(world, layout, first, spec, 300f, 50f);
            world.Params.Substeps = 3;
            StepUntilAsleep(world, 900);

            // union-find over the constraints the GPU sees
            int n = world.BodyCount;
            var parent = new int[n];
            for (int i = 0; i < n; i++) parent[i] = i;
            int Find(int i) { while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; }
            void Union(int a, int b) { a = Find(a); b = Find(b); if (a != b) parent[math.max(a, b)] = math.min(a, b); }
            bool Dynamic(int i) => i >= 0 && !world.GetBodyDef(i).IsStatic;
            var manifolds = world.GetManifoldsSync(out _);
            var stats = world.GetStatsSync();
            for (int m = 0; m < stats.Manifolds; m++)
                if (Dynamic((int)manifolds[m].BodyA) && Dynamic((int)manifolds[m].BodyB)) Union((int)manifolds[m].BodyA, (int)manifolds[m].BodyB);
            var states = world.GetJointStatesSync();
            for (int j = 0; j < world.JointCount; j++)
            {
                var d = world.GetJointDef(j);
                if (states[j].Broken == 0 && Dynamic(d.BodyA) && Dynamic(d.BodyB)) Union(d.BodyA, d.BodyB);
            }
            var labels = world.GetLabelsSync();
            var islands = new HashSet<uint>();
            var components = new HashSet<int>();
            for (int i = 0; i < n; i++)
            {
                if (!Dynamic(i)) continue;
                islands.Add(labels[i]);
                components.Add(Find(i));
                Assert.AreEqual(labels[Find(i)], labels[i], $"brick {i} and the root of its component carry different labels");
            }
            Debug.Log($"outpost: {islands.Count} islands on the GPU, {components.Count} connected components on the CPU");
            Assert.Greater(islands.Count, 1, "the castle is several islands");
            Assert.LessOrEqual(islands.Count, components.Count, "islands are unions of components");
        }

        [Test]
        public void TwoStacksAreTwoIslands()
        {
            using var world = SleepWorld();
            Ground(world);
            for (int s = 0; s < 2; s++)
                for (int i = 0; i < 5; i++)
                    world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(s * 10f, 0.5f + i, 0), quaternion.identity, float3.zero);
            StepUntilAsleep(world, 600);
            var labels = world.GetLabelsSync();
            for (int i = 1; i <= 5; i++) Assert.AreEqual(labels[1], labels[i], $"box {i} is in the first stack's island");
            for (int i = 6; i <= 10; i++) Assert.AreEqual(labels[6], labels[i], $"box {i} is in the second stack's island");
            Assert.AreNotEqual(labels[1], labels[6], "the stacks are separate islands");
        }

        [Test]
        public void ATouchWakesTheWholeIslandInTheSameStep()
        {
            using var world = SleepWorld();
            Ground(world, 0f);   // ice: the shot slides in at full speed
            for (int s = 0; s < 2; s++)
                for (int i = 0; i < 6; i++)
                    world.AddBody(new float3(1, 1, 1), 1f, 0f, new float3(s * 10f, 0.5f + i, 0), quaternion.identity, float3.zero);
            StepUntilAsleep(world, 600);
            int shot = world.AddBody(new float3(1, 1, 1), 1f, 0f, new float3(-6f, 0.5f, 0), quaternion.identity, new float3(12f, 0, 0));

            int impactStep = -1;
            for (int step = 0; step < 120 && impactStep < 0; step++)
            {
                world.Step();
                var words = world.GetSleepSync();
                bool anyAwake = false;
                for (int i = 1; i <= 6; i++) anyAwake |= !Asleep(words, i);
                if (!anyAwake) continue;
                impactStep = step;
                for (int i = 1; i <= 6; i++) Assert.IsFalse(Asleep(words, i), $"box {i} of the hit stack is awake in the step of the impact");
                for (int i = 7; i <= 12; i++) Assert.IsTrue(Asleep(words, i), $"box {i} of the other stack still sleeps");
                world.GetVelocitiesSync(out var vel, out _);
                Assert.Greater(vel[1].x, 0.5f, "the hit box took momentum in the impact step (it was solved, not treated as static)");
                Assert.Greater(world.GetStatsSync().Woken, 0, "the stats count the woken bodies");
            }
            Assert.GreaterOrEqual(impactStep, 0, "the shot reached the stack");
            _ = shot;
        }

        [Test]
        public void ABodyNextToAMovingOneStaysAwake()
        {
            using var world = SleepWorld();
            Ground(world);
            int a = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 0.5f, 0), quaternion.identity, float3.zero);
            // b slides along z past a, overlapping it by a millimetre, driven by a motor stronger than its friction
            uint driven = GpuBodyDef.FlagDriven | GpuBodyDef.FlagLockRotation;
            int b = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0.999f, 0.5f, -2f), quaternion.identity, float3.zero, driven,
                GpuBodyDrive.Velocity(new float3(0, 0, 0.5f), 20f, new float3(0, 0, 1)));
            for (int i = 0; i < 240; i++) world.Step();
            var words = world.GetSleepSync();
            Assert.IsFalse(Asleep(words, a), "a rests but touches the moving b: it stays awake");
            Assert.IsFalse(Asleep(words, b), "b is moving");
            world.GetPosesSync(out var pos, out _);
            Assert.Greater(pos[b].z, -1.5f, "b moved");
            Assert.Less(math.abs(pos[a].x) + math.abs(pos[a].z), 0.05f, "a did not move");

            world.SetBodyDrive(b, default);   // b stops on its friction and both rest
            StepUntilAsleep(world, 300);
            words = world.GetSleepSync();
            Assert.IsTrue(Asleep(words, a) && Asleep(words, b), "both sleep once the neighbourhood rests");
        }

        [Test]
        public void RetiringASupportWakesWhatRestsOnIt()
        {
            using var world = SleepWorld(1024);
            Ground(world);
            var pool = new BodyPool(world, 4);
            int support = pool.Spawn(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 0.5f, 0), quaternion.identity, float3.zero, 0u);
            int top = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 1.5f, 0), quaternion.identity, float3.zero);
            StepUntilAsleep(world, 600);
            pool.Retire(support);
            for (int i = 0; i < 60; i++) world.Step();
            world.GetPosesSync(out var pos, out _);
            Assert.Less(pos[top].y, 0.7f, "the box above fell once its support was retired");
        }

        [Test]
        public void GameplayChangesWake()
        {
            using var world = SleepWorld();
            Ground(world);
            uint unitFlags = GpuBodyDef.FlagDriven | GpuBodyDef.FlagLockRotation;
            int unit = world.AddBody(new float3(1, 2, 1), 1f, 0.5f, new float3(0, 1f, 0), quaternion.identity, float3.zero, unitFlags,
                GpuBodyDrive.Velocity(float3.zero, 20f, new float3(1, 0, 1)));
            int hanger = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(10, 0.5f, 0), quaternion.identity, float3.zero);
            int flagged = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(20, 0.5f, 0), quaternion.identity, float3.zero);
            int landing = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(30, 0.5f, 0), quaternion.identity, float3.zero);
            StepUntilAsleep(world, 600);
            world.GetPosesSync(out var pos0, out _);

            // an identical drive does not wake, a new target does
            world.SetBodyDrive(unit, GpuBodyDrive.Velocity(float3.zero, 20f, new float3(1, 0, 1)));
            world.Step();
            Assert.IsTrue(Asleep(world.GetSleepSync(), unit), "re-setting the same drive leaves the unit asleep");
            world.SetBodyDrive(unit, GpuBodyDrive.Velocity(new float3(1f, 0, 0), 20f, new float3(1, 0, 1)));
            for (int i = 0; i < 60; i++) world.Step();
            world.GetPosesSync(out var pos1, out _);
            Assert.Greater(pos1[unit].x - pos0[unit].x, 0.3f, "the unit walks off on its new target");

            // a world joint pulls a sleeping box up
            world.AddJointIndexed(-1, hanger, new float3(10, 4f, 0), float3.zero, 5000f, 0f);
            for (int i = 0; i < 90; i++) world.Step();
            world.GetPosesSync(out pos1, out _);
            Assert.Greater(pos1[hanger].y, 1.5f, "the sleeping box rose on the joint");

            // a flag change wakes for a step
            world.SetBodyFlags(flagged, GpuBodyDef.FlagReportEvents);
            world.Step();
            Assert.IsFalse(Asleep(world.GetSleepSync(), flagged), "a flag change wakes the body");

            // a box dropped onto a sleeping one wakes it through the new contact
            int dropped = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(30, 3f, 0), quaternion.identity, float3.zero);
            bool woke = false;
            for (int i = 0; i < 90 && !woke; i++)
            {
                world.Step();
                woke = !Asleep(world.GetSleepSync(), landing);
            }
            Assert.IsTrue(woke, "the landing box wakes the box it lands on");
            world.GetPosesSync(out pos1, out _);
            Assert.Greater(pos1[dropped].y, 1.2f, "and rests on it");
            Assert.AreEqual(0, world.GetStatsSync().OverflowFlags);
        }

        [Test]
        public void DebrisLeavesTheIslandAfterARelabel()
        {
            using var world = SleepWorld();
            Ground(world);
            int a = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 0.5f, 0), quaternion.identity, float3.zero);
            uint driven = GpuBodyDef.FlagDriven | GpuBodyDef.FlagLockRotation;
            int b = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0.999f, 0.5f, 0), quaternion.identity, float3.zero, driven);
            StepUntilAsleep(world, 600);
            Assert.AreEqual(world.GetLabelsSync()[a], world.GetLabelsSync()[b], "touching boxes share an island");

            // b drives away and stops 5 m further; after a relabel the two are separate islands
            world.SetBodyDrive(b, GpuBodyDrive.Velocity(new float3(2f, 0, 0), 20f, new float3(1, 0, 1)));
            for (int i = 0; i < 180; i++) world.Step();
            world.SetBodyDrive(b, default);
            StepUntilAsleep(world, 600);
            for (int i = 0; i < world.Params.SleepRelabelSteps + 8; i++) world.Step();
            var labels = world.GetLabelsSync();
            Assert.AreNotEqual(labels[a], labels[b], "the box that left is its own island now");

            // touching b leaves a asleep
            world.GetPosesSync(out var pos, out _);
            world.AddBody(new float3(0.5f, 0.5f, 0.5f), 1f, 0.5f, new float3(pos[b].x, 3f, 0), quaternion.identity, float3.zero);
            for (int i = 0; i < 60; i++)
            {
                world.Step();
                Assert.IsTrue(Asleep(world.GetSleepSync(), a), "a stays asleep while b is hit");
            }
        }

        /// <summary>A box shot into the settled pyramid, once asleep and once kept awake: the island wakes in the impact step and the
        /// outcome matches the awake run. At 10 iterations the two differ more, since the woken bodies are coloured afresh and the
        /// Gauss-Seidel order matters for an impact that is far from converged (the same is true of the reference comparison).</summary>
        [TestCase(10)]
        [TestCase(40)]
        public void SleepingMatchesAwakeDynamicsOnImpact(int iterations)
        {
            using var asleep = SleepWorld();
            using var awake = SleepWorld();
            awake.Params.Sleep = false;
            asleep.Params.Iterations = awake.Params.Iterations = iterations;
            foreach (var w in new[] { asleep, awake }) w.BuildScene(AvbdScenes.Pyramid);
            int settle = StepUntilAsleep(asleep, 1200);
            for (int i = 0; i < settle; i++) awake.Step();
            Assert.AreEqual(136, asleep.GetStatsSync().Sleeping, "the pyramid sleeps before the shot");
            awake.GetPosesSync(out var rest, out _);

            int hit = 65;   // the box the shot strikes
            float vHitA = 0f, vHitB = 0f;
            foreach (var w in new[] { asleep, awake })
            {
                w.AddBody(new float3(1, 1, 1), 2f, 0.5f, new float3(0, 3f, -6f), quaternion.identity, new float3(0, 0, 18f));
                for (int i = 0; i < 150; i++)
                {
                    w.Step();
                    if (i == 18)
                    {
                        w.GetVelocitiesSync(out var v, out _);
                        if (w == asleep) vHitA = math.length(v[hit].xyz); else vHitB = math.length(v[hit].xyz);
                        if (w == asleep) Assert.AreEqual(0, w.GetStatsSync().Sleeping, "the whole pyramid woke in the impact step");
                    }
                }
            }
            asleep.GetPosesSync(out var pa, out _);
            awake.GetPosesSync(out var pb, out _);
            float maxErr = 0f, sumErr = 0f; int displacedA = 0, displacedB = 0;
            for (int i = 1; i < 137; i++)
            {
                float e = math.length(pa[i].xyz - pb[i].xyz);
                maxErr = math.max(maxErr, e); sumErr += e;
                if (math.length(pa[i].xyz - rest[i].xyz) > 0.5f) displacedA++;
                if (math.length(pb[i].xyz - rest[i].xyz) > 0.5f) displacedB++;
            }
            Debug.Log($"impact on the sleeping pyramid vs awake, {iterations} iterations: hit box {vHitA:F2} vs {vHitB:F2} m/s in the impact step, " +
                $"max error {maxErr:F3} m, mean {sumErr / 136:F4} m, displaced {displacedA} vs {displacedB}");
            Assert.Greater(vHitA, 0.5f * vHitB, "the hit box took its momentum in the impact step");
            Assert.Greater(displacedA, 0); Assert.Greater(displacedB, 0);
            Assert.LessOrEqual(math.abs(displacedA - displacedB), math.max(3, math.max(displacedA, displacedB) / 2), "about as many boxes displaced");
            Assert.Less(sumErr / 136, iterations >= 40 ? 0.1f : 0.25f, "the piles ended up close");
        }

        [Test]
        public void SleepingIsBitwiseDeterministic()
        {
            using var a = SleepWorld();
            using var b = SleepWorld();
            foreach (var w in new[] { a, b })
            {
                w.BuildScene(AvbdScenes.Pyramid);
                for (int i = 0; i < 700; i++) w.Step();
                Assert.AreEqual(136, w.GetStatsSync().Sleeping, "the pyramid sleeps before the shot");
                w.AddBody(new float3(1, 1, 1), 2f, 0.5f, new float3(0, 3f, -6f), quaternion.identity, new float3(0, 0, 18f));
                for (int i = 0; i < 300; i++) w.Step();
            }
            a.GetPosesSync(out var pa, out var ra);
            b.GetPosesSync(out var pb, out var rb);
            var sa = a.GetSleepSync(); var sb = b.GetSleepSync();
            for (int i = 0; i < a.BodyCount; i++)
            {
                Assert.IsTrue(math.all(pa[i] == pb[i]), $"body {i} position differs: {pa[i]} vs {pb[i]}");
                Assert.IsTrue(math.all(ra[i] == rb[i]), $"body {i} rotation differs");
                Assert.AreEqual(sa[i], sb[i], $"body {i} sleep word differs");
            }
            Assert.Greater(a.GetStatsSync().Sleeping, 0, "part of the pyramid sleeps again after the impact");
        }

        /// <summary>A chain of boxes hanging from a static one on hard ball-socket joints, and two boxes snapped together on the
        /// ground: the joints sleep with their bodies (state frozen) and a touch on the chain wakes its whole island.</summary>
        [Test]
        public void JointedBodiesSleepWithTheirJointsFrozen()
        {
            using var world = SleepWorld();
            Ground(world);
            int anchor = world.AddBody(new float3(1, 1, 1), 0f, 0.5f, new float3(0, 10f, 0), quaternion.identity, float3.zero);
            int prev = anchor;
            var chain = new List<int>();
            for (int i = 1; i <= 5; i++)
            {
                int curr = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 10f - 1.1f * i, 0), quaternion.identity, float3.zero);
                world.AddJointIndexed(prev, curr, new float3(0, -0.55f, 0), new float3(0, 0.55f, 0));
                chain.Add(curr);
                prev = curr;
            }
            int lower = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(10, 0.5f, 0), quaternion.identity, float3.zero);
            int upper = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(10, 1.5f, 0), quaternion.identity, float3.zero);
            world.AddJointIndexed(lower, upper, new float3(0, 0.5f, 0), new float3(0, -0.5f, 0));

            int steps = StepUntilAsleep(world, 900);
            Debug.Log($"jointed: asleep after {steps} steps");
            var labels = world.GetLabelsSync();
            foreach (int c in chain) Assert.AreEqual(labels[chain[0]], labels[c], "the chain is one island through its joints");
            Assert.AreEqual(labels[lower], labels[upper], "the snapped pair is one island");
            Assert.AreNotEqual(labels[lower], labels[chain[0]], "and a different one from the chain");

            var states0 = world.GetJointStatesSync();
            world.GetPosesSync(out var pos0, out _);
            for (int i = 0; i < 60; i++) world.Step();
            var states1 = world.GetJointStatesSync();
            world.GetPosesSync(out var pos1, out _);
            for (int j = 0; j < world.JointCount; j++)
                Assert.IsTrue(math.all(states0[j].LambdaLin == states1[j].LambdaLin) && math.all(states0[j].PenaltyLin == states1[j].PenaltyLin), $"joint {j} changed while asleep");
            for (int i = 0; i < world.BodyCount; i++) Assert.IsTrue(math.all(pos0[i] == pos1[i]), $"body {i} moved while asleep");

            // a box thrown at the bottom of the chain wakes the whole chain but not the snapped pair
            world.AddBody(new float3(0.5f, 0.5f, 0.5f), 1f, 0.5f, new float3(-4f, 10f - 1.1f * 5, 0), quaternion.identity, new float3(10f, 3f, 0));
            bool woke = false;
            for (int i = 0; i < 90 && !woke; i++)
            {
                world.Step();
                var words = world.GetSleepSync();
                woke = !Asleep(words, chain[4]);
                if (woke)
                {
                    foreach (int c in chain) Assert.IsFalse(Asleep(words, c), $"chain box {c} woke with the hit one");
                    Assert.IsTrue(Asleep(words, lower) && Asleep(words, upper), "the snapped pair sleeps on");
                }
            }
            Assert.IsTrue(woke, "the thrown box hit the chain");
        }
    }
}
