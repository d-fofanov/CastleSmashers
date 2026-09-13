using System.Collections.Generic;
using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdRef;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Physical invariants and component checks of the GPU solver that do not need the reference's trajectory.</summary>
    public class InvariantTests
    {
        [Test]
        public void PyramidRestsWithoutPenetration()
        {
            using var world = GpuTestUtil.NewWorld();
            world.BuildScene(AvbdScenes.Pyramid);
            for (int i = 0; i < 300; i++) world.Step();
            Assert.Less(GpuTestUtil.MaxSpeed(world), 0.3f, "pyramid settles");
            world.GetPosesSync(out var pos, out _);
            var top = pos[world.BodyCount - 1];
            Assert.Greater(top.y, 7.0f, "top box still on top");
            for (int i = 1; i < world.BodyCount; i++) Assert.Greater(pos[i].y, 0.25f - 0.05f, $"box {i} sank into the ground");
            var stats = world.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags);
        }

        [Test]
        public void WarmStartPersistsAndCarriesTheWeight()
        {
            using var world = GpuTestUtil.NewWorld();
            world.BuildScene(AvbdScenes.Ground);
            for (int i = 0; i < 180; i++) world.Step();
            var manifolds = world.GetManifoldsSync(out var contacts);
            var stats = world.GetStatsSync();
            Assert.AreEqual(1, stats.Manifolds);
            Assert.AreEqual(4, (int)manifolds[0].NumContacts);
            float normal = 0f;
            for (uint c = 0; c < 4; c++)
            {
                var ct = contacts[manifolds[0].ContactStart + c];
                normal += -ct.Lambda.x;
                Assert.Greater(ct.Penalty.x, 100f, "normal penalty ramped and persisted across steps");
                Assert.IsTrue(ct.Stick, "a resting box sticks");
            }
            Assert.AreEqual(10f, normal, 1f, "contact multipliers carry the weight (1 kg * 10)");
        }

        [Test]
        public void BreakableChainBreaks()
        {
            using var world = GpuTestUtil.NewWorld();
            world.BuildScene(AvbdScenes.Breakable);
            for (int i = 0; i < 240; i++) world.Step();
            var states = world.GetJointStatesSync();
            int broken = 0;
            for (int j = 0; j < world.JointCount; j++) if (states[j].Broken != 0) broken++;
            Assert.Greater(broken, 0, "at least one joint fractured");
            GpuTestUtil.AssertFinite(world);
        }

        [Test]
        public void RuntimeSpawnAndJointRemoval()
        {
            using var world = GpuTestUtil.NewWorld();
            world.BuildScene(AvbdScenes.Ground);
            for (int i = 0; i < 30; i++) world.Step();
            int b = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 8, 0), quaternion.identity, float3.zero);
            int j = world.AddJointIndexed(-1, b, new float3(0, 8, 0), float3.zero, 5000f, 0f);
            for (int i = 0; i < 60; i++) world.Step();
            world.GetPosesSync(out var pos, out _);
            Assert.Greater(pos[b].y, 7.5f, "the spawned box hangs from the world joint");
            world.RemoveJoint(j);
            for (int i = 0; i < 120; i++) world.Step();
            world.GetPosesSync(out pos, out _);
            Assert.Less(pos[b].y, 3f, "after removing the joint the box falls onto the first one");
            int j2 = world.AddJointIndexed(-1, b, new float3(0, 5, 0), float3.zero, 5000f, 0f);
            Assert.AreEqual(j, j2, "the freed joint slot is reused");
            GpuTestUtil.AssertFinite(world);
            Assert.AreEqual(0, world.GetStatsSync().OverflowFlags);
        }

        static void WorldAabb(float3 pos, quaternion rot, float3 size, out float3 mn, out float3 mx)
        {
            float3x3 r = new float3x3(rot);
            float3 half = size * 0.5f;
            // columns of r are the rotated body axes: world extent_i = sum_j |R_ij| half_j
            float3 ext = new float3(math.abs(r.c0.x) * half.x + math.abs(r.c1.x) * half.y + math.abs(r.c2.x) * half.z,
                math.abs(r.c0.y) * half.x + math.abs(r.c1.y) * half.y + math.abs(r.c2.y) * half.z,
                math.abs(r.c0.z) * half.x + math.abs(r.c1.z) * half.y + math.abs(r.c2.z) * half.z) + AvbdGpuConstants.CollisionMargin;
            mn = pos - ext; mx = pos + ext;
        }

        [Test]
        public void BroadphasePairsMatchBruteForce()
        {
            using var world = GpuTestUtil.NewWorld();
            world.Clear();
            var rng = new Unity.Mathematics.Random(99u);
            var sizes = new List<float3>(); var poses = new List<float3>(); var rots = new List<quaternion>();
            // a large static ground (large-body path), a slab (grid, many cells) and random boxes of mixed size
            void Add(float3 size, float density, float3 p, quaternion q) { world.AddBody(size, density, 0.5f, p, q, float3.zero); sizes.Add(size); poses.Add(p); rots.Add(q); }
            Add(new float3(100, 1, 100), 0f, new float3(0, -0.5f, 0), quaternion.identity);
            Add(new float3(12, 0.5f, 3), 1f, new float3(0, 3, 0), quaternion.identity);
            for (int i = 0; i < 600; i++)
            {
                float s = rng.NextFloat(0.3f, 2.5f);
                Add(new float3(s, rng.NextFloat(0.3f, 2.5f), rng.NextFloat(0.3f, 2.5f)), rng.NextFloat() < 0.1f ? 0f : 1f,
                    rng.NextFloat3(new float3(-15, 0, -15), new float3(15, 12, 15)), rng.NextQuaternionRotation());
            }
            world.AddIgnoreCollision(2, 3);
            world.Step();
            var stats = world.GetStatsSync();
            Assert.AreEqual(0, stats.OverflowFlags);
            var pairs = new uint2[math.max(stats.Pairs, 1)];
            if (stats.Pairs > 0) world.Buffers.Pairs.GetData(pairs, 0, 0, stats.Pairs);
            var gpu = new HashSet<(int, int)>();
            for (int i = 0; i < stats.Pairs; i++)
            {
                Assert.Less(pairs[i].x, pairs[i].y, "pairs are ordered");
                Assert.IsTrue(gpu.Add(((int)pairs[i].x, (int)pairs[i].y)), $"duplicate pair {pairs[i]}");
            }
            var cpu = new HashSet<(int, int)>();
            int n = sizes.Count;
            var mins = new float3[n]; var maxs = new float3[n];
            for (int i = 0; i < n; i++) WorldAabb(poses[i], rots[i], sizes[i], out mins[i], out maxs[i]);
            for (int i = 0; i < n; i++)
                for (int j = i + 1; j < n; j++)
                {
                    if (world.GetBodyDef(i).IsStatic && world.GetBodyDef(j).IsStatic) continue;
                    if (i == 2 && j == 3) continue;
                    if (math.all(mins[i] <= maxs[j]) && math.all(mins[j] <= maxs[i])) cpu.Add((i, j));
                }
            Assert.Greater(cpu.Count, 100, "test has a meaningful number of overlaps");
            gpu.SymmetricExceptWith(cpu);
            Assert.AreEqual(0, gpu.Count, $"GPU and brute-force pair sets differ by {gpu.Count} pairs (gpu {stats.Pairs}, cpu {cpu.Count})");
            Debug.Log($"broadphase: {stats.Pairs} pairs match brute force, {stats.LargeBodies} large bodies");
        }

        [Test]
        public void NarrowphaseMatchesReferenceCollide()
        {
            using var world = GpuTestUtil.NewWorld();
            world.Clear();
            var solver = new Solver();
            var rng = new Unity.Mathematics.Random(5u);
            var refBodies = new List<Rigid>();
            const int pairs = 40;
            for (int p = 0; p < pairs; p++)
            {
                float3 origin = new float3(p * 50f, 0, 0);   // pairs far apart from each other
                float3 sizeA = rng.NextFloat3(new float3(0.5f), new float3(3f));
                float3 sizeB = rng.NextFloat3(new float3(0.5f), new float3(3f));
                quaternion qA = p % 3 == 0 ? quaternion.identity : rng.NextQuaternionRotation();
                quaternion qB = p % 3 == 1 ? quaternion.identity : rng.NextQuaternionRotation();
                float3 posB = origin + rng.NextFloat3Direction() * rng.NextFloat(0.2f, 0.5f * (math.cmin(sizeA) + math.cmin(sizeB)));
                world.AddBody(sizeA, 1f, 0.5f, origin, qA, float3.zero);
                world.AddBody(sizeB, 1f, 0.5f, posB, qB, float3.zero);
                var ra = new Rigid(solver, sizeA, 1f, 0.5f, origin) { positionAng = Quat.FromUnity(qA) };
                var rb = new Rigid(solver, sizeB, 1f, 0.5f, posB) { positionAng = Quat.FromUnity(qB) };
                refBodies.Add(ra); refBodies.Add(rb);
            }
            world.Step();
            var manifolds = world.GetManifoldsSync(out var contacts);
            var stats = world.GetStatsSync();
            var byPair = new Dictionary<uint, int>();
            for (int m = 0; m < stats.Manifolds; m++) byPair[manifolds[m].BodyA] = m;

            int compared = 0, faces = 0, edges = 0;
            var refContacts = new Manifold.Contact[Manifold.MaxContacts];
            for (int p = 0; p < pairs; p++)
            {
                int n = RefCollide.Collide(refBodies[2 * p], refBodies[2 * p + 1], refContacts, out Mat3 basis);
                bool gpuHas = byPair.TryGetValue((uint)(2 * p), out int mi);
                Assert.AreEqual(n > 0, gpuHas, $"pair {p}: reference {n} contacts, gpu manifold {(gpuHas ? "present" : "absent")}");
                if (n == 0) continue;
                var m = manifolds[mi];
                Assert.AreEqual(n, (int)m.NumContacts, $"pair {p}: contact count");
                Assert.Less(math.length((float3)basis[0] - m.Normal), 1e-4f, $"pair {p}: normal");
                for (int c = 0; c < n; c++)
                {
                    var g = contacts[m.ContactStart + c];
                    Assert.AreEqual((uint)refContacts[c].feature, g.FeatureKey, $"pair {p} contact {c}: feature key");
                    Assert.Less(math.length(refContacts[c].rA - g.RA), 1e-3f, $"pair {p} contact {c}: rA");
                    Assert.Less(math.length(refContacts[c].rB - g.RB), 1e-3f, $"pair {p} contact {c}: rB");
                }
                if ((refContacts[0].feature >> 24) == RefCollide.AXIS_EDGE) edges++; else faces++;
                compared++;
            }
            Debug.Log($"narrowphase: {compared} colliding pairs compared ({faces} face, {edges} edge manifolds)");
            Assert.Greater(compared, 10);
        }
    }
}
