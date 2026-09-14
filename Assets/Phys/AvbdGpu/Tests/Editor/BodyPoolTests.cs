using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Body pools: retired slots take no part in the simulation, spawns into them start from a clean state, and the
    /// narrowphase reports what units and projectiles touch.</summary>
    public class BodyPoolTests
    {
        const uint Unit = GpuBodyDef.FlagReportEvents | (GpuBodyDef.KindUnit << GpuBodyDef.KindShift);
        const uint Projectile = GpuBodyDef.FlagReportEvents | (GpuBodyDef.KindProjectile << GpuBodyDef.KindShift);

        static AvbdGpuWorld WorldWithGround()
        {
            var world = GpuTestUtil.NewWorld(1024);
            world.AddBody(new float3(100, 1, 100), 0f, 0.5f, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);
            return world;
        }

        static HashSet<(int, int)> Pairs(AvbdGpuWorld world)
        {
            var stats = world.GetStatsSync();
            var pairs = new uint2[math.max(stats.Pairs, 1)];
            if (stats.Pairs > 0) world.Buffers.Pairs.GetData(pairs, 0, 0, stats.Pairs);
            var set = new HashSet<(int, int)>();
            for (int i = 0; i < stats.Pairs; i++) set.Add(((int)pairs[i].x, (int)pairs[i].y));
            return set;
        }

        [Test]
        public void RetiredBodiesNeitherCollideNorMove()
        {
            using var world = WorldWithGround();
            var pool = new BodyPool(world, 8);
            Assert.AreEqual(1, pool.Start);
            Assert.AreEqual(9, world.BodyCount);
            var slots = new int[4];
            for (int i = 0; i < 4; i++) slots[i] = pool.Spawn(new float3(1, 1, 1), 1f, 0.5f, new float3(i * 1.2f, 0.5f, 0), quaternion.identity, float3.zero, 0u);
            CollectionAssert.AreEqual(new[] { 1, 2, 3, 4 }, slots, "low slots first");
            for (int i = 0; i < 30; i++) world.Step();
            var pairs = Pairs(world);
            Assert.AreEqual(4, pairs.Count, "every live box pairs with the ground and nothing else (1.2 m apart with a 1 cm margin)");
            Assert.IsFalse(pairs.Contains((0, 5)), "the empty slots pair with nothing");

            // retire two, drop a box onto where one of them was: it falls straight through to the ground
            pool.Retire(slots[1]); pool.Retire(slots[2]);
            Assert.AreEqual(2, pool.Alive);
            world.GetPosesSync(out var before, out _);
            int dropped = world.AddBody(new float3(0.5f, 0.5f, 0.5f), 1f, 0.5f, new float3(1.2f, 3f, 0), quaternion.identity, float3.zero);
            for (int i = 0; i < 90; i++) world.Step();
            pairs = Pairs(world);
            foreach (var (a, b) in pairs) Assert.IsTrue(a != slots[1] && b != slots[1] && a != slots[2] && b != slots[2], $"pair ({a}, {b}) involves a retired body");
            world.GetPosesSync(out var after, out _);
            Assert.AreEqual(before[slots[1]], after[slots[1]], "a retired body does not move");
            Assert.Less(after[dropped].y, 0.3f, "the dropped box reached the ground through the retired box");
            Assert.AreEqual(0, world.GetStatsSync().OverflowFlags);
            GpuTestUtil.AssertFinite(world);
        }

        [Test]
        public void SpawnIntoARetiredSlotStartsClean()
        {
            using var world = WorldWithGround();
            var pool = new BodyPool(world, 4);
            int a = pool.Spawn(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 3f, 0), quaternion.identity, float3.zero, Projectile);
            for (int i = 0; i < 120; i++) world.Step();
            world.GetPosesSync(out var pos, out _);
            Assert.Less(pos[a].y, 0.6f, "the first box landed");
            Assert.IsTrue(GpuBodyEvents.Touched(world.GetEventsSync(a, 1)[0]), "it touched the ground");

            // retire and respawn in the same frame: the slot is still cooling, so the next slot is used
            pool.Retire(a);
            int b = pool.Spawn(new float3(1, 1, 1), 1f, 0.5f, new float3(5, 3f, 0), quaternion.identity, float3.zero, Projectile);
            Assert.AreEqual(a + 1, b, "a slot retired this frame is not reused before the next step");
            world.Step();
            int c = pool.Spawn(new float3(1, 1, 1), 1f, 0.5f, new float3(10, 3f, 0), quaternion.RotateY(0.3f), new float3(0, 0, 1f), Projectile);
            Assert.AreEqual(a, c, "after the step the retired slot is free again");
            world.Step();
            world.GetPosesSync(out pos, out var rot);
            world.GetVelocitiesSync(out var vel, out var ang);
            Assert.Less(math.distance(pos[c].xyz, new float3(10, 3f - 10f / 3600f, 1f / 60f)), 1e-4f, "one step of free fall from the spawn pose");
            Assert.Less(math.distance(vel[c].xyz, new float3(0, -10f / 60f, 1f)), 1e-4f, "velocity = spawn velocity + g dt");
            Assert.Less(math.length(ang[c].xyz), 1e-6f, "no angular velocity");
            Assert.Less(math.length(math.mul(new quaternion(rot[c]), math.inverse(quaternion.RotateY(0.3f))).value.xyz), 1e-5f, "spawn rotation");
            Assert.AreEqual(0u, world.GetEventsSync(c, 1)[0], "events reset");
            var color = new uint[1];
            world.Buffers.BodyColor.GetData(color, 0, c, 1);
            Assert.Less(color[0], (uint)AvbdGpuConstants.MaxColors + 1, "the respawned body is coloured again");
            Assert.AreEqual(2, pool.Alive);
            GpuTestUtil.AssertFinite(world);
        }

        [Test]
        public void PoolCapacityAndClear()
        {
            using var world = WorldWithGround();
            var pool = new BodyPool(world, 3);
            for (int i = 0; i < 3; i++) Assert.GreaterOrEqual(pool.Spawn(new float3(1, 1, 1), 1f, 0.5f, new float3(i * 2f, 0.5f, 0), quaternion.identity, float3.zero, 0u), 0);
            Assert.AreEqual(-1, pool.Spawn(new float3(1, 1, 1), 1f, 0.5f, float3.zero, quaternion.identity, float3.zero, 0u), "full");
            Assert.AreEqual(0, pool.Free);
            pool.Clear();
            Assert.AreEqual(0, pool.Alive);
            world.Step();
            Assert.AreEqual(0, world.GetStatsSync().Pairs, "nothing left to collide");
            for (int i = 0; i < 3; i++) Assert.GreaterOrEqual(pool.Spawn(new float3(1, 1, 1), 1f, 0.5f, new float3(i * 2f, 0.5f, 0), quaternion.identity, float3.zero, 0u), 0);
            world.Step();
            Assert.AreEqual(3, world.GetStatsSync().Pairs, "three boxes on the ground again");
        }

        [Test]
        public void EventsReportWhatUnitsAndProjectilesTouch()
        {
            using var world = WorldWithGround();
            int brick = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(6, 0.5f, 0), quaternion.identity, float3.zero);
            var units = new BodyPool(world, 4);
            var projectiles = new BodyPool(world, 8);
            int unit = units.Spawn(new float3(0.6f, 1.8f, 0.6f), 1f, 0.5f, new float3(0, 0.9f, 0), quaternion.identity, float3.zero, Unit | GpuBodyDef.FlagLockRotation);
            int arrow = projectiles.Spawn(new float3(0.1f, 0.1f, 0.6f), 1f, 0.5f, new float3(-3f, 1.2f, 0), quaternion.identity, new float3(12f, 0, 0), Projectile);
            int ball = projectiles.Spawn(new float3(0.4f, 0.4f, 0.4f), 1f, 0.5f, new float3(6, 4f, 0), quaternion.identity, float3.zero, Projectile);
            int idle = projectiles.Spawn(new float3(0.4f, 0.4f, 0.4f), 1f, 0.5f, new float3(0, 30f, 20f), quaternion.identity, float3.zero, Projectile);
            for (int i = 0; i < 60; i++) world.Step();
            uint eUnit = world.GetEventsSync(unit, 1)[0], eArrow = world.GetEventsSync(arrow, 1)[0], eBall = world.GetEventsSync(ball, 1)[0], eIdle = world.GetEventsSync(idle, 1)[0];
            Debug.Log($"events: unit {eUnit:X} arrow {eArrow:X} ball {eBall:X} idle {eIdle:X}");
            Assert.AreNotEqual(0u, eUnit & GpuBodyEvents.TouchStatic, "the unit stands on the ground");
            Assert.AreNotEqual(0u, eUnit & GpuBodyEvents.TouchProjectile, "the arrow hit the unit");
            Assert.GreaterOrEqual(GpuBodyEvents.Hits(eUnit), 1, "at least one impact counted");
            Assert.AreNotEqual(0u, eArrow & GpuBodyEvents.TouchUnit, "the arrow knows it hit a unit");
            Assert.AreNotEqual(0u, eBall & GpuBodyEvents.TouchBody, "the ball fell on the brick");
            Assert.AreEqual(0u, eIdle, "a projectile in the air touched nothing");
            Assert.AreEqual(0u, world.GetEventsSync(brick, 1)[0], "plain bodies do not report");
            GpuTestUtil.AssertFinite(world);
        }

        [Test]
        public void DeadFlagCollapsesNothingElse()
        {
            // a plain AddBody with the dead flag is a placeholder like a reserved slot; flags on live bodies can be changed later
            using var world = WorldWithGround();
            int box = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 3f, 0), quaternion.identity, float3.zero, GpuBodyDef.FlagLockRotation);
            for (int i = 0; i < 60; i++) world.Step();
            Assert.AreEqual(GpuBodyDef.FlagLockRotation, world.GetBodyFlags(box));
            world.SetBodyFlags(box, GpuBodyDef.FlagReportEvents);
            Assert.AreEqual(GpuBodyDef.FlagReportEvents, world.GetBodyFlags(box), "static / dead bits kept, the rest replaced");
            for (int i = 0; i < 30; i++) world.Step();
            Assert.AreNotEqual(0u, world.GetEventsSync(box, 1)[0] & GpuBodyEvents.TouchStatic, "reporting starts once the flag is set");
            world.RetireBody(box);
            Assert.IsFalse(world.IsAlive(box));
            world.Step();
            Assert.AreEqual(0, world.GetStatsSync().Pairs);
        }
    }
}
