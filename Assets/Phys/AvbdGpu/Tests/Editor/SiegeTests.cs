using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdGpu.Siege;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The siege layer: ballistics that match the solver's implicit Euler, units that march and hold, volleys that hit
    /// and kill, purges that free the pools; all with synchronous readbacks (no frame loop in EditMode).</summary>
    public class SiegeTests
    {
        const float G = 10f, Dt = 1f / 60f;

        [Test]
        public void BallisticsPassThroughTheTargetUnderImplicitEuler()
        {
            var cases = new[]
            {
                (from: new float3(0, 1, 0), to: new float3(20, 1, 5), elevation: 55f),
                (from: new float3(0, 1, 0), to: new float3(-30, 6, 10), elevation: 45f),
                (from: new float3(3, 8, -2), to: new float3(3, 0, 40), elevation: 30f),
            };
            foreach (var (from, to, elevationDeg) in cases)
            {
                Assert.IsTrue(Ballistics.AtElevation(from, to, math.radians(elevationDeg), G, Dt, out float3 v), "reachable");
                Assert.IsTrue(Ballistics.AtSpeed(from, to, math.length(v) * 1.2f, G, true, Dt, out float3 vHigh), "in range at 1.2 x the speed (high arc)");
                Assert.IsTrue(Ballistics.AtSpeed(from, to, math.length(v) * 1.2f, G, false, Dt, out float3 vLow), "low arc");
                foreach (var (velocity, label) in new[] { (v, "elevation"), (vHigh, "high arc"), (vLow, "low arc") })
                {
                    // closest approach of the discrete trajectory, interpolated within the step
                    float best = float.PositiveInfinity;
                    for (int n = 1; n < 2000; n++)
                    {
                        float3 a = Ballistics.Integrate(from, velocity, G, Dt, n - 1), b = Ballistics.Integrate(from, velocity, G, Dt, n);
                        float3 ab = b - a;
                        float t = math.clamp(math.dot(to - a, ab) / math.max(math.dot(ab, ab), 1e-12f), 0f, 1f);
                        best = math.min(best, math.distance(a + ab * t, to));
                        if (b.y < math.min(from.y, to.y) - 50f) break;
                    }
                    Assert.Less(best, 2e-3f, $"{label} from {from} to {to}: closest approach {best}");
                }
                Assert.IsFalse(Ballistics.AtSpeed(from, to, 5f, G, true, Dt, out _), "5 m/s cannot reach 20+ m");
            }
            Assert.IsFalse(Ballistics.AtElevation(new float3(0, 0, 0), new float3(10, 20, 0), math.radians(45f), G, Dt, out _), "a target above the elevation line is unreachable");
        }

        [Test]
        public void ALaunchedBoxFollowsTheBallisticSolution()
        {
            using var world = GpuTestUtil.NewWorld(1024);
            float3 from = new float3(0, 5, 0), to = new float3(25, 0.5f, -8);
            Assert.IsTrue(Ballistics.AtElevation(from, to, math.radians(50f), G, Dt, out float3 v));
            int box = world.AddBody(new float3(0.2f, 0.2f, 0.2f), 1f, 0.5f, from, quaternion.identity, v);
            float best = float.PositiveInfinity;
            float3 prev = from;
            for (int n = 0; n < 600; n++)
            {
                world.Step();
                world.GetPosesSync(out var pos, out _);
                float3 p = pos[box].xyz;
                float3 ab = p - prev;
                float t = math.clamp(math.dot(to - prev, ab) / math.max(math.dot(ab, ab), 1e-12f), 0f, 1f);
                best = math.min(best, math.distance(prev + ab * t, to));
                Assert.Less(math.distance(p, Ballistics.Integrate(from, v, G, Dt, n + 1)), 5e-3f, $"step {n}: the solver integrates the free flight (float round-off aside)");
                prev = p;
                if (p.y < 0f) break;
            }
            Assert.Less(best, 5e-3f, $"closest approach to the target {best}");
        }

        static SiegeSettings TestSettings()
        {
            var s = SiegeSettings.Default;
            s.Spread = 0f;
            s.VolleyInterval = 60; s.PurgeInterval = 120;
            s.ArchersPerRank = 3; s.Ranks = 1; s.Defenders = 4; s.MarchDistance = 4f; s.AttackDistance = 10f;
            return s;
        }

        /// <summary>A world with a ground, a siege with small pools and a step helper that reads poses and events back synchronously.</summary>
        sealed class Arena : System.IDisposable
        {
            public AvbdGpuWorld World;
            public SiegeSystem Siege;
            public Arena(int bodies = 4096)
            {
                World = GpuTestUtil.NewWorld(bodies);
                World.AddBody(new float3(400, 1, 400), 0f, 0.6f, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);
            }
            public void MakeSiege(SiegeSettings settings) => Siege = new SiegeSystem(World, SiegeSpec.Default, 64, 256, 64, settings);
            public void Run(int steps)
            {
                for (int i = 0; i < steps; i++)
                {
                    Siege.SyncEvents();
                    World.GetPosesSync(out var pos, out _);
                    Siege.Tick(pos, pos.Length, World.StepIndex);
                    World.Step();
                }
            }
            public void Dispose() => World.Dispose();
        }

        [Test]
        public void UnitsMarchToTheirWaypointAndHold()
        {
            using var a = new Arena();
            a.MakeSiege(TestSettings());
            var spec = a.Siege.Spec;
            int body = a.Siege.SpawnUnit(UnitKind.Archer, SiegeSystem.Attackers, new float3(-12, 0, 0), new float3(0, 0, 0), 0f);
            Assert.GreaterOrEqual(body, 0);
            a.Siege.AutoVolleys = false;
            a.Run(60);
            a.World.GetPosesSync(out var pos, out var rot);
            Assert.AreEqual(UnitState.Marching, a.Siege.UnitList[0].State);
            Assert.Greater(pos[body].x, -11f, "walking towards the waypoint");
            float3 forward = math.mul(new quaternion(rot[body]), new float3(0, 0, 1));
            Assert.Greater(forward.x, 0.99f, "facing the way it walks (heading)");
            Assert.Less(math.abs(pos[body].y - spec.UnitStandHeight), 0.05f, "standing on the ground");
            a.Run(420);
            a.World.GetPosesSync(out pos, out rot);
            Assert.AreEqual(UnitState.Holding, a.Siege.UnitList[0].State, "arrived");
            Assert.Less(math.length(pos[body].xz), 1f, $"holding at the waypoint, is at {pos[body]}");
            Assert.Greater(math.mul(new quaternion(rot[body]), new float3(0, 1, 0)).y, 0.9999f, "still upright");
            GpuTestUtil.AssertFinite(a.World);
        }

        [Test]
        public void AVolleyHitsAndKillsTheTarget()
        {
            using var a = new Arena();
            var settings = TestSettings();
            a.MakeSiege(settings);
            a.Siege.AutoVolleys = false; a.Siege.AutoPurges = false;
            int archer = a.Siege.SpawnUnit(UnitKind.Archer, SiegeSystem.Attackers, new float3(0, 0, -15), new float3(0, 0, -15), 0f, holding: true);
            // a gunner only aims at walls, so the victim does not shoot back
            int victim = a.Siege.SpawnUnit(UnitKind.Gunner, SiegeSystem.Defenders, new float3(0, 0, 0), new float3(0, 0, 0), math.PI, holding: true);
            a.Run(5);
            a.World.GetPosesSync(out var pos, out _);
            int fired = a.Siege.Volley(pos, pos.Length, a.World.StepIndex);
            Assert.AreEqual(1, fired, "the archer fires, the gunner has no wall to aim at");
            Assert.AreEqual(1, a.Siege.ProjectileList.Count);
            int arrowOfArcher = a.Siege.ProjectileList[0].Body;
            a.Run(1);   // the spawn reaches the GPU with the next step
            a.World.GetVelocitiesSync(out var vel, out _);
            Assert.Greater(vel[arrowOfArcher].z, 1f, "the attacker's arrow flies towards +z");
            Assert.Greater(vel[arrowOfArcher].y, 1f, "in an arc");
            a.Run(240);
            uint events = a.World.GetEventsSync(victim, 1)[0];
            Debug.Log($"victim events {events:X}, archer events {a.World.GetEventsSync(archer, 1)[0]:X}");
            Assert.GreaterOrEqual(GpuBodyEvents.Hits(events), 1, "the defender was hit");
            Assert.AreEqual(UnitState.Dead, a.Siege.UnitList[1].State, "and died");
            Assert.AreEqual(0, a.Siege.Alive[SiegeSystem.Defenders]);
            Assert.AreEqual(1, a.Siege.Dead[SiegeSystem.Defenders]);
            Assert.AreEqual(0u, a.World.GetBodyFlags(victim) & (GpuBodyDef.FlagLockRotation | GpuBodyDef.FlagHeading), "a dead unit may topple");
            // the arrow is spent, and the purge retires it together with the corpse
            Assert.IsTrue(GpuBodyEvents.Touched(a.World.GetEventsSync(arrowOfArcher, 1)[0]), "the arrow touched something");
            Assert.AreEqual(UnitState.Holding, a.Siege.UnitList[0].State, "the archer is untouched");
            int retired = a.Siege.Purge();
            Assert.AreEqual(2, retired, "one spent arrow and one dead unit");
            Assert.AreEqual(0, a.Siege.ProjectileList.Count);
            Assert.AreEqual(1, a.Siege.UnitList.Count);
            a.Run(2);
            Assert.AreEqual(a.Siege.Arrows.Capacity, a.Siege.Arrows.Free, "the arrow slot is free again");
            Assert.IsFalse(a.World.IsAlive(victim));
            GpuTestUtil.AssertFinite(a.World);
            Assert.AreEqual(0, a.World.GetStatsSync().OverflowFlags);
        }

        [Test]
        public void EveryProjectileKindFliesAndIsPurged()
        {
            using var a = new Arena();
            var settings = TestSettings();
            a.MakeSiege(settings);
            a.Siege.AutoVolleys = false; a.Siege.AutoPurges = false;
            var target = new float3(0, 0, 20);
            foreach (var kind in new[] { UnitKind.Archer, UnitKind.Gunner, UnitKind.Rocketeer, UnitKind.Mage })
            {
                float x = ((int)kind - 1.5f) * 4f;
                a.Siege.SpawnUnit(kind, SiegeSystem.Attackers, new float3(x, 0, 0), new float3(x, 0, 0), 0f, holding: true);
            }
            int wall = a.World.AddBody(new float3(20, 6, 1), 1f, 0.6f, target + new float3(0, 3, 0), quaternion.identity, float3.zero);   // a heavy slab to hit
            a.Run(5);
            a.World.GetPosesSync(out var pos, out _);
            // no enemies and no castle: only the gunner (aims at walls) has nothing to shoot at... give them all an aim by hand
            int fired = 0;
            foreach (var kind in new[] { ProjectileKind.Arrow, ProjectileKind.Cannonball, ProjectileKind.Rocket, ProjectileKind.Bolt })
            {
                float3 from = new float3(((int)kind - 1.5f) * 4f, 3f, 3f);
                float3 dir = math.normalize(target + new float3(0, 3, 0) - from);
                var drive = kind == ProjectileKind.Rocket ? GpuBodyDrive.ConstantForce(dir * settings.RocketThrust)
                          : kind == ProjectileKind.Bolt ? GpuBodyDrive.TowardsPoint(target + new float3(0, 3, 0), settings.BoltForce) : default;
                float speed = kind == ProjectileKind.Cannonball ? settings.CannonSpeed : kind == ProjectileKind.Rocket ? settings.RocketSpeed : kind == ProjectileKind.Bolt ? settings.BoltSpeed : 20f;
                if (a.Siege.Launch(kind, SiegeSystem.Attackers, from, dir * speed, drive) >= 0) fired++;
            }
            Assert.AreEqual(4, fired);
            Assert.AreEqual(2, a.Siege.Arrows.Alive); Assert.AreEqual(2, a.Siege.Shots.Alive);
            a.Run(180);
            foreach (var p in a.Siege.ProjectileList)
            {
                uint e = a.World.GetEventsSync(p.Body, 1)[0];
                Assert.IsTrue(GpuBodyEvents.Touched(e), $"{p.Kind} touched something within 3 s (events {e:X})");
            }
            Assert.AreEqual(4, a.Siege.Purge());
            Assert.AreEqual(0, a.Siege.Arrows.Alive + a.Siege.Shots.Alive);
            GpuTestUtil.AssertFinite(a.World);
        }

        /// <summary>The Outpost, dry-stacked and settled for 60 steps, with the test armies around it.</summary>
        static Arena Outpost()
        {
            var a = new Arena(8192);
            var plan = CastlePlan.Presets[0];
            var layout = BrickCastle.Generate(plan);
            float scale = 5f;
            float2 c = BrickCastle.Center(plan) * Brick.Pitch * scale;
            float volume = Brick.Width * Brick.BodyHeight * Brick.Length * scale * scale * scale;
            var brick = new BrickSpec { Scale = scale, Density = 0.25f / volume, Friction = 0.6f, Margin = AvbdGpuConstants.CollisionMargin, Origin = new float3(-c.x, 0, -c.y) };
            BrickCastle.Build(a.World, layout, brick);
            a.World.Params.Substeps = 2;
            for (int i = 0; i < 60; i++) a.World.Step();   // settle
            a.MakeSiege(TestSettings());
            a.Siege.SpawnArmies(layout, plan, brick);
            return a;
        }

        [Test]
        public void SiegeOfTheOutpostRuns()
        {
            using var a = Outpost();
            int attackers = a.Siege.Alive[SiegeSystem.Attackers], defenders = a.Siege.Alive[SiegeSystem.Defenders];
            Debug.Log($"outpost siege: {attackers} attackers, {defenders} defenders");
            Assert.AreEqual(12, attackers, "3 per side");
            Assert.AreEqual(4, defenders);
            a.Run(600);   // 10 s: the attackers march 4 m, volleys every second, purges every two
            Debug.Log("outpost siege: " + a.Siege.Summary());
            Assert.Greater(a.Siege.Volleys, 5);
            Assert.Greater(a.Siege.ShotsFired, 20);
            Assert.Greater(a.Siege.Purges, 3);
            foreach (var u in a.Siege.UnitList) if (u.Team == SiegeSystem.Attackers && u.State != UnitState.Dead) Assert.AreEqual(UnitState.Holding, u.State, "attackers arrived at their firing line");
            Assert.Less(a.Siege.ProjectileList.Count, a.Siege.ShotsFired, "purges retired spent projectiles");
            Assert.AreEqual(0, a.World.GetStatsSync().OverflowFlags);
            GpuTestUtil.AssertFinite(a.World);
        }

        /// <summary>With synchronous readbacks the whole siege is scripted by the step index: two runs are bitwise identical
        /// (spawns are ordered on the CPU, the event atomics are order-independent).</summary>
        [Test]
        public void ScriptedSiegeIsBitwiseDeterministic()
        {
            float4[] posA, rotA, posB, rotB;
            int shotsA, shotsB;
            using (var a = Outpost()) { a.Run(300); a.World.GetPosesSync(out posA, out rotA); shotsA = a.Siege.ShotsFired; }
            using (var b = Outpost()) { b.Run(300); b.World.GetPosesSync(out posB, out rotB); shotsB = b.Siege.ShotsFired; }
            Assert.AreEqual(shotsA, shotsB);
            Assert.Greater(shotsA, 20);
            Assert.AreEqual(posA.Length, posB.Length);
            int differ = 0;
            for (int i = 0; i < posA.Length; i++)
                if (!posA[i].Equals(posB[i]) || !rotA[i].Equals(rotB[i])) differ++;
            Assert.AreEqual(0, differ, $"{differ} of {posA.Length} bodies differ between two runs of the same siege");
        }
    }
}
