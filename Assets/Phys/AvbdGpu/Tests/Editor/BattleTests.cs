using System.Collections.Generic;
using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdGpu.Siege;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The battle: orders that walk units in formation and into range, shots by trajectory, kills, impacts that break snaps
    /// and toss bricks, defenders on the castle's posts, the pools recycling; synchronous readbacks (no frame loop in EditMode).</summary>
    public class BattleTests
    {
        const float Margin = 0.01f;

        // ---- archetypes at the demo's scale (a 0.48 m figure x 5)
        static UnitArchetype Archer(string name = "archer") => new UnitArchetype
        {
            Name = name, BoxSize = new float3(1.77f, 2.4f + Margin, 0.6f), MeshOffset = new float3(0f, -1.2f + Margin * 0.5f, 0f), StandHeight = 1.2f - Margin * 0.5f,
            Mass = 1f, Friction = 0.6f, Speed = 3f, Force = 12f, HitPoints = 1, Range = 25f, CooldownSteps = 60, AutoEngage = true,
            Trajectory = Trajectory.Elevation, ElevationDeg = 55f, MaxSpeed = 40f, Spread = 0f, LaunchLocal = new float3(0f, 0.6f, 0f), LaunchOffset = 2.6f,
            ProjectileBoxSize = new float3(0.17f, 0.185f, 1.4f), ProjectileMass = 0.02f, ProjectileAlign = true, ProjectileRetireDelay = 120,
        };

        static UnitArchetype ExplosiveArcher()
        {
            var a = Archer("explosive archer");
            a.ProjectileBoxSize = new float3(0.22f, 0.23f, 1.4f);
            a.ProjectileMass = 0.05f;
            a.ProjectileRetireDelay = 0;
            a.Hit = new HitEffect { PulverizeRadius = 2f, ImpactRadius = 3f, Impulse = 8f, Lift = 0.3f, KillRadius = 2f };
            return a;
        }

        /// <summary>The default Trebuchet config's numbers (the Base model's bounds x 5, the basin as the launch point).</summary>
        static UnitArchetype Trebuchet() => new UnitArchetype
        {
            Name = "trebuchet", BoxSize = new float3(3.63f, 3.585f + Margin, 4.6f), MeshOffset = new float3(0f, -1.7925f + Margin * 0.5f, 0f), StandHeight = 1.7925f - Margin * 0.5f,
            Mass = 30f, Friction = 0.6f, Speed = 1.2f, Force = 120f, HitPoints = 5,
            Range = 70f, CooldownSteps = 240, AutoEngage = false, Trajectory = Trajectory.HighArc, LaunchSpeed = 32f, Spread = 0f,
            LaunchLocal = new float3(0f, 5.25f - 1.7925f, 3.25f), LaunchOffset = 1f, LaunchDelaySteps = 30,
            ProjectileBoxSize = new float3(0.58f, 0.58f, 0.56f), ProjectileMass = 6f, ProjectileAlign = false, ProjectileVariants = 4, ProjectileRetireDelay = 300,
            Hit = new HitEffect { PulverizeRadius = 0.8f, ImpactRadius = 2.5f, Impulse = 8f, Lift = 0.5f, KillRadius = 1.5f },
        };

        static UnitArchetype Mage()
        {
            var a = Archer("mage");
            a.Trajectory = Trajectory.Homing; a.LaunchSpeed = 16f; a.Thrust = 6f; a.Range = 30f; a.CooldownSteps = 120;
            a.ProjectileBoxSize = new float3(0.5f, 0.5f, 1.5f); a.ProjectileMass = 0.1f; a.ProjectileRetireDelay = 0;
            a.Hit = new HitEffect { PulverizeRadius = 2f, ImpactRadius = 4f, Impulse = 15f, Lift = 0.3f, KillRadius = 3f };
            return a;
        }

        /// <summary>A flat static ground, a battle with the given archetypes (unit pools of 16, projectile pools of 128 each).</summary>
        sealed class Arena : System.IDisposable
        {
            public readonly AvbdGpuWorld World;
            public readonly Battle Battle;
            public readonly List<(int type, float3 at)> Impacts = new List<(int, float3)>();
            public readonly List<(int unit, int step)> Shots = new List<(int, int)>();

            public Arena(params UnitArchetype[] types) : this(4096, types) { }

            public Arena(int bodies, params UnitArchetype[] types)
            {
                World = GpuTestUtil.NewWorld(bodies);
                World.AddBody(new float3(400, 1, 400), 0f, 0.6f, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);
                var units = new int[types.Length]; var shots = new int[types.Length];
                for (int i = 0; i < types.Length; i++) { units[i] = 16; shots[i] = 128; }
                Battle = new Battle(World, types, units, shots, BattleSettings.Default);
                Battle.OnImpact = (type, at) => Impacts.Add((type, at));
                Battle.OnShot = unit => Shots.Add((unit, World.StepIndex));
            }

            /// <summary>Steps with the battle ticking on synchronous readbacks: the current poses and events every step.</summary>
            public void Run(int steps)
            {
                for (int i = 0; i < steps; i++)
                {
                    World.ReadEventsSync();
                    World.GetPosesSync(out var pos, out _);
                    Battle.Tick(pos, pos.Length, World.StepIndex);
                    World.Step();
                }
            }

            public float3 Position(int body) { World.GetPosesSync(out var pos, out _); return pos[body].xyz; }
            public int Spawn(int type, int team, float3 foot, float yaw = 0f) => Battle.SpawnUnit(type, team, foot, yaw);
            public void Dispose() => World.Dispose();
        }

        [Test]
        public void MoveKeepsFormationOffsets()
        {
            using var a = new Arena(Archer());
            var units = new List<int>();
            for (int i = 0; i < 3; i++) { a.Spawn(0, Battle.Attackers, new float3(-2f + 2f * i, 0f, 0f)); units.Add(i); }
            a.Run(2);
            a.Battle.Move(units, new float3(0f, 0f, 20f));
            foreach (int i in units) Assert.AreEqual(UnitState.Moving, a.Battle.Units[i].State);
            a.Run(600);
            a.World.GetPosesSync(out var pos, out var rot);
            for (int i = 0; i < 3; i++)
            {
                var u = a.Battle.Units[i];
                float3 p = pos[u.Body].xyz;
                Assert.Less(math.distance(p.xz, new float2(-2f + 2f * i, 20f)), 0.7f, $"unit {i} arrived keeping its offset: {p}");
                Assert.AreEqual(UnitState.Idle, u.State, $"unit {i} idle on arrival");
                Assert.AreEqual(OrderKind.None, u.Order);
                float3 up = math.mul(new quaternion(rot[u.Body]), new float3(0, 1, 0));
                Assert.Greater(up.y, 0.999f, $"unit {i} upright");
                Assert.AreEqual(a.Battle.StandHeight(0, p.xz), p.y, 0.05f, "standing on the ground");
            }
            GpuTestUtil.AssertFinite(a.World);
        }

        [Test]
        public void AttackWalksIntoRangeThenFires()
        {
            using var a = new Arena(Archer());
            int slab = a.World.AddBody(new float3(2f, 1f, 2f), 50f, 0.6f, new float3(0f, 0.5f, 0f), quaternion.identity, float3.zero);
            a.Spawn(0, Battle.Attackers, new float3(0f, 0f, -40f));
            a.Run(2);
            a.Battle.Attack(new[] { 0 }, slab);
            Assert.AreEqual(UnitState.Moving, a.Battle.Units[0].State, "40 m off: it walks in first");
            int firedAt = -1;
            for (int i = 0; i < 60 && firedAt < 0; i++)
            {
                a.Run(10);
                if (a.Battle.ShotsFired > 0) firedAt = a.World.StepIndex;
            }
            Assert.Greater(firedAt, 0, "it fired");
            var u = a.Battle.Units[0];
            Assert.AreEqual(UnitState.Attacking, u.State);
            Assert.AreEqual(OrderKind.Attack, u.Order);
            float3 p = a.Position(u.Body);
            Assert.LessOrEqual(math.length(p.xz), a.Battle.Types[0].Range + 1f, "within range of the target");
            Assert.Greater(math.length(p.xz), a.Battle.Types[0].Range * 0.6f, "it stopped at its firing line, not on the target");
            Assert.Greater(firedAt, 200, "it walked some 19 m first");
            var proj = a.Battle.Projectiles[0];
            a.World.GetVelocitiesSync(out var vel, out _);
            if (!proj.Spent)
            {
                float3 v = vel[proj.Body].xyz;
                Assert.Greater(v.z, 1f, "the arrow flies toward the target");
                Assert.Greater(v.y, 1f, "and upward, on its elevation");
            }
            // a unit already in range shoots where it stands
            a.Spawn(0, Battle.Attackers, new float3(10f, 0f, -10f));
            a.Run(2);
            a.Battle.Attack(new[] { 1 }, slab);
            Assert.AreEqual(UnitState.Attacking, a.Battle.Units[1].State, "in range: no walk");
            GpuTestUtil.AssertFinite(a.World);
        }

        [Test]
        public void ArrowsKillAndToppleAndTheDeadRetire()
        {
            var defender = Archer("target"); defender.AutoEngage = false;
            using var a = new Arena(Archer(), defender);
            a.Spawn(0, Battle.Attackers, new float3(0f, 0f, 0f));
            int victim = a.Spawn(1, Battle.Defenders, new float3(0f, 0f, 15f), math.PI);
            int deathStep = -1;
            for (int i = 0; i < 30 && deathStep < 0; i++)
            {
                a.Run(10);
                if (a.Battle.Units[1].State == UnitState.Dead) deathStep = a.Battle.Units[1].DeathStep;
            }
            Assert.Greater(deathStep, 0, "the auto-engaging archer killed the defender in range");
            Assert.Greater(a.Battle.ShotsFired, 0);
            Assert.AreEqual(1, a.Battle.Dead[Battle.Defenders]);
            Assert.AreEqual(0, a.Battle.Alive[Battle.Defenders]);
            uint flags = a.World.GetBodyFlags(victim);
            Assert.AreEqual(0u, flags & (GpuBodyDef.FlagHeading | GpuBodyDef.FlagDriven), "the dead lose their heading and motor");
            int retireAt = deathStep + a.Battle.Settings.RetireDelay;
            while (a.World.StepIndex < retireAt - 1) a.Run(1);
            Assert.AreEqual(2, a.Battle.Units.Count, "still there a step before its retirement");
            a.Run(3);
            Assert.AreEqual(1, a.Battle.Units.Count, "retired");
            Assert.AreEqual(-1, a.Battle.UnitOfBody(victim));
            Assert.IsFalse(a.World.IsAlive(victim));
            Assert.AreEqual(0, a.Battle.UnitOfBody(a.Battle.Units[0].Body), "the survivor's index moved up");
            GpuTestUtil.AssertFinite(a.World);
        }

        /// <summary>Two columns of four cubes glued with unbreakable snaps 14 m from an explosive archer: its arrow's impact breaks the
        /// snaps within the pulverize radius, tosses the cubes, retires the arrow at once and reports the impact.</summary>
        [Test]
        public void ExplosiveArrowBreaksSnapsAndTossesBricks()
        {
            using var a = new Arena(ExplosiveArcher());
            var joints = new List<int>();
            var cubes = new List<int>();
            float inf = float.PositiveInfinity;
            int ground = 0;
            for (int c = 0; c < 2; c++)
            {
                int below = ground;
                for (int h = 0; h < 4; h++)
                {
                    int cube = a.World.AddBody(new float3(1, 1, 1), 1f, 0.6f, new float3(c - 0.5f, 0.5f + h, 14f), quaternion.identity, float3.zero);
                    cubes.Add(cube);
                    joints.Add(a.World.AddJointIndexed(below, cube, h == 0 ? new float3(c - 0.5f, 0.5f, 14f) : new float3(0, 0.5f, 0), new float3(0, -0.5f, 0), inf, inf, inf, inf, inf, inf, 2));
                    below = cube;
                }
            }
            a.Spawn(0, Battle.Attackers, float3.zero);
            a.Run(2);
            a.Battle.Attack(new[] { 0 }, cubes[3]);   // the top of the first column
            Assert.AreEqual(UnitState.Attacking, a.Battle.Units[0].State, "14 m: in range");
            a.World.GetPosesSync(out var before, out _);
            for (int i = 0; i < 60 && a.Battle.Impacts == 0; i++) a.Run(5);
            Assert.GreaterOrEqual(a.Battle.Impacts, 1, "an arrow hit and went off");
            Assert.AreEqual(a.Battle.Impacts, a.Impacts.Count);
            Assert.Less(math.distance(a.Impacts[0].at, new float3(0f, 2f, 14f)), 3f, $"the impact is at the wall: {a.Impacts[0].at}");
            foreach (var p in a.Battle.Projectiles) Assert.IsFalse(p.Spent, "an exploding arrow is gone at its impact");
            a.Run(120);
            var states = a.World.GetJointStatesSync();
            int broken = 0;
            var report = new System.Text.StringBuilder($"impact at {a.Impacts[0].at}:");
            foreach (int j in joints)
            {
                if (states[j].Broken != 0) broken++;
                var d = a.World.GetJointDef(j);
                report.Append($" [{j}: B {d.BodyB} at {before[d.BodyB].xyz} {(states[j].Broken != 0 ? "broken" : "holds")}]");
            }
            Assert.Greater(broken, 0, "snaps within the pulverize radius broke: " + report);
            a.World.GetPosesSync(out var after, out _);
            float moved = 0f;
            foreach (int c in cubes) moved = math.max(moved, math.distance(after[c].xyz, before[c].xyz));
            Assert.Greater(moved, 0.5f, "the blast tossed the cubes");
            GpuTestUtil.AssertFinite(a.World);
        }

        [Test]
        public void TrebuchetLobsOnAHighArcAfterItsDelay()
        {
            using var a = new Arena(Trebuchet());
            int slab = a.World.AddBody(new float3(4f, 1f, 4f), 200f, 0.6f, new float3(0f, 0.5f, 0f), quaternion.identity, float3.zero);
            a.Spawn(0, Battle.Attackers, new float3(0f, 0f, -50f));
            a.Run(2);
            a.Battle.Attack(new[] { 0 }, slab);
            Assert.AreEqual(UnitState.Attacking, a.Battle.Units[0].State, "50 m: within its 60 m range");
            while (a.Shots.Count == 0 && a.World.StepIndex < 400) a.Run(1);
            Assert.AreEqual(1, a.Shots.Count, "the shot started");
            int started = a.Shots[0].step;
            Assert.AreEqual(0, a.Battle.ShotsFired, "the rock is not out yet");
            Assert.GreaterOrEqual(a.Battle.Units[0].PendingShotStep, started + 30);
            a.Run(29);
            Assert.AreEqual(0, a.Battle.ShotsFired, "still swinging");
            a.Run(2);
            Assert.AreEqual(1, a.Battle.ShotsFired, "the rock leaves at the top of the swing");
            var rock = a.Battle.Projectiles[0];
            Assert.AreEqual(0, rock.Variant);
            a.World.GetVelocitiesSync(out var vel, out _);
            float3 v = vel[rock.Body].xyz;
            Assert.Greater(v.y, 0.8f * math.length(v.xz), $"a high arc: {v}");
            Assert.Greater(v.z, 0f, "toward the target");
            // the discrete trajectory passes over the target: closest approach to the aim point
            float3 from = a.Position(rock.Body);
            float g = math.length(a.World.Params.Gravity), dt = a.World.Params.Dt / math.max(1, a.World.Params.Substeps);
            float best = float.PositiveInfinity;
            float3 aim = new float3(0f, 0.5f, 0f);
            for (int n = 1; n < 1200; n++)
            {
                float3 p0 = Ballistics.Integrate(from, v, g, dt, n - 1), p1 = Ballistics.Integrate(from, v, g, dt, n);
                float3 ab = p1 - p0;
                float t = math.clamp(math.dot(aim - p0, ab) / math.max(math.dot(ab, ab), 1e-12f), 0f, 1f);
                best = math.min(best, math.distance(p0 + ab * t, aim));
                if (p1.y < -5f) break;
            }
            Assert.Less(best, 1.5f, $"aimed at the slab (closest approach {best:F2} m; the rock leaves a step or two after the solution)");
            // the rock lands; the machine is untouched by its own shots (the basin's launch point clears its box)
            a.Run(600);
            Assert.GreaterOrEqual(a.Battle.Impacts, 1, "the rock landed and went off");
            Assert.AreEqual(UnitState.Attacking, a.Battle.Units[0].State, "the trebuchet is alive and attacking");
            Assert.AreEqual(0, a.Battle.Units[0].Hits, "never hit by its own rocks");
            Assert.Greater(a.Battle.ShotsFired, 1, "and it keeps firing");
            GpuTestUtil.AssertFinite(a.World);
        }

        [Test]
        public void MagesHomeInAndBlast()
        {
            using var a = new Arena(Mage());
            int slab = a.World.AddBody(new float3(3f, 2f, 3f), 100f, 0.6f, new float3(0f, 1f, 0f), quaternion.identity, float3.zero);
            a.Spawn(0, Battle.Attackers, new float3(0f, 0f, -20f));
            a.Run(2);
            a.Battle.Attack(new[] { 0 }, slab);
            while (a.Battle.ShotsFired == 0 && a.World.StepIndex < 400) a.Run(1);
            Assert.AreEqual(1, a.Battle.ShotsFired);
            var bolt = a.Battle.Projectiles[0];
            Assert.AreEqual(GpuBodyDrive.ToPoint, a.World.GetBodyDrive(bolt.Body).Mode, "the spell homes in");
            for (int i = 0; i < 60 && a.Battle.Impacts == 0; i++) a.Run(5);
            Assert.AreEqual(1, a.Battle.Impacts, "it hit and went off");
            Assert.Less(math.distance(a.Impacts[0].at.xz, float2.zero), 5f, $"near the slab: {a.Impacts[0].at}");
            GpuTestUtil.AssertFinite(a.World);
        }

        /// <summary>The Outpost document snapped at the demo's scale with its occupancy posts: archers spawned on the wall posts stand
        /// there.</summary>
        [Test]
        public void DefendersStandOnTheOutpostsPosts()
        {
            var document = Resources.Load<TextAsset>("Castles/outpost");
            Assert.IsNotNull(document, "Resources/Castles/outpost.json");
            var assembly = BrickAssembly.Parse(document.text);
            var occupancy = assembly.OccupancyOrDerived();
            if (!occupancy.ExplicitPosts) occupancy.DerivePosts(AssemblyOccupancy.PostRules.Default);
            Assert.GreaterOrEqual(occupancy.Posts.Count, 8);
            using var a = new Arena(16384, Archer());   // room for the castle's 8 000 snaps
            float scale = 5f, unit = PieceCatalog.GridToUnity * scale;
            float3 centre = (assembly.Min + assembly.Max) * 0.5f;
            var spec = new AssemblySpec
            {
                Scale = scale, Density = 1f / (2f * 3f * 1.2f * unit * unit * unit), Friction = 0.6f, Margin = Margin, Clearance = AssemblySpec.DefaultClearance,
                Origin = new float3(-centre.x * unit, 0f, -centre.z * unit),
            };
            var bodies = AssemblyBuilder.Build(a.World, assembly, spec);
            AssemblyBuilder.AddSnapJoints(a.World, assembly, bodies, spec, 800f, 300f);
            a.World.Params.Substeps = 2;
            a.Battle.Occupancy = occupancy; a.Battle.OccupancyOrigin = spec.Origin; a.Battle.OccupancyUnit = unit;
            var posts = new List<AssemblyPost>();
            foreach (var p in occupancy.Posts) if (p.OnWall && posts.Count < 8) posts.Add(p);
            Assert.AreEqual(8, posts.Count, "eight wall posts");
            var units = new List<int>();
            foreach (var p in posts) units.Add(a.Battle.UnitOfBody(a.Spawn(0, Battle.Defenders, spec.Origin + p.Position * unit, p.Yaw)));
            a.Run(180);
            a.World.GetPosesSync(out var pos, out _);
            for (int i = 0; i < posts.Count; i++)
            {
                float3 expected = spec.Origin + posts[i].Position * unit;
                float3 p = pos[a.Battle.Units[units[i]].Body].xyz;
                Assert.Less(math.distance(p.xz, expected.xz), 0.6f, $"defender {i} stays on its post ({p} vs {expected})");
                Assert.AreEqual(expected.y + a.Battle.Types[0].StandHeight, p.y, 0.35f, $"defender {i} stands on the wall top");
                Assert.IsTrue(a.Battle.InsideCastle(expected - new float3(0f, 1f, 0f)), "the wall under the post is solid");
                Assert.IsFalse(a.Battle.InsideCastle(p + new float3(0f, 1f, 0f)), "the air above it is not");
            }
            Assert.AreEqual(8, a.Battle.Alive[Battle.Defenders]);
            GpuTestUtil.AssertFinite(a.World);
        }

        [Test]
        public void SpentProjectilesRetireAndSlotsRecycleWithoutStaleEvents()
        {
            using var a = new Arena(Archer());
            var pool = a.Battle.ProjectilePools[0][0];
            int capacity = pool.Capacity;
            // arrows straight down into the ground: spent within a few steps
            for (int i = 0; i < 8; i++) Assert.GreaterOrEqual(a.Battle.Launch(0, Battle.Attackers, 0, new float3(i * 2f, 3f, 0f), new float3(0, -20f, 0), default), 0);
            Assert.AreEqual(8, a.Battle.Projectiles.Count);
            a.Run(30);
            foreach (var p in a.Battle.Projectiles) Assert.IsTrue(p.Spent, "landed");
            a.Run(a.Battle.Settings.RetireDelay + 5);
            Assert.AreEqual(0, a.Battle.Projectiles.Count, "retired after the delay");
            a.Run(2);
            Assert.AreEqual(capacity, pool.Free, "every slot is free again");
            // the recycled slots still hold their predecessors' event words for a couple of frames: a new arrow in the air is not spent
            int body = a.Battle.Launch(0, Battle.Attackers, 0, new float3(0f, 30f, 0f), new float3(0, 12f, 0), default);
            Assert.GreaterOrEqual(body, 0);
            a.Run(a.Battle.Settings.EventGuardSteps + 3);
            Assert.IsFalse(a.Battle.Projectiles[0].Spent, "an arrow flying upward through the air is not spent by a stale word");
            GpuTestUtil.AssertFinite(a.World);
        }

        [Test]
        public void ScriptedBattleIsBitwiseDeterministic()
        {
            var (posA, shotsA, impactsA) = Scripted();
            var (posB, shotsB, impactsB) = Scripted();
            Assert.AreEqual(shotsA, shotsB);
            Assert.AreEqual(impactsA, impactsB);
            Assert.AreEqual(posA.Length, posB.Length);
            for (int i = 0; i < posA.Length; i++) Assert.IsTrue(math.all(posA[i] == posB[i]), $"body {i}: {posA[i]} vs {posB[i]}");
        }

        static (float4[] poses, int shots, int impacts) Scripted()
        {
            var defender = Archer("defender");
            using var a = new Arena(Archer(), ExplosiveArcher(), defender);
            float inf = float.PositiveInfinity;
            int below = 0;
            for (int h = 0; h < 5; h++)
            {
                int cube = a.World.AddBody(new float3(1, 1, 1), 1f, 0.6f, new float3(0f, 0.5f + h, 20f), quaternion.identity, float3.zero);
                a.World.AddJointIndexed(below, cube, h == 0 ? new float3(0f, 0.5f, 20f) : new float3(0, 0.5f, 0), new float3(0, -0.5f, 0), inf, inf, inf, inf, inf, inf, 2);
                below = cube;
            }
            int top = below;
            for (int i = 0; i < 3; i++) a.Spawn(0, Battle.Attackers, new float3(-4f + 4f * i, 0f, 0f));
            a.Spawn(1, Battle.Attackers, new float3(6f, 0f, -2f));
            for (int i = 0; i < 2; i++) a.Spawn(2, Battle.Defenders, new float3(-3f + 6f * i, 0f, 24f), math.PI);
            a.Run(2);
            a.Battle.Move(new[] { 0, 1, 2 }, new float3(0f, 0f, 6f));
            a.Battle.Attack(new[] { 3 }, top);
            a.Run(300);
            a.World.GetPosesSync(out var pos, out _);
            return (pos, a.Battle.ShotsFired, a.Battle.Impacts);
        }
    }
}
