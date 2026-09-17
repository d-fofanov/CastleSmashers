// The battle: units of the archetypes in body pools, the player's orders (move, attack), steering, shooting on a cooldown,
// the impacts of the shots (blasts) and the retirement of the dead and the spent. Gameplay reads the asynchronous pose
// readback (a frame or two old) and drives the bodies through their motors; the solver stays authoritative.

using System;
using System.Collections.Generic;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Siege
{
    public enum UnitState : byte { Idle, Moving, Attacking, Dead }
    public enum OrderKind : byte { None, Move, Attack }

    public struct Unit
    {
        public int Body;
        public int Team;              // 0 attackers, 1 defenders
        public int Type;              // index into Battle.Types
        public UnitState State;
        public OrderKind Order;
        public float3 Goal;           // where a moving unit walks to (Move: the point; Attack: the firing position)
        public int TargetBody;        // the body an attack order aims at
        public float Yaw;
        public int Hits;              // fast projectile hits read back so far
        public int SpawnStep;         // read-back positions of earlier steps do not hold the unit yet
        public int DeathStep;
        public int NextShotStep;      // the cooldown
        public int LastShotStep;      // -1 before the first shot (the weapon's swing runs from it)
        public int PendingShotStep;   // a shot started, its projectile leaves at this step (-1: none)
        public float3 PendingAim;
        public float3 Aim;            // the last aim point (an idle unit faces it)
        public bool HasAim;
    }

    public struct Projectile
    {
        public int Body;
        public int Type, Team, Variant;
        public int LaunchStep;
        public int SpentStep;         // -1 while in flight
        public bool HitApplied;       // its hit effect (if any) went off
        public bool Spent => SpentStep >= 0;
    }

    /// <summary>Tunables shared by every unit (steps at 60 Hz, solver metres).</summary>
    [Serializable]
    public struct BattleSettings
    {
        /// <summary>Steps a dead unit lies about before its slot is freed.</summary>
        public int RetireDelay;
        /// <summary>A projectile that has not touched anything within this many steps is retired anyway.</summary>
        public int ProjectileMaxAge;
        /// <summary>Steps after a launch before the (sticky, recycled) event word of a projectile's slot is trusted.</summary>
        public int EventGuardSteps;
        /// <summary>A moving unit has arrived within this distance of its goal.</summary>
        public float ArriveRadius;
        /// <summary>An attack order walks the unit to this fraction of its range from the target.</summary>
        public float RangeFraction;

        public static BattleSettings Default => new BattleSettings { RetireDelay = 120, ProjectileMaxAge = 1800, EventGuardSteps = 3, ArriveRadius = 0.5f, RangeFraction = 0.85f };

        /// <summary>Every field a scene was serialised without (zero) at its default.</summary>
        public BattleSettings WithDefaults()
        {
            var d = Default;
            var s = this;
            if (s.RetireDelay <= 0) s.RetireDelay = d.RetireDelay;
            if (s.ProjectileMaxAge <= 0) s.ProjectileMaxAge = d.ProjectileMaxAge;
            if (s.EventGuardSteps <= 0) s.EventGuardSteps = d.EventGuardSteps;
            if (s.ArriveRadius <= 0f) s.ArriveRadius = d.ArriveRadius;
            if (s.RangeFraction <= 0f) s.RangeFraction = d.RangeFraction;
            return s;
        }
    }

    /// <summary>Two teams of units drawn from the archetypes, each archetype with a body pool for its units and one per projectile
    /// variant. Once per step, before the world steps, <see cref="Tick"/> reads the hits and contacts, applies the impacts, steers
    /// the units after their orders, fires the shots whose cooldown is up and retires the dead and the spent.</summary>
    public sealed class Battle
    {
        public const int Attackers = 0, Defenders = 1;
        const uint UnitFlags = GpuBodyDef.FlagReportEvents | GpuBodyDef.FlagDriven | GpuBodyDef.FlagHeading | (GpuBodyDef.KindUnit << GpuBodyDef.KindShift);
        const uint DeadUnitFlags = GpuBodyDef.FlagReportEvents | (GpuBodyDef.KindUnit << GpuBodyDef.KindShift);
        const uint ProjectileFlags = GpuBodyDef.FlagReportEvents | (GpuBodyDef.KindProjectile << GpuBodyDef.KindShift);

        public readonly AvbdGpuWorld World;
        public readonly UnitArchetype[] Types;
        public BattleSettings Settings;
        /// <summary>The terrain the units stand on (null: flat at y = 0).</summary>
        public Heightfield Terrain;
        /// <summary>The castle's occupancy map: shots never start inside a wall. World = <see cref="OccupancyOrigin"/> + grid x <see cref="OccupancyUnit"/>.</summary>
        public AssemblyOccupancy Occupancy;
        public float3 OccupancyOrigin;
        public float OccupancyUnit = 0.5f;

        /// <summary>One pool per archetype (null where the roster has none of it), one per archetype and projectile variant.</summary>
        public readonly BodyPool[] UnitPools;
        public readonly BodyPool[][] ProjectilePools;
        public readonly List<Unit> Units = new List<Unit>();
        public readonly List<Projectile> Projectiles = new List<Projectile>();

        /// <summary>(body, type) of a unit / projectile just spawned: tints and attachments.</summary>
        public Action<int, int> OnUnitSpawned, OnProjectileSpawned;
        /// <summary>A body about to be retired (release anything holding it).</summary>
        public Action<int> OnRetiring;
        /// <summary>A unit (index into <see cref="Units"/>) started a shot: its weapon swings.</summary>
        public Action<int> OnShot;
        /// <summary>(type, world position) of a projectile's impact with a hit effect.</summary>
        public Action<int, float3> OnImpact;

        public readonly int[] Alive = new int[2], Dead = new int[2];
        public int ShotsFired, Retired, Impacts;

        readonly Dictionary<int, int> m_UnitOfBody = new Dictionary<int, int>();
        float4[] m_Positions;
        int m_PositionCount, m_PositionsStep = -1;

        /// <summary>Reserves the pools: <paramref name="unitCapacity"/>[type] units and <paramref name="projectileCapacity"/>[type]
        /// projectiles (split over the archetype's variants) per archetype.</summary>
        public Battle(AvbdGpuWorld world, UnitArchetype[] types, int[] unitCapacity, int[] projectileCapacity, BattleSettings settings)
        {
            World = world;
            Types = types;
            Settings = settings.WithDefaults();
            UnitPools = new BodyPool[types.Length];
            ProjectilePools = new BodyPool[types.Length][];
            for (int t = 0; t < types.Length; t++)
            {
                if (unitCapacity[t] > 0) UnitPools[t] = new BodyPool(world, unitCapacity[t]);
                int variants = math.max(types[t].ProjectileVariants, 1);
                ProjectilePools[t] = new BodyPool[variants];
                int per = (projectileCapacity[t] + variants - 1) / variants;
                if (per > 0) for (int v = 0; v < variants; v++) ProjectilePools[t][v] = new BodyPool(world, per);
            }
        }

        public static float YawOf(float3 forward) => math.atan2(forward.x, forward.z);
        public float GroundHeight(float2 xz) => Terrain?.Height(xz) ?? 0f;
        /// <summary>The body centre height of a unit of the type standing on the ground at xz.</summary>
        public float StandHeight(int type, float2 xz) => GroundHeight(xz) + Types[type].StandHeight;
        public int LiveProjectiles { get { int n = 0; foreach (var p in Projectiles) if (!p.Spent) n++; return n; } }
        /// <summary>The unit (index into <see cref="Units"/>) with the body, or -1.</summary>
        public int UnitOfBody(int body) => m_UnitOfBody.TryGetValue(body, out int i) ? i : -1;
        float SubstepDt => World.Params.Dt / math.max(1, World.Params.Substeps);

        /// <summary>Whether a world point lies inside the castle's walls (by the occupancy map).</summary>
        public bool InsideCastle(float3 world)
        {
            if (Occupancy == null) return false;
            return Occupancy.Solid((world - OccupancyOrigin) / OccupancyUnit);
        }

        /// <summary>Live units of a team (and type, -1 = any), as indices into <see cref="Units"/>.</summary>
        public List<int> Select(int team, int type = -1)
        {
            var result = new List<int>();
            for (int i = 0; i < Units.Count; i++)
                if (Units[i].Team == team && Units[i].State != UnitState.Dead && (type < 0 || Units[i].Type == type)) result.Add(i);
            return result;
        }

        /// <summary>Live units of a team and type.</summary>
        public int AliveOf(int team, int type)
        {
            int n = 0;
            foreach (var u in Units) if (u.Team == team && u.Type == type && u.State != UnitState.Dead) n++;
            return n;
        }

        // ------------------------------------------------------------------------------------------------ spawning

        /// <summary>Spawns a unit standing on <paramref name="foot"/> (the surface point under it) facing <paramref name="yaw"/>; -1 when
        /// its pool is full.</summary>
        public int SpawnUnit(int type, int team, float3 foot, float yaw)
        {
            var a = Types[type];
            var pool = UnitPools[type];
            if (pool == null) return -1;
            float3 pos = foot + new float3(0f, a.StandHeight, 0f);
            var drive = GpuBodyDrive.Velocity(float3.zero, a.Force, new float3(1, 0, 1), yaw);
            int body = pool.Spawn(a.BoxSize, a.Density, a.Friction, pos, quaternion.RotateY(yaw), float3.zero, UnitFlags, drive);
            if (body < 0) return -1;
            m_UnitOfBody[body] = Units.Count;
            Units.Add(new Unit
            {
                Body = body, Team = team, Type = type, State = UnitState.Idle, Order = OrderKind.None, TargetBody = -1, Yaw = yaw,
                SpawnStep = World.StepIndex, NextShotStep = World.StepIndex + (body * 37) % math.max(a.CooldownSteps, 1), LastShotStep = -1, PendingShotStep = -1,
            });
            Alive[team]++;
            OnUnitSpawned?.Invoke(body, type);
            return body;
        }

        // ------------------------------------------------------------------------------------------------ orders

        static bool Known(in Unit u, int positionCount, int positionsStep) => u.Body < positionCount && u.SpawnStep < positionsStep;
        bool Known(in Unit u) => m_Positions != null && Known(u, m_PositionCount, m_PositionsStep);

        /// <summary>The centroid of the units' known positions (the ground plane); false when none is known yet.</summary>
        bool Centroid(IReadOnlyList<int> units, out float3 centroid)
        {
            centroid = float3.zero; int n = 0;
            foreach (int i in units)
            {
                var u = Units[i];
                if (u.State == UnitState.Dead || !Known(u)) continue;
                centroid += m_Positions[u.Body].xyz; n++;
            }
            if (n == 0) return false;
            centroid /= n;
            return true;
        }

        /// <summary>Orders the units to walk in a straight line so that the group's centroid lands on <paramref name="point"/>, each
        /// keeping its offset from the centroid (the formation walks as it stands).</summary>
        public void Move(IReadOnlyList<int> units, float3 point)
        {
            if (!Centroid(units, out float3 c)) return;
            foreach (int i in units)
            {
                var u = Units[i];
                if (u.State == UnitState.Dead || !Known(u)) continue;
                float3 offset = m_Positions[u.Body].xyz - c; offset.y = 0f;
                u.Goal = point + offset;
                u.Order = OrderKind.Move; u.State = UnitState.Moving; u.TargetBody = -1; u.PendingShotStep = -1;
                Units[i] = u;
            }
        }

        /// <summary>Orders the units to attack a body: they walk in a straight line to their range of it (the formation's offsets kept
        /// around the line from the centroid to the target) and shoot at it until it is gone.</summary>
        public void Attack(IReadOnlyList<int> units, int targetBody)
        {
            if (targetBody < 0 || m_Positions == null || targetBody >= m_PositionCount || !World.IsAlive(targetBody)) return;
            if (!Centroid(units, out float3 c)) return;
            float3 target = m_Positions[targetBody].xyz;
            float3 n = math.normalizesafe(new float3(target.x - c.x, 0f, target.z - c.z), new float3(0, 0, 1));
            foreach (int i in units)
            {
                var u = Units[i];
                if (u.State == UnitState.Dead || !Known(u)) continue;
                var a = Types[u.Type];
                float3 pos = m_Positions[u.Body].xyz;
                float3 offset = pos - c; offset.y = 0f;
                bool inRange = math.length((target - pos).xz) <= a.Range;   // already in range: shoot from where it stands
                u.Goal = inRange ? pos : target - n * (a.Range * Settings.RangeFraction) + offset;
                u.Order = OrderKind.Attack; u.State = inRange ? UnitState.Attacking : UnitState.Moving; u.TargetBody = targetBody; u.PendingShotStep = -1;
                Units[i] = u;
            }
        }

        /// <summary>Cancels the units' orders (they stand where they are).</summary>
        public void Halt(IReadOnlyList<int> units)
        {
            foreach (int i in units)
            {
                var u = Units[i];
                if (u.State == UnitState.Dead) continue;
                u.Order = OrderKind.None; u.State = UnitState.Idle; u.TargetBody = -1;
                Units[i] = u;
            }
        }

        // ------------------------------------------------------------------------------------------------ per step

        /// <summary>Everything the battle does before a world step, from the latest read-back positions (taken after step
        /// <paramref name="positionsStep"/>; null: nothing moves).</summary>
        public void Tick(float4[] positions, int positionCount, int positionsStep)
        {
            m_Positions = positions; m_PositionCount = positionCount; m_PositionsStep = positionsStep;
            UpdateHits();
            UpdateSpent();
            ApplyImpacts();
            if (positions != null)
            {
                Steer();
                Fire();
                PendingShots();
            }
            RetireDue();
        }

        void UpdateHits()
        {
            var events = World.ReadEvents;
            for (int i = 0; i < Units.Count; i++)
            {
                var u = Units[i];
                if (u.State == UnitState.Dead) continue;
                int hits = GpuBodyEvents.Hits(events[u.Body]);
                if (hits <= u.Hits) continue;
                u.Hits = hits;
                Units[i] = u;
                if (hits >= Types[u.Type].HitPoints) Kill(i);
            }
        }

        /// <summary>A unit dies: its rotation is unlocked and its motor switched off, so it topples; retired after the delay.</summary>
        public void Kill(int unitIndex)
        {
            var u = Units[unitIndex];
            if (u.State == UnitState.Dead) return;
            u.State = UnitState.Dead; u.Order = OrderKind.None; u.PendingShotStep = -1;
            u.DeathStep = World.StepIndex;
            Units[unitIndex] = u;
            World.SetBodyFlags(u.Body, DeadUnitFlags);
            World.SetBodyDrive(u.Body, default);
            Alive[u.Team]--;
            Dead[u.Team]++;
        }

        /// <summary>A projectile that has touched anything (or flown for too long) is spent: its drive goes at once. The event word of
        /// a recycled slot may still be its predecessor's for a couple of frames, hence the guard.</summary>
        void UpdateSpent()
        {
            var events = World.ReadEvents;
            int step = World.StepIndex;
            for (int i = 0; i < Projectiles.Count; i++)
            {
                var p = Projectiles[i];
                if (p.Spent) continue;
                int age = step - p.LaunchStep;
                bool touched = age >= Settings.EventGuardSteps && GpuBodyEvents.Touched(events[p.Body]);
                if (!touched && age <= Settings.ProjectileMaxAge) continue;
                p.SpentStep = step;
                p.HitApplied = !touched || !Types[p.Type].Hit.Any;   // over age: gone without a bang
                if (!touched) p.SpentStep = step - Types[p.Type].ProjectileRetireDelay;
                Projectiles[i] = p;
                if (World.GetBodyDrive(p.Body).Mode != GpuBodyDrive.None) World.SetBodyDrive(p.Body, default);
            }
        }

        /// <summary>The hit effects of the spent projectiles whose position is known: a blast at the projectile, units within the
        /// impact radius killed, the impact reported.</summary>
        void ApplyImpacts()
        {
            for (int i = 0; i < Projectiles.Count; i++)
            {
                var p = Projectiles[i];
                if (!p.Spent || p.HitApplied) continue;
                if (m_Positions == null || p.Body >= m_PositionCount || p.LaunchStep >= m_PositionsStep) continue;   // its position is not read back yet
                var hit = Types[p.Type].Hit;
                float3 centre = m_Positions[p.Body].xyz;
                World.Blast(centre, hit.ImpactRadius, hit.Impulse, hit.Lift, hit.PulverizeRadius, p.Body);
                if (hit.KillUnits && hit.ImpactRadius > 0f)
                    for (int k = 0; k < Units.Count; k++)
                    {
                        var u = Units[k];
                        if (u.State == UnitState.Dead || !Known(u)) continue;
                        if (math.distance(m_Positions[u.Body].xyz, centre) < hit.ImpactRadius) Kill(k);
                    }
                p.HitApplied = true;
                Projectiles[i] = p;
                Impacts++;
                OnImpact?.Invoke(p.Type, centre);
            }
        }

        void Steer()
        {
            for (int i = 0; i < Units.Count; i++)
            {
                var u = Units[i];
                if (u.State == UnitState.Dead || !Known(u)) continue;
                var a = Types[u.Type];
                float3 pos = m_Positions[u.Body].xyz;
                float3 vel = float3.zero;
                if (u.Order == OrderKind.Attack)
                {
                    bool gone = u.TargetBody < 0 || u.TargetBody >= m_PositionCount || !World.IsAlive(u.TargetBody);
                    if (gone) { u.Order = OrderKind.None; u.State = UnitState.Idle; u.TargetBody = -1; }
                    else
                    {
                        float3 target = m_Positions[u.TargetBody].xyz;
                        float3 toTarget = target - pos; toTarget.y = 0f;
                        float distance = math.length(toTarget);
                        if (u.State == UnitState.Attacking && distance > a.Range * 1.05f)
                        {
                            // the target moved out of range (a fallen brick): walk after it along the unit's own line
                            u.Goal = pos + toTarget / math.max(distance, 1e-4f) * (distance - a.Range * Settings.RangeFraction);
                            u.State = UnitState.Moving;
                        }
                        if (u.State == UnitState.Moving)
                        {
                            float3 d = u.Goal - pos; d.y = 0f;
                            float dist = math.length(d);
                            if (dist < Settings.ArriveRadius)
                            {
                                if (distance <= a.Range) u.State = UnitState.Attacking;
                                else u.Goal = pos + toTarget / math.max(distance, 1e-4f) * (distance - a.Range * Settings.RangeFraction);   // the formation's offset left it short: advance
                            }
                            else { vel = d / dist * a.Speed; u.Yaw = YawOf(d); }
                        }
                        if (u.State == UnitState.Attacking && math.lengthsq(toTarget) > 1e-4f) u.Yaw = YawOf(toTarget);
                    }
                }
                else if (u.Order == OrderKind.Move)
                {
                    float3 d = u.Goal - pos; d.y = 0f;
                    float dist = math.length(d);
                    if (dist < Settings.ArriveRadius) { u.Order = OrderKind.None; u.State = UnitState.Idle; }
                    else { vel = d / dist * a.Speed; u.Yaw = YawOf(d); }
                }
                if (u.Order == OrderKind.None && u.HasAim)
                {
                    float3 d = u.Aim - pos; d.y = 0f;
                    if (math.lengthsq(d) > 1e-4f) u.Yaw = YawOf(d);
                }
                Units[i] = u;
                World.SetBodyDrive(u.Body, GpuBodyDrive.Velocity(vel, a.Force, new float3(1, 0, 1), u.Yaw));
            }
        }

        /// <summary>Every live unit whose cooldown is up and that has something to shoot at (its attack target once in range, or the
        /// nearest enemy in range when it engages on its own) starts a shot; the projectile leaves at once or after the archetype's
        /// launch delay. Deterministic: the spread comes from a random stream seeded with the step.</summary>
        void Fire()
        {
            int step = World.StepIndex;
            var rng = new Unity.Mathematics.Random((uint)(step * 7919u + 1u));
            for (int i = 0; i < Units.Count; i++)
            {
                var u = Units[i];
                if (u.State == UnitState.Dead || u.State == UnitState.Moving || !Known(u) || step < u.NextShotStep || u.PendingShotStep >= 0) continue;
                var a = Types[u.Type];
                float3 pos = m_Positions[u.Body].xyz;
                if (!AimPoint(u, pos, out float3 aim)) { u.HasAim = false; Units[i] = u; continue; }
                u.Aim = aim; u.HasAim = true;
                u.NextShotStep = step + math.max(a.CooldownSteps, 1);
                u.LastShotStep = step;
                if (a.LaunchDelaySteps > 0) { u.PendingShotStep = step + a.LaunchDelaySteps; u.PendingAim = aim; }
                Units[i] = u;
                OnShot?.Invoke(i);
                if (a.LaunchDelaySteps <= 0) Shoot(u, pos, aim, ref rng);
            }
        }

        void PendingShots()
        {
            int step = World.StepIndex;
            var rng = new Unity.Mathematics.Random((uint)(step * 104729u + 7u));
            for (int i = 0; i < Units.Count; i++)
            {
                var u = Units[i];
                if (u.PendingShotStep < 0 || step < u.PendingShotStep) continue;
                u.PendingShotStep = -1;
                Units[i] = u;
                if (u.State == UnitState.Dead || !Known(u)) continue;
                float3 aim = u.PendingAim;
                if (u.Order == OrderKind.Attack && u.TargetBody >= 0 && u.TargetBody < m_PositionCount && World.IsAlive(u.TargetBody)) aim = m_Positions[u.TargetBody].xyz;
                Shoot(u, m_Positions[u.Body].xyz, aim, ref rng);
            }
        }

        bool AimPoint(in Unit u, float3 pos, out float3 aim)
        {
            aim = float3.zero;
            var a = Types[u.Type];
            if (u.Order == OrderKind.Attack)
            {
                if (u.State != UnitState.Attacking || u.TargetBody < 0 || u.TargetBody >= m_PositionCount) return false;
                aim = m_Positions[u.TargetBody].xyz;
                return true;
            }
            if (!a.AutoEngage) return false;
            float best = a.Range * a.Range;
            int bestBody = -1;
            foreach (var other in Units)
            {
                if (other.Team == u.Team || other.State == UnitState.Dead || !Known(other)) continue;
                float3 p = m_Positions[other.Body].xyz;
                float d = math.lengthsq((p - pos).xz);
                if (d < best) { best = d; bestBody = other.Body; }
            }
            if (bestBody < 0) return false;
            aim = m_Positions[bestBody].xyz;
            return true;
        }

        /// <summary>The shot: the launch point from the unit's launch local, the velocity by the archetype's trajectory (the two-pass
        /// launch-point refinement: the projectile starts LaunchOffset along its velocity, clear of the shooter's box).</summary>
        bool Shoot(in Unit u, float3 pos, float3 aim, ref Unity.Mathematics.Random rng)
        {
            var a = Types[u.Type];
            float g = math.length(World.Params.Gravity), dt = SubstepDt;
            float3 launch = pos + math.mul(quaternion.RotateY(u.Yaw), a.LaunchLocal);
            float3 from = launch + math.normalizesafe(aim - launch, new float3(0, 0, 1)) * a.LaunchOffset;
            float3 v;
            GpuBodyDrive drive = default;
            switch (a.Trajectory)
            {
                case Trajectory.HighArc:
                case Trajectory.LowArc:
                {
                    bool high = a.Trajectory == Trajectory.HighArc;
                    if (!Ballistics.AtSpeed(from, aim, a.LaunchSpeed, g, high, dt, out v)) v = math.normalizesafe(aim - from, new float3(0, 0, 1)) * a.LaunchSpeed;
                    from = launch + math.normalizesafe(v, new float3(0, 0, 1)) * a.LaunchOffset;
                    if (!Ballistics.AtSpeed(from, aim, a.LaunchSpeed, g, high, dt, out v)) v = math.normalizesafe(aim - from, new float3(0, 0, 1)) * a.LaunchSpeed;
                    v = Spread(v, a.Spread, ref rng);
                    break;
                }
                case Trajectory.Straight:
                {
                    float3 dir = math.normalizesafe(aim - from, new float3(0, 0, 1));
                    v = Spread(dir * a.LaunchSpeed, a.Spread, ref rng);
                    if (a.Thrust > 0f) drive = GpuBodyDrive.ConstantForce(dir * a.Thrust);
                    break;
                }
                case Trajectory.Homing:
                {
                    // straight at the aim and pulled towards it: gravity bends it down, the pull bends it back
                    float3 dir = math.normalizesafe(aim - from, new float3(0, 0, 1));
                    v = dir * a.LaunchSpeed;
                    if (a.Thrust > 0f) drive = GpuBodyDrive.TowardsPoint(aim, a.Thrust);
                    break;
                }
                default:
                {
                    float elevation = math.radians(a.ElevationDeg);
                    if (!ElevationVelocity(from, aim, elevation, a.MaxSpeed, g, dt, out v)) return false;
                    from = launch + math.normalizesafe(v, new float3(0, 0, 1)) * a.LaunchOffset;
                    if (!ElevationVelocity(from, aim, elevation, a.MaxSpeed, g, dt, out v)) return false;
                    v = Spread(v, a.Spread, ref rng);
                    break;
                }
            }
            int variant = a.ProjectileVariants > 1 ? ShotsFired % a.ProjectileVariants : 0;
            return Launch(u.Type, u.Team, variant, from, v, drive) >= 0;
        }

        /// <summary>An arrow at the set elevation, or the lobbed shot at the top speed when the target is above that line; a target out
        /// of range gets the top speed and falls short.</summary>
        static bool ElevationVelocity(float3 from, float3 aim, float elevation, float maxSpeed, float g, float dt, out float3 v)
        {
            if (!Ballistics.AtElevation(from, aim, elevation, g, dt, out v) && !Ballistics.AtSpeed(from, aim, maxSpeed, g, true, dt, out v)) return false;
            float speed = math.length(v);
            if (speed > maxSpeed) v *= maxSpeed / speed;
            return true;
        }

        static float3 Spread(float3 v, float spread, ref Unity.Mathematics.Random rng) => spread > 0f ? v + rng.NextFloat3(-1f, 1f) * (math.length(v) * spread) : v;

        /// <summary>Spawns a projectile of the archetype at <paramref name="pos"/> with the velocity and drive; -1 when the pool is full or
        /// the spot (or either end of the projectile) is inside the castle.</summary>
        public int Launch(int type, int team, int variant, float3 pos, float3 velocity, GpuBodyDrive drive)
        {
            var a = Types[type];
            var pools = ProjectilePools[type];
            if (pools == null || pools.Length == 0) return -1;
            variant = math.clamp(variant, 0, pools.Length - 1);
            var pool = pools[variant];
            if (pool == null) return -1;
            float3 dir = math.normalizesafe(velocity, new float3(0, 0, 1));
            float3 size = a.ProjectileBoxSize;
            float half = size.z * 0.5f;
            if (InsideCastle(pos) || InsideCastle(pos + dir * half) || InsideCastle(pos - dir * half)) return -1;
            uint flags = ProjectileFlags | (drive.Mode != GpuBodyDrive.None ? GpuBodyDef.FlagDriven : 0u) | (a.ProjectileAlign ? GpuBodyDef.FlagAlignVelocity : 0u);
            quaternion rot = a.ProjectileAlign ? quaternion.LookRotationSafe(dir, new float3(0, 1, 0)) : quaternion.identity;
            int body = pool.Spawn(size, a.ProjectileDensity, a.Friction, pos, rot, velocity, flags, drive);
            if (body < 0) return -1;
            Projectiles.Add(new Projectile { Body = body, Type = type, Team = team, Variant = variant, LaunchStep = World.StepIndex, SpentStep = -1 });
            ShotsFired++;
            OnProjectileSpawned?.Invoke(body, type);
            return body;
        }

        // ------------------------------------------------------------------------------------------------ retirement

        /// <summary>Retires the dead units after <see cref="BattleSettings.RetireDelay"/> and the spent projectiles after their
        /// archetype's delay (once their hit effect went off).</summary>
        public int RetireDue() => Retire(false);

        /// <summary>Retires every dead unit and every spent projectile at once.</summary>
        public int Purge() => Retire(true);

        /// <summary>Retires everything, alive or not.</summary>
        public void Clear()
        {
            for (int i = Units.Count - 1; i >= 0; i--)
            {
                var u = Units[i];
                OnRetiring?.Invoke(u.Body);
                UnitPools[u.Type].Retire(u.Body);
                if (u.State != UnitState.Dead) Alive[u.Team]--; else Dead[u.Team]--;
            }
            Units.Clear();
            for (int i = Projectiles.Count - 1; i >= 0; i--)
            {
                var p = Projectiles[i];
                OnRetiring?.Invoke(p.Body);
                ProjectilePools[p.Type][p.Variant].Retire(p.Body);
            }
            Projectiles.Clear();
            m_UnitOfBody.Clear();
        }

        int Retire(bool everything)
        {
            int retired = 0, step = World.StepIndex;
            bool removedUnit = false;
            for (int i = Units.Count - 1; i >= 0; i--)
            {
                var u = Units[i];
                if (u.State != UnitState.Dead || (!everything && step - u.DeathStep < Settings.RetireDelay)) continue;
                OnRetiring?.Invoke(u.Body);
                UnitPools[u.Type].Retire(u.Body);
                Units.RemoveAt(i);
                retired++; removedUnit = true;
            }
            if (removedUnit)
            {
                m_UnitOfBody.Clear();
                for (int i = 0; i < Units.Count; i++) m_UnitOfBody[Units[i].Body] = i;
            }
            for (int i = Projectiles.Count - 1; i >= 0; i--)
            {
                var p = Projectiles[i];
                if (!p.Spent || (!everything && (!p.HitApplied || step - p.SpentStep < Types[p.Type].ProjectileRetireDelay))) continue;
                OnRetiring?.Invoke(p.Body);
                ProjectilePools[p.Type][p.Variant].Retire(p.Body);
                Projectiles.RemoveAt(i);
                retired++;
            }
            Retired += retired;
            return retired;
        }

        public string Summary()
        {
            int moving = 0, attacking = 0, spent = 0;
            foreach (var u in Units) { if (u.State == UnitState.Moving) moving++; else if (u.State == UnitState.Attacking) attacking++; }
            foreach (var p in Projectiles) if (p.Spent) spent++;
            return $"attackers {Alive[Attackers]} alive / {Dead[Attackers]} dead ({moving} moving, {attacking} attacking), defenders {Alive[Defenders]} alive / {Dead[Defenders]} dead; " +
                   $"projectiles {Projectiles.Count - spent} flying, {spent} spent, {ShotsFired} fired, {Impacts} impacts, {Retired} retired";
        }
    }
}
