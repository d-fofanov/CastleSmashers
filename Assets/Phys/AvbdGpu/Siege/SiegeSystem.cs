// Units and projectiles on the GPU solver: figures that march, hold a line and fire volleys, arrows / cannonballs / rockets
// / homing bolts that fly, hit and lie about until the next purge. Everything is an ordinary box body of the solver; the
// CPU side only steers (drive uploads), fires (batched spawns), reads the contact events back and purges in batches.

using System;
using System.Collections.Generic;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Siege
{
    public enum UnitKind : byte { Archer, Gunner, Rocketeer, Mage }
    public enum UnitState : byte { Marching, Holding, Dead }
    public enum ProjectileKind : byte { Arrow, Cannonball, Rocket, Bolt }

    public struct Unit
    {
        public int Body;
        public int Team;              // 0 attackers, 1 defenders
        public UnitKind Kind;
        public UnitState State;
        public float3 Waypoint;       // where a marching unit walks to
        public float Yaw;
        public int Hits;              // impacts read back so far
        public int SpawnStep;         // the step the unit was spawned at: read-back positions of earlier steps do not hold it
        public int DeathStep;
        public float3 Aim;            // last aim point (holding units face it)
        public bool HasAim;
    }

    public struct Projectile
    {
        public int Body;
        public ProjectileKind Kind;
        public int Team;
        public int LaunchStep;
    }

    /// <summary>Tunables of the siege (solver metres, seconds as steps at 60 Hz). Defaults suit the castle at brick scale 5.</summary>
    [Serializable]
    public struct SiegeSettings
    {
        public float UnitSpeed, UnitForce, ArriveRadius;
        public int VolleyInterval, PurgeInterval, ProjectileMaxAge, HitPoints;
        public float ArrowElevationDeg, ArrowMaxSpeed, CannonSpeed, RocketSpeed, RocketThrust, BoltSpeed, BoltForce;
        public float Range, Spread, LaunchOffset;
        public float AttackDistance, MarchDistance, RankSpacing, ColumnSpacing;
        public int ArchersPerRank, Ranks, Defenders;

        public static SiegeSettings Default => new SiegeSettings
        {
            UnitSpeed = 3f, UnitForce = 12f, ArriveRadius = 0.5f,
            VolleyInterval = 240, PurgeInterval = 300, ProjectileMaxAge = 1800, HitPoints = 1,
            ArrowElevationDeg = 55f, ArrowMaxSpeed = 40f, CannonSpeed = 35f, RocketSpeed = 20f, RocketThrust = 0.3f, BoltSpeed = 12f, BoltForce = 3f,
            Range = 60f, Spread = 0.03f, LaunchOffset = 2.6f,
            AttackDistance = 14f, MarchDistance = 16f, RankSpacing = 2.5f, ColumnSpacing = 2.4f,
            ArchersPerRank = 12, Ranks = 2, Defenders = 24,
        };
    }

    /// <summary>Two armies around a brick castle: attackers outside its walls, defenders inside. Call <see cref="Tick"/> once
    /// before every world step with the latest read-back positions.</summary>
    public sealed class SiegeSystem
    {
        public const int Attackers = 0, Defenders = 1;
        const uint UnitFlags = GpuBodyDef.FlagReportEvents | GpuBodyDef.FlagDriven | GpuBodyDef.FlagHeading | (GpuBodyDef.KindUnit << GpuBodyDef.KindShift);
        const uint DeadUnitFlags = GpuBodyDef.FlagReportEvents | (GpuBodyDef.KindUnit << GpuBodyDef.KindShift);
        const uint ProjectileFlags = GpuBodyDef.FlagReportEvents | (GpuBodyDef.KindProjectile << GpuBodyDef.KindShift);

        public readonly AvbdGpuWorld World;
        public readonly SiegeSpec Spec;
        public SiegeSettings Settings;
        /// <summary>Units, arrow-shaped projectiles (arrows, rockets: drawn with the arrow model) and cube projectiles
        /// (cannonballs, bolts: drawn as their boxes); each pool is one contiguous body range.</summary>
        public readonly BodyPool Units, Arrows, Shots;
        public readonly List<Unit> UnitList = new List<Unit>();
        public readonly List<Projectile> ProjectileList = new List<Projectile>();
        /// <summary>Called with the body index and a colour whenever a unit or projectile is spawned (tints).</summary>
        public Action<int, Color32> OnSpawned;
        /// <summary>Called with the body index before a unit or projectile is retired (release joints on it).</summary>
        public Action<int> OnRetiring;
        public bool AutoVolleys = true, AutoPurges = true;
        public int Volleys, Purges, ShotsFired;
        public readonly int[] Alive = new int[2], Dead = new int[2];

        // the castle's outer wall faces (xz) and the crest height, for aiming at the walls, and its layout, so that no
        // projectile is spawned inside a brick; unset until SpawnArmies
        float2 m_WallMin, m_WallMax;
        float m_WallTop;
        bool m_HasCastle;
        BrickLayout m_Layout;
        BrickSpec m_Brick;
        float3 m_Gravity => World.Params.Gravity;

        public SiegeSystem(AvbdGpuWorld world, SiegeSpec spec, int unitCapacity, int arrowCapacity, int shotCapacity, SiegeSettings? settings = null)
        {
            World = world;
            Spec = spec;
            Settings = settings ?? SiegeSettings.Default;
            Units = new BodyPool(world, unitCapacity);
            Arrows = new BodyPool(world, arrowCapacity);
            Shots = new BodyPool(world, shotCapacity);
        }

        public static bool IsArrowShaped(ProjectileKind kind) => kind == ProjectileKind.Arrow || kind == ProjectileKind.Rocket;
        BodyPool PoolOf(ProjectileKind kind) => IsArrowShaped(kind) ? Arrows : Shots;

        /// <summary>Copies the pools' event words from the GPU synchronously (tests; the demo reads them back asynchronously).</summary>
        public void SyncEvents() => World.ReadEventsSync();

        public int LiveProjectiles => ProjectileList.Count;
        public int NextVolleyIn => Settings.VolleyInterval - World.StepIndex % Settings.VolleyInterval;
        public int NextPurgeIn => Settings.PurgeInterval - World.StepIndex % Settings.PurgeInterval;

        // ------------------------------------------------------------------------------------------------ armies

        /// <summary>Attackers in ranks outside every wall (marching in from further out), defenders on free ground inside.</summary>
        public void SpawnArmies(BrickLayout layout, CastlePlan plan, BrickSpec brick)
        {
            int S = plan.Side, face = BrickCastle.TowerOut;
            float3 lo = brick.GridPoint(face, face, 0), hi = brick.GridPoint(S - face, S - face, 0);
            m_WallMin = lo.xz; m_WallMax = hi.xz;
            m_WallTop = (plan.WallCourses + 0.5f) * Brick.BodyHeight * brick.Scale;
            m_HasCastle = true;
            m_Layout = layout; m_Brick = brick;

            // attackers: for every side a block of ranks centred on the wall, facing it, marching in from further out
            float3 centre = (lo + hi) * 0.5f;
            var sides = new[]
            {
                (outward: new float3(0, 0, -1), along: new float3(1, 0, 0), gate: true),
                (outward: new float3(0, 0, 1), along: new float3(1, 0, 0), gate: false),
                (outward: new float3(-1, 0, 0), along: new float3(0, 0, 1), gate: false),
                (outward: new float3(1, 0, 0), along: new float3(0, 0, 1), gate: false),
            };
            foreach (var side in sides)
            {
                float3 axis = math.abs(side.outward);
                float faceAt = math.dot(axis, math.dot(side.outward, axis) > 0 ? hi : lo);   // the outer wall face on the outward axis
                float mid = math.dot(side.along, centre);
                for (int rank = 0; rank < Settings.Ranks; rank++)
                    for (int c = 0; c < Settings.ArchersPerRank; c++)
                    {
                        float alongAt = mid + (c - (Settings.ArchersPerRank - 1) * 0.5f) * Settings.ColumnSpacing;
                        float3 target = side.along * alongAt + axis * faceAt + side.outward * (Settings.AttackDistance + rank * Settings.RankSpacing);
                        float3 start = target + side.outward * Settings.MarchDistance;
                        var kind = UnitKind.Archer;
                        if (side.gate && rank == 0)
                        {
                            int half = Settings.ArchersPerRank / 2;
                            if (c == half - 1 || c == half) kind = UnitKind.Gunner;
                            else if (c % 3 == 0) kind = UnitKind.Rocketeer;
                            else if (c % 3 == 1) kind = UnitKind.Mage;
                        }
                        SpawnUnit(kind, Attackers, start, target, YawOf(-side.outward));
                    }
            }

            // defenders: free 4 x 2 stud footprints a few studs inside every inner wall face, spread over the candidates
            var spots = DefenderSpots(layout, plan, brick);
            int take = math.min(Settings.Defenders, spots.Count);
            for (int i = 0; i < take; i++)
            {
                var (p, yaw) = spots[(int)((long)i * spots.Count / math.max(take, 1))];
                SpawnUnit(UnitKind.Archer, Defenders, p, p, yaw, holding: true);
            }
        }

        /// <summary>Positions (box centre on the ground) and outward yaws of free footprints just inside the four walls, in
        /// the layout's stud grid (the gatehouse, stairs, mid towers and the keep are avoided through the occupancy map).</summary>
        public List<(float3 position, float yaw)> DefenderSpots(BrickLayout layout, CastlePlan plan, BrickSpec brick)
        {
            int S = plan.Side, T = plan.TowerSize, face = BrickCastle.TowerOut, depth = BrickCastle.WallDepth * plan.WallLayers;
            int inner = face + depth;                 // first free stud inside a wall
            const int inset = 4;                      // studs from the inner face to the figure's centre
            var spots = new List<(float3, float)>();
            for (int a = T + 2; a <= S - T - 2; a += 4)
            {
                // (cx, cz, half extents of the footprint in studs, outward yaw)
                foreach (var (cx, cz, hw, hd, yaw) in new[]
                {
                    (a, inner + inset, 2, 1, math.PI),            // inside the front wall, facing -z
                    (a, S - inner - inset, 2, 1, 0f),             // back wall, facing +z
                    (inner + inset, a, 1, 2, -math.PI * 0.5f),    // west wall, facing -x
                    (S - inner - inset, a, 1, 2, math.PI * 0.5f), // east wall, facing +x
                })
                {
                    if (!Free(layout, cx, cz, hw, hd)) continue;
                    float3 p = brick.GridPoint(cx, cz, 0);
                    p.y = Spec.UnitStandHeight;
                    spots.Add((p, yaw));
                }
            }
            return spots;
        }

        static bool Free(BrickLayout layout, int cx, int cz, int hw, int hd)
        {
            for (int x = cx - hw; x < cx + hw; x++)
                for (int z = cz - hd; z < cz + hd; z++)
                    for (int layer = 0; layer < 4; layer++)
                        if (layout.BrickAt(x, z, layer) >= 0) return false;
            return true;
        }

        public static float YawOf(float3 forward) => math.atan2(forward.x, forward.z);

        /// <summary>True when a castle brick occupies the stud cell and course of the point (as laid out; fallen bricks are not tracked).</summary>
        public bool InsideBrick(float3 p)
        {
            if (m_Layout == null) return false;
            float studs = Brick.Pitch * m_Brick.Scale;
            float3 rel = p - m_Brick.Origin;
            int course = (int)math.floor(rel.y / (Brick.BodyHeight * m_Brick.Scale));
            if (course < 0) return false;
            return m_Layout.BrickAt((int)math.floor(rel.x / studs), (int)math.floor(rel.z / studs), course) >= 0;
        }

        /// <summary>A projectile of the given half length along <paramref name="dir"/> may be spawned at <paramref name="pos"/>.</summary>
        bool ClearToSpawn(float3 pos, float3 dir, float halfLength)
        {
            return !InsideBrick(pos) && !InsideBrick(pos + dir * halfLength) && !InsideBrick(pos - dir * halfLength);
        }

        public int SpawnUnit(UnitKind kind, int team, float3 start, float3 waypoint, float yaw, bool holding = false)
        {
            start.y = Spec.UnitStandHeight;
            waypoint.y = Spec.UnitStandHeight;
            var drive = GpuBodyDrive.Velocity(float3.zero, Settings.UnitForce, new float3(1, 0, 1), yaw);
            int body = Units.Spawn(Spec.UnitBoxSize, Spec.UnitDensity, Spec.Friction, start, quaternion.RotateY(yaw), float3.zero, UnitFlags, drive);
            if (body < 0) return -1;
            UnitList.Add(new Unit { Body = body, Team = team, Kind = kind, State = holding ? UnitState.Holding : UnitState.Marching, Waypoint = waypoint, Yaw = yaw, SpawnStep = World.StepIndex });
            Alive[team]++;
            OnSpawned?.Invoke(body, UnitColor(kind, team));
            return body;
        }

        public static Color32 UnitColor(UnitKind kind, int team)
        {
            if (team == Defenders) return new Color32(70, 110, 200, 255);
            switch (kind)
            {
                case UnitKind.Gunner: return new Color32(120, 40, 40, 255);
                case UnitKind.Rocketeer: return new Color32(230, 130, 40, 255);
                case UnitKind.Mage: return new Color32(140, 70, 190, 255);
                default: return new Color32(200, 60, 50, 255);
            }
        }

        public static Color32 ProjectileColor(ProjectileKind kind)
        {
            switch (kind)
            {
                case ProjectileKind.Cannonball: return new Color32(70, 72, 78, 255);
                case ProjectileKind.Rocket: return new Color32(240, 120, 40, 255);
                case ProjectileKind.Bolt: return new Color32(170, 90, 230, 255);
                default: return new Color32(222, 200, 150, 255);
            }
        }

        /// <summary>Retires every unit and projectile.</summary>
        public void ClearArmies()
        {
            foreach (var u in UnitList) { OnRetiring?.Invoke(u.Body); Units.Retire(u.Body); }
            foreach (var p in ProjectileList) { OnRetiring?.Invoke(p.Body); PoolOf(p.Kind).Retire(p.Body); }
            UnitList.Clear();
            ProjectileList.Clear();
            Alive[0] = Alive[1] = Dead[0] = Dead[1] = 0;
        }

        // ------------------------------------------------------------------------------------------------ per step

        /// <summary>Hits, steering, the scheduled volley and purge. <paramref name="positions"/> are the latest read-back body
        /// positions, taken after step <paramref name="positionsStep"/> (null: units keep their drives); units spawned at or after
        /// that step are not in them yet and wait.</summary>
        public void Tick(float4[] positions, int positionCount, int positionsStep)
        {
            UpdateHits();
            Steer(positions, positionCount, positionsStep);
            int step = World.StepIndex;
            if (AutoVolleys && step > 0 && step % Settings.VolleyInterval == 0) Volley(positions, positionCount, positionsStep);
            if (AutoPurges && step > 0 && step % Settings.PurgeInterval == 0) Purge();
        }

        static bool Known(in Unit u, int positionCount, int positionsStep) => u.Body < positionCount && u.SpawnStep < positionsStep;

        void UpdateHits()
        {
            var events = World.ReadEvents;
            for (int i = 0; i < UnitList.Count; i++)
            {
                var u = UnitList[i];
                if (u.State == UnitState.Dead) continue;
                int hits = GpuBodyEvents.Hits(events[u.Body]);
                if (hits <= u.Hits) continue;
                u.Hits = hits;
                UnitList[i] = u;
                if (hits >= Settings.HitPoints) Kill(i);
            }
        }

        /// <summary>A unit dies: its rotation is unlocked and its motor switched off, so it topples; retired at the next purge.</summary>
        public void Kill(int unitIndex)
        {
            var u = UnitList[unitIndex];
            if (u.State == UnitState.Dead) return;
            u.State = UnitState.Dead;
            u.DeathStep = World.StepIndex;
            UnitList[unitIndex] = u;
            World.SetBodyFlags(u.Body, DeadUnitFlags);
            World.SetBodyDrive(u.Body, default);
            Alive[u.Team]--;
            Dead[u.Team]++;
        }

        void Steer(float4[] positions, int positionCount, int positionsStep)
        {
            if (positions == null) return;
            for (int i = 0; i < UnitList.Count; i++)
            {
                var u = UnitList[i];
                if (u.State == UnitState.Dead || !Known(u, positionCount, positionsStep)) continue;
                float3 pos = positions[u.Body].xyz;
                float3 vel = float3.zero;
                if (u.State == UnitState.Marching)
                {
                    float3 d = u.Waypoint - pos; d.y = 0;
                    float dist = math.length(d);
                    if (dist < Settings.ArriveRadius) u.State = UnitState.Holding;
                    else { vel = d / dist * Settings.UnitSpeed; u.Yaw = YawOf(d); }
                }
                if (u.State == UnitState.Holding && u.HasAim)
                {
                    float3 d = u.Aim - pos; d.y = 0;
                    if (math.lengthsq(d) > 1e-4f) u.Yaw = YawOf(d);
                }
                UnitList[i] = u;
                World.SetBodyDrive(u.Body, GpuBodyDrive.Velocity(vel, Settings.UnitForce, new float3(1, 0, 1), u.Yaw));
            }
        }

        // ------------------------------------------------------------------------------------------------ volleys

        /// <summary>Every holding unit with something to aim at fires once; all the spawns go up in one batch.</summary>
        public int Volley(float4[] positions, int positionCount, int positionsStep)
        {
            if (positions == null) return 0;
            int fired = 0;
            var rng = new Unity.Mathematics.Random((uint)(World.StepIndex * 7919u + 1u));
            for (int i = 0; i < UnitList.Count; i++)
            {
                var u = UnitList[i];
                if (u.State != UnitState.Holding || !Known(u, positionCount, positionsStep)) continue;
                float3 pos = positions[u.Body].xyz;
                if (!AimPoint(u, pos, positions, positionCount, positionsStep, out float3 aim)) { u.HasAim = false; UnitList[i] = u; continue; }
                u.Aim = aim; u.HasAim = true;
                UnitList[i] = u;
                if (Fire(u, pos, aim, ref rng)) fired++;
            }
            Volleys++;
            ShotsFired += fired;
            return fired;
        }

        bool AimPoint(in Unit u, float3 pos, float4[] positions, int positionCount, int positionsStep, out float3 aim)
        {
            aim = float3.zero;
            bool wantsWall = u.Kind == UnitKind.Gunner;
            if (!wantsWall)
            {
                float best = Settings.Range * Settings.Range;
                int bestBody = -1;
                foreach (var other in UnitList)
                {
                    if (other.Team == u.Team || other.State == UnitState.Dead || !Known(other, positionCount, positionsStep)) continue;
                    float3 p = positions[other.Body].xyz;
                    float d = math.lengthsq((p - pos).xz);
                    if (d < best) { best = d; bestBody = other.Body; }
                }
                if (bestBody >= 0) { aim = positions[bestBody].xyz; return true; }
            }
            if (u.Team != Attackers || !m_HasCastle) return false;
            // the nearest point of the outer wall square, at the crest (gunners aim low to smash the wall)
            float2 q = math.clamp(pos.xz, m_WallMin, m_WallMax);
            aim = new float3(q.x, wantsWall ? m_WallTop * 0.4f : m_WallTop, q.y);
            return true;
        }

        float SubstepDt => World.Params.Dt / math.max(1, World.Params.Substeps);

        bool Fire(in Unit u, float3 pos, float3 aim, ref Unity.Mathematics.Random rng)
        {
            float g = math.length(m_Gravity), dt = SubstepDt;
            float3 shoulder = pos + new float3(0, Spec.UnitBoxSize.y * 0.25f, 0);
            // the projectile starts LaunchOffset along its velocity, clear of the shooter's box; solve from there (two passes)
            float3 from = shoulder + math.normalize(aim - shoulder) * Settings.LaunchOffset;
            float3 v;
            switch (u.Kind)
            {
                case UnitKind.Gunner:
                    if (!Ballistics.AtSpeed(from, aim, Settings.CannonSpeed, g, false, dt, out v)) v = math.normalize(aim - from) * Settings.CannonSpeed;
                    from = shoulder + math.normalize(v) * Settings.LaunchOffset;
                    if (!Ballistics.AtSpeed(from, aim, Settings.CannonSpeed, g, false, dt, out v)) v = math.normalize(aim - from) * Settings.CannonSpeed;
                    return Launch(ProjectileKind.Cannonball, u.Team, from, Spread(v, ref rng), default) >= 0;
                case UnitKind.Rocketeer:
                {
                    float3 dir = math.normalize(aim - from);
                    v = dir * Settings.RocketSpeed;
                    return Launch(ProjectileKind.Rocket, u.Team, from, Spread(v, ref rng), GpuBodyDrive.ConstantForce(dir * Settings.RocketThrust)) >= 0;
                }
                case UnitKind.Mage:
                {
                    float3 dir = math.normalize(aim - from);
                    v = math.normalize(dir + new float3(0, 1, 0)) * Settings.BoltSpeed;
                    from = shoulder + math.normalize(v) * Settings.LaunchOffset;
                    return Launch(ProjectileKind.Bolt, u.Team, from, v, GpuBodyDrive.TowardsPoint(aim, Settings.BoltForce)) >= 0;
                }
                default:
                {
                    float elevation = math.radians(Settings.ArrowElevationDeg);
                    if (!ArrowVelocity(from, aim, elevation, g, dt, out v)) return false;
                    from = shoulder + math.normalize(v) * Settings.LaunchOffset;
                    if (!ArrowVelocity(from, aim, elevation, g, dt, out v)) return false;
                    return Launch(ProjectileKind.Arrow, u.Team, from, Spread(v, ref rng), default) >= 0;
                }
            }
        }

        /// <summary>An arrow at the set elevation, or the lobbed shot at the top speed when the target is above that line;
        /// a target out of range gets the top speed and falls short.</summary>
        bool ArrowVelocity(float3 from, float3 aim, float elevation, float g, float dt, out float3 v)
        {
            if (!Ballistics.AtElevation(from, aim, elevation, g, dt, out v) && !Ballistics.AtSpeed(from, aim, Settings.ArrowMaxSpeed, g, true, dt, out v)) return false;
            float speed = math.length(v);
            if (speed > Settings.ArrowMaxSpeed) v *= Settings.ArrowMaxSpeed / speed;
            return true;
        }

        float3 Spread(float3 v, ref Unity.Mathematics.Random rng) => v + rng.NextFloat3(-1f, 1f) * (math.length(v) * Settings.Spread);

        /// <summary>Spawns a projectile at <paramref name="pos"/> (which must be clear of the shooter's box); returns the body,
        /// -1 when the pool is full or the spot is inside a castle brick. <paramref name="mass"/> overrides the spec's mass.</summary>
        public int Launch(ProjectileKind kind, int team, float3 pos, float3 velocity, GpuBodyDrive drive, float mass = 0f)
        {
            float3 dir = math.normalizesafe(velocity, new float3(0, 0, 1));
            quaternion rot = quaternion.LookRotationSafe(dir, new float3(0, 1, 0));
            uint flags = ProjectileFlags | (drive.Mode != GpuBodyDrive.None ? GpuBodyDef.FlagDriven : 0u);
            float3 size = kind == ProjectileKind.Cannonball ? Spec.BallSize : kind == ProjectileKind.Bolt ? Spec.BoltSize : Spec.ArrowBoxSize;
            if (!ClearToSpawn(pos, dir, size.z * 0.5f)) return -1;
            float density = mass > 0f ? mass / (size.x * size.y * size.z) : 0f;
            int body;
            switch (kind)
            {
                case ProjectileKind.Cannonball:
                    body = Shots.Spawn(size, density > 0f ? density : Spec.BallDensity, Spec.Friction, pos, quaternion.identity, velocity, flags, drive);
                    break;
                case ProjectileKind.Bolt:
                    body = Shots.Spawn(size, density > 0f ? density : Spec.BoltDensity, Spec.Friction, pos, quaternion.identity, velocity, flags, drive);
                    break;
                default:
                    body = Arrows.Spawn(size, density > 0f ? density : Spec.ArrowDensity, Spec.Friction, pos, rot, velocity, flags | GpuBodyDef.FlagAlignVelocity, drive);
                    break;
            }
            if (body < 0) return -1;
            ProjectileList.Add(new Projectile { Body = body, Kind = kind, Team = team, LaunchStep = World.StepIndex });
            OnSpawned?.Invoke(body, ProjectileColor(kind));
            return body;
        }

        // ------------------------------------------------------------------------------------------------ purge

        /// <summary>Retires every dead unit and every spent projectile (one that touched anything, or older than
        /// <see cref="SiegeSettings.ProjectileMaxAge"/>) in one batch.</summary>
        public int Purge()
        {
            int retired = 0;
            var events = World.ReadEvents;
            for (int i = UnitList.Count - 1; i >= 0; i--)
            {
                var u = UnitList[i];
                if (u.State != UnitState.Dead) continue;
                OnRetiring?.Invoke(u.Body);
                Units.Retire(u.Body);
                UnitList.RemoveAt(i);
                retired++;
            }
            for (int i = ProjectileList.Count - 1; i >= 0; i--)
            {
                var p = ProjectileList[i];
                bool spent = GpuBodyEvents.Touched(events[p.Body]) || World.StepIndex - p.LaunchStep > Settings.ProjectileMaxAge;
                if (!spent) continue;
                OnRetiring?.Invoke(p.Body);
                PoolOf(p.Kind).Retire(p.Body);
                ProjectileList.RemoveAt(i);
                retired++;
            }
            Purges++;
            return retired;
        }

        public string Summary()
        {
            int marching = 0;
            foreach (var u in UnitList) if (u.State == UnitState.Marching) marching++;
            return $"attackers {Alive[Attackers]} alive / {Dead[Attackers]} dead ({marching} marching), defenders {Alive[Defenders]} alive / {Dead[Defenders]} dead; " +
                   $"projectiles {ProjectileList.Count} live, {ShotsFired} fired in {Volleys} volleys; next volley {NextVolleyIn / 60f:F1} s, purge {NextPurgeIn / 60f:F1} s; " +
                   $"pools {Units.Alive}/{Units.Capacity} units, {Arrows.Alive}/{Arrows.Capacity} arrows, {Shots.Alive}/{Shots.Capacity} shots";
        }
    }
}
