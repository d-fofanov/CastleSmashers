using Unity.Mathematics;

namespace Phys.AvbdGpu.Siege
{
    /// <summary>How a unit's projectile flies.</summary>
    public enum Trajectory : byte
    {
        /// <summary>A fixed elevation, the speed solved from the range (arrows); a target above the line or out of range gets the top speed.</summary>
        Elevation,
        /// <summary>A fixed speed on the high arc (a trebuchet's rock); straight at the target when out of range.</summary>
        HighArc,
        /// <summary>A fixed speed on the low arc (a cannonball); straight at the target when out of range.</summary>
        LowArc,
        /// <summary>Straight at the target at the launch speed, under a constant thrust along the launch direction (a rocket).</summary>
        Straight,
        /// <summary>Straight at the aim at the launch speed and pulled towards it with the thrust (a spell that homes in).</summary>
        Homing,
    }

    /// <summary>What a projectile does where it lands (solver metres, N s): every joint anchored within the pulverize radius breaks,
    /// every body within the impact radius gets a radial impulse falling off linearly to the radius (biased upward by the lift),
    /// units within the impact radius die when <see cref="KillUnits"/>.</summary>
    public struct HitEffect
    {
        public float PulverizeRadius, ImpactRadius, Impulse, Lift;
        public bool KillUnits;

        public bool Any => PulverizeRadius > 0f || ImpactRadius > 0f;
    }

    /// <summary>A kind of unit in solver units (m, kg, N, steps at 60 Hz): its body, how it moves, how and what it shoots, what its
    /// shots do. Built once per scene from a <see cref="UnitConfig"/> (or by hand in tests); the visuals stay with the config.</summary>
    public sealed class UnitArchetype
    {
        public string Name = "unit";

        // ---- the body
        /// <summary>The collision box (the model's bounds scaled, plus one collision margin in height so that the feet rest on the ground).</summary>
        public float3 BoxSize = new float3(1.77f, 2.41f, 0.6f);
        /// <summary>Where the model's pivot lies in body space (the mesh range's offset).</summary>
        public float3 MeshOffset = new float3(0f, -1.195f, 0f);
        /// <summary>Height of the body centre above the surface the unit stands on.</summary>
        public float StandHeight = 1.195f;
        public float Mass = 1f, Friction = 0.6f;
        /// <summary>Walking speed (m/s) and the motor's force cap (N).</summary>
        public float Speed = 3f, Force = 12f;
        /// <summary>Fast projectile hits that kill the unit.</summary>
        public int HitPoints = 1;

        // ---- engagement
        /// <summary>How far it shoots; an attack order walks it to <see cref="BattleSettings.RangeFraction"/> of it.</summary>
        public float Range = 30f;
        /// <summary>Steps between two shots.</summary>
        public int CooldownSteps = 180;
        /// <summary>Shoots at the nearest enemy unit in range on its own when it has no order (defenders always do).</summary>
        public bool AutoEngage = true;

        // ---- the shot
        public Trajectory Trajectory = Trajectory.Elevation;
        public float ElevationDeg = 55f;
        /// <summary>Launch speed (m/s) for the arc, straight and homing trajectories; the top speed for the elevation one.</summary>
        public float LaunchSpeed = 35f, MaxSpeed = 40f;
        /// <summary>Random velocity scatter as a fraction of the speed (0 = every shot exact).</summary>
        public float Spread = 0.03f;
        /// <summary>Thrust (N) of a straight shot along its direction, or the pull of a homing shot towards its aim.</summary>
        public float Thrust;
        /// <summary>Where the shot leaves, in body space (the shoulder, a trebuchet's arm); the projectile spawns <see cref="LaunchOffset"/>
        /// further along its velocity, clear of the shooter's box.</summary>
        public float3 LaunchLocal = new float3(0f, 0.6f, 0f);
        public float LaunchOffset = 2.6f;
        /// <summary>Steps between the shot's start (the weapon's swing) and the projectile's launch.</summary>
        public int LaunchDelaySteps;

        // ---- the projectile
        public float3 ProjectileBoxSize = new float3(0.17f, 0.185f, 1.4f);
        public float3 ProjectileMeshOffset;
        public float ProjectileMass = 0.02f;
        /// <summary>The projectile keeps its long axis along its velocity (arrows, spells); a rock tumbles.</summary>
        public bool ProjectileAlign = true;
        /// <summary>Visual variants (one body pool and mesh range each; shots cycle through them).</summary>
        public int ProjectileVariants = 1;
        /// <summary>Steps a spent projectile lies about before its slot is freed (0: gone at its impact, an exploding shot).</summary>
        public int ProjectileRetireDelay = 120;
        public HitEffect Hit;

        public float Density => Mass / math.max(BoxSize.x * BoxSize.y * BoxSize.z, 1e-6f);
        public float ProjectileDensity => ProjectileMass / math.max(ProjectileBoxSize.x * ProjectileBoxSize.y * ProjectileBoxSize.z, 1e-6f);

        /// <summary>A unit archetype from a model's bounds (model metres, the pivot at the origin) at the given scale, the box grown by
        /// one collision margin in height, the mesh offset placing the pivot so that the model's feet rest on the ground.</summary>
        public static void BoxOf(float3 boundsCentre, float3 boundsSize, float scale, float margin, out float3 box, out float3 meshOffset)
        {
            box = boundsSize * scale + new float3(0f, margin, 0f);
            meshOffset = -boundsCentre * scale + new float3(0f, margin * 0.5f, 0f);
        }
    }
}
