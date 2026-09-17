using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace Phys.AvbdGpu
{
    // Structured-buffer records mirrored by AvbdCommon.hlsl (4-byte packing, sizes multiples of 16 bytes).

    [StructLayout(LayoutKind.Sequential)]
    public struct GpuBodyDef
    {
        public float3 Size;      // full widths
        public float Mass;       // <= 0: static
        public float3 Moment;    // body-frame diagonal inertia
        public float Friction;
        public float Radius;
        public uint Flags;       // Flag* bits and the Kind field
        public float Pad0, Pad1;
        public const int Stride = 48;
        public const uint FlagStatic = 1;
        /// <summary>The angular degrees of freedom are frozen: 3x3 primal solve, no angular velocity.</summary>
        public const uint FlagLockRotation = 2;
        /// <summary>Locked rotation set to the drive's yaw about +y at the start of every step (units).</summary>
        public const uint FlagHeading = 4;
        /// <summary>Locked rotation set to point the body's +z along its velocity while it moves faster than 1 m/s (arrows).</summary>
        public const uint FlagAlignVelocity = 8;
        /// <summary>Retired slot: no collisions, no update, not drawn; reused by a later spawn.</summary>
        public const uint FlagDead = 16;
        /// <summary>The narrowphase records the kinds of bodies this one touches (see <see cref="GpuBodyEvents"/>).</summary>
        public const uint FlagReportEvents = 32;
        /// <summary>The body has a <see cref="GpuBodyDrive"/> record.</summary>
        public const uint FlagDriven = 64;
        /// <summary>The terrain slot: a static body at the identity pose that collides through the world's heightfield (in no grid
        /// cell, drawn as nothing); the partner of every terrain manifold.</summary>
        public const uint FlagTerrain = 128;
        public const int KindShift = 8;
        public const uint KindMask = 3u << KindShift;
        public const uint KindPlain = 0, KindUnit = 1, KindProjectile = 2;
        public static uint Kind(uint kind) => (kind << KindShift) & KindMask;
        public bool IsStatic => Mass <= 0f || IsDead;
        public bool IsDead => (Flags & FlagDead) != 0;
        public bool IsTerrain => (Flags & FlagTerrain) != 0;
        public uint KindOf => (Flags & KindMask) >> KindShift;
    }

    /// <summary>External drive of a body (<see cref="GpuBodyDef.FlagDriven"/>): an acceleration of its inertial pose like gravity.
    /// The motor is a proportional controller with gain m / dt (capped), so under a steady load F it runs F dt / m below its target.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GpuBodyDrive
    {
        /// <summary>Force (N) for <see cref="Force"/>, the point for <see cref="ToPoint"/>, target velocity (m/s) for <see cref="Motor"/>.</summary>
        public float3 Target;
        public uint Mode;
        /// <summary>Motor: the axes it acts on (1 / 0).</summary>
        public float3 Mask;
        /// <summary>ToPoint: force magnitude (N); Motor: force cap (N).</summary>
        public float Limit;
        /// <summary>Heading about +y (rad) for <see cref="GpuBodyDef.FlagHeading"/> bodies.</summary>
        public float Yaw;
        public float Pad0, Pad1, Pad2;
        public const int Stride = 48;
        public const uint None = 0, Force = 1, ToPoint = 2, Motor = 3;

        public static GpuBodyDrive ConstantForce(float3 force) => new GpuBodyDrive { Mode = Force, Target = force };
        public static GpuBodyDrive TowardsPoint(float3 point, float magnitude) => new GpuBodyDrive { Mode = ToPoint, Target = point, Limit = magnitude };
        public static GpuBodyDrive Velocity(float3 target, float maxForce, float3 mask, float yaw = 0f) =>
            new GpuBodyDrive { Mode = Motor, Target = target, Limit = maxForce, Mask = mask, Yaw = yaw };
    }

    /// <summary>Decoding of a body's event word (sticky until the slot is respawned).</summary>
    public static class GpuBodyEvents
    {
        public const uint TouchStatic = 1, TouchBody = 2, TouchUnit = 4, TouchProjectile = 8;
        public const int HitsShift = 8;
        public static bool Touched(uint e) => (e & 0xFFu) != 0;
        /// <summary>Number of manifolds with projectiles moving faster than 2 m/s that appeared (impacts).</summary>
        public static int Hits(uint e) => (int)(e >> HitsShift);
    }

    /// <summary>A body spawned into a retired slot (SpawnBodies kernel); the definition and drive are uploaded separately.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GpuSpawnRecord
    {
        public uint Slot, Pad0, Pad1, Pad2;
        public float4 Pos, Rot, Vel;
        public const int Stride = 64;
    }

    /// <summary>An explosion applied between steps (BlastBodies / BlastJoints kernels, <see cref="AvbdGpuWorld.Blast"/>): bodies
    /// within <see cref="ImpactRadius"/> get a radial velocity change of Impulse (1 - d / R) / mass biased upward by
    /// <see cref="Lift"/>, joints anchored within <see cref="PulverizeRadius"/> break, everything touched wakes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GpuBlastRecord
    {
        public float3 Centre;
        public float ImpactRadius;
        public float Impulse, Lift, PulverizeRadius;
        public uint Exclude;            // a body left alone (the projectile that caused it); 0xFFFFFFFF = none
        public const int Stride = 32;
        public const uint NoBody = 0xFFFFFFFFu;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GpuManifold
    {
        public uint BodyA, BodyB, NumContacts, ContactStart;
        public float Friction;
        public float3 Normal;    // basis row 0, pointing from B to A
        public float3 Tangent1;
        public float3 Tangent2;
        public float2 Pad;
        public const int Stride = 64;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GpuContact
    {
        public float3 RA, RB, C0, Penalty, Lambda;
        public uint Feature;     // bit 31 = stick
        public const int Stride = 64;
        public bool Stick => (Feature & 0x80000000u) != 0;
        public uint FeatureKey => Feature & 0x7FFFFFFFu;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GpuJointDef
    {
        public int BodyA, BodyB;  // BodyA = -1: RA is a world anchor
        public float3 RA, RB;
        public float StiffnessLin, StiffnessAng, Fracture, TorqueArm;
        /// <summary>Snap fracture (each <see cref="AvbdGpuConstants.HardStiffness"/> = off): the linear multiplier's pull along the snap axis
        /// (tension), its part across the axis (shear) and the anchor separation each break the joint past their limit.</summary>
        public float FractureLateral, FractureTension, BreakDistance;
        /// <summary>The snap axis in body B's frame: +-1 x, +-2 y, +-3 z, the direction in which B separates from A (0 = +y).</summary>
        public float SnapAxis;
        public const int Stride = 64;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GpuJointState
    {
        public float3 C0Lin, C0Ang, PenaltyLin, PenaltyAng, LambdaLin, LambdaAng;
        public uint Broken;
        public float Pad;
        public const int Stride = 80;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GpuSpringDef
    {
        public int BodyA, BodyB;
        public float3 RA, RB;
        public float Stiffness, Rest;
        public float2 Pad;
        public const int Stride = 48;
    }

    /// <summary>Mirror of cbuffer AvbdParams (HLSL packing: 13 float4 registers).</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GpuParams
    {
        public float Dt, Alpha, BetaLin, BetaAng;
        public float Gamma, LambdaDecay, CellSize, InvCellSize;
        public float3 Gravity; public float GravityMag;
        public float3 GravityDir; public uint BodyCount;
        public uint JointCount, SpringCount, ActiveColors, RotatedInertia;
        public uint HashMask, CellMask, MaxPairs, MaxManifolds;
        public uint MaxContacts, MaxCellEntries, MaxLargeBodies, LargeBodyCells;
        /// <summary>SleepSteps: steps at rest before an island sleeps (0 = sleeping off); WakeCount: entries of the wake list.</summary>
        public uint ColorRounds, Substep, SleepSteps, WakeCount;
        // the terrain (AvbdTerrain.hlsl): sample (0, 0), spacing, resolution, max-mip size, body slot (NoTerrain: none)
        public float2 TerrainOrigin, TerrainCell, TerrainInvCell;
        public uint TerrainResX, TerrainResZ, TerrainMipX, TerrainMipZ, TerrainSlot;
        public float TerrainMaxHeight;
        // sleeping: the rest thresholds squared, the relabel flag of this step, the squared speed above which a touch wakes an island
        public float SleepDistSq, SleepAngleSq;
        public uint Relabel;
        public float WakeSpeedSq;
        /// <summary>Steps an island woken by a fast touch stays awake at least.</summary>
        public uint WakeHoldSteps;
        /// <summary>The split capacities: the pools and the hot grid are sized for MaxActive awake bodies; the sleeping grid has its
        /// own hash table (SleepCellMask) and is rebuilt once RebuildMin bodies fell asleep or woke since the last rebuild.</summary>
        public uint MaxActive, SleepCellMask, RebuildMin;
        /// <summary>The cold store: the manifolds of sleeping bodies, their contacts, and the hash table that finds them by pair.</summary>
        public uint MaxColdManifolds, MaxColdContacts, ColdHashMask, Pad5;
        public const int Stride = 224;
        public const uint NoTerrain = 0xFFFFFFFFu;
    }

    /// <summary>Decoding of a body's sleep word (<see cref="AvbdGpuWorld.GetSleepSync"/>): bit 31 = asleep, bit 30 = woken by a
    /// touch this step, bit 29 = in the sleeping grid (out of the hot list), bit 28 = woke this step, below them the steps the
    /// body has rested within its rest anchor.</summary>
    public static class GpuBodySleep
    {
        public const uint Asleep = 0x80000000u;
        public const uint Woken = 0x40000000u;
        public const uint InGrid = 0x20000000u;
        public const uint Fresh = 0x10000000u;
        public const uint CounterMask = 0x0FFFFFFFu;
        public static bool IsAsleep(uint w) => (w & Asleep) != 0;
        public static bool IsInGrid(uint w) => (w & InGrid) != 0;
        public static int RestSteps(uint w) => (int)(w & CounterMask);
    }

    /// <summary>Wake list entry modes (AvbdSleep.compute WAKE_*).</summary>
    public static class WakeMode
    {
        public const uint Self = 0, Neighbours = 1, Spawn = 2;
    }

    /// <summary>Constraint reference packing shared with the shaders.</summary>
    public static class ConsRef
    {
        public const uint Manifold = 0, Joint = 1, Spring = 2, Ignore = 3;
        public static uint Make(uint type, uint index) => (type << 30) | index;
        public static uint Type(uint r) => r >> 30;
        public static uint Index(uint r) => r & 0x3FFFFFFFu;
    }

    /// <summary>Counter / stats slots (AvbdCommon.hlsl CNT_*); the slots from <see cref="Persistent"/> on carry over between steps.</summary>
    public enum StatSlot
    {
        Pairs = 0, Manifolds = 1, Contacts = 2, LargeBodies = 3, Overflow = 4, OverflowBodies = 5, ColorsUsed = 6, Constraints = 7, ActiveJoints = 8,
        TerrainManifolds = 9, Sleeping = 10, Frozen = 11, Woken = 12, Thawed = 13, Hot = 14, WokenList = 15, ActiveSprings = 16,
        Marks = 17, Active = 18, RebuildDue = 19, AsleepStart = 20, Pairs1 = 21, ColdCompactDue = 22,
        Persistent = 24, Pending = 24, Stale = 25, SleepGrid = 26, Rebuilds = 27, RebuildForce = 28, Cold = 29, ColdContacts = 30, ColdDead = 31, Count = 32
    }

    /// <summary>Per-step statistics read back asynchronously (one or two frames old).</summary>
    public struct AvbdGpuStats
    {
        public int Pairs, Manifolds, Contacts, LargeBodies, OverflowBodies, ColorsUsed, Constraints;
        /// <summary>Manifolds against the terrain (counted in <see cref="Manifolds"/> as well).</summary>
        public int TerrainManifolds;
        /// <summary>Bodies asleep at the end of the step and sleeping bodies woken by a touch during the step.</summary>
        public int Sleeping, Woken;
        /// <summary>The cold store: manifolds of sleeping bodies in it (thawed ones included until a compaction), their contacts,
        /// the thawed ones since the last compaction, and the manifolds frozen and thawed this step.</summary>
        public int ColdManifolds, ColdContacts, ColdDead, Frozen, Thawed;
        /// <summary>Bodies of the hot list (the per-body passes run over them: active, or asleep but not yet in the sleeping grid)
        /// and active bodies (dynamic, alive, awake) at the start of the step.</summary>
        public int Hot, Active;
        /// <summary>Bodies in the sleeping grid (static ones included), bodies that fell asleep (pending) or woke (stale) since it
        /// was rebuilt, and the rebuilds so far.</summary>
        public int SleepGrid, Pending, Stale, Rebuilds;
        /// <summary>Capacity overflow bits: 1 pairs, 2 manifolds, 4 contacts, 8 cell entries, 16 large bodies, 32 unsorted cells,
        /// 64 more active bodies than <see cref="AvbdGpuConfig.MaxActive"/>.</summary>
        public int OverflowFlags;
        public int ActiveColors;
        public float LastStepMs, AvgStepMs, MaxStepMs;
        public int Frame;
    }
}
