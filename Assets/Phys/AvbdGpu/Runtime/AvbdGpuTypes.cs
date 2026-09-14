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
        public uint Flags;       // bit 0: static
        public float Pad0, Pad1;
        public const int Stride = 48;
        public const uint FlagStatic = 1;
        public bool IsStatic => Mass <= 0f;
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

    /// <summary>Mirror of cbuffer AvbdParams (HLSL packing: 8 float4 registers).</summary>
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
        public uint ColorRounds, Substep, Pad0, Pad1;
        public const int Stride = 128;
    }

    /// <summary>Constraint reference packing shared with the shaders.</summary>
    public static class ConsRef
    {
        public const uint Manifold = 0, Joint = 1, Spring = 2, Ignore = 3;
        public static uint Make(uint type, uint index) => (type << 30) | index;
        public static uint Type(uint r) => r >> 30;
        public static uint Index(uint r) => r & 0x3FFFFFFFu;
    }

    /// <summary>Counter / stats slots (AvbdCommon.hlsl CNT_*).</summary>
    public enum StatSlot
    {
        Pairs = 0, Manifolds = 1, Contacts = 2, LargeBodies = 3, Overflow = 4, OverflowBodies = 5, ColorsUsed = 6, Constraints = 7, ActiveJoints = 8, Count = 16
    }

    /// <summary>Per-step statistics read back asynchronously (one or two frames old).</summary>
    public struct AvbdGpuStats
    {
        public int Pairs, Manifolds, Contacts, LargeBodies, OverflowBodies, ColorsUsed, Constraints;
        /// <summary>Capacity overflow bits: 1 pairs, 2 manifolds, 4 contacts, 8 cell entries, 16 large bodies, 32 unsorted cells.</summary>
        public int OverflowFlags;
        public int ActiveColors;
        public float LastStepMs, AvgStepMs, MaxStepMs;
        public int Frame;
    }
}
