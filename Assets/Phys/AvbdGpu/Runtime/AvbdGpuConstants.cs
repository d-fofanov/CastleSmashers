namespace Phys.AvbdGpu
{
    /// <summary>Solver constants shared by the C# side and the compute shaders (AvbdCommon.hlsl mirrors them).</summary>
    public static class AvbdGpuConstants
    {
        public const float PenaltyMin = 1.0f;              // Minimum penalty parameter
        public const float PenaltyMax = 10000000000.0f;    // Maximum penalty parameter
        public const float CollisionMargin = 0.01f;        // Margin for collision detection to avoid flickering contacts
        public const float StickThresh = 0.00001f;         // Position threshold for sticking contacts (ie static friction)
        /// <summary>Stiffness at or above this value is a hard constraint (the reference uses INFINITY; HLSL gets a finite sentinel).</summary>
        public const float HardStiffness = 1e30f;
        public const int MaxContactsPerManifold = 8;
        public const int MaxColors = 32;
        public const int ThreadGroupSize = 64;

        public static float ToGpuStiffness(float k) => float.IsPositiveInfinity(k) || k >= HardStiffness ? HardStiffness : k;
        public static bool IsHard(float k) => float.IsPositiveInfinity(k) || k >= HardStiffness;
    }
}
