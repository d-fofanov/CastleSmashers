using Unity.Mathematics;

namespace Phys.AvbdGpu.Siege
{
    /// <summary>Launch velocities of drag-free projectiles under a downward gravity of magnitude g. The solver's implicit Euler
    /// falls g t dt / 2 further than the parabola after a time t; <paramref name="dt"/> (the substep) corrects the launch
    /// velocity for that, so the discrete trajectory passes through the target exactly (0 for the continuous parabola).</summary>
    public static class Ballistics
    {
        /// <summary>The velocity that reaches <paramref name="to"/> from <paramref name="from"/> when launched at
        /// <paramref name="elevation"/> radians above the horizontal; false when the target lies above that line.</summary>
        public static bool AtElevation(float3 from, float3 to, float elevation, float g, float dt, out float3 velocity)
        {
            velocity = float3.zero;
            float3 d = to - from;
            float2 flat = d.xz;
            float dist = math.length(flat);
            if (dist < 1e-4f) return false;
            float cos = math.cos(elevation), sin = math.sin(elevation);
            float denom = 2f * cos * cos * (dist * sin / cos - d.y);
            if (denom <= 1e-6f) return false;
            float v = math.sqrt(g * dist * dist / denom);
            velocity = new float3(flat.x / dist * cos, sin, flat.y / dist * cos) * v;
            velocity.y += 0.5f * g * dt;
            return true;
        }

        /// <summary>The direction that reaches <paramref name="to"/> at the given speed; false when out of range.
        /// <paramref name="highArc"/> picks the lobbed solution instead of the flat one.</summary>
        public static bool AtSpeed(float3 from, float3 to, float speed, float g, bool highArc, float dt, out float3 velocity)
        {
            velocity = float3.zero;
            float3 d = to - from;
            float2 flat = d.xz;
            float dist = math.length(flat);
            if (dist < 1e-4f) return false;
            float v2 = speed * speed;
            float disc = v2 * v2 - g * (g * dist * dist + 2f * d.y * v2);
            if (disc < 0f) return false;
            float root = math.sqrt(disc);
            float angle = math.atan((v2 + (highArc ? root : -root)) / (g * dist));
            float cos = math.cos(angle), sin = math.sin(angle);
            velocity = new float3(flat.x / dist * cos, sin, flat.y / dist * cos) * speed;
            velocity.y += 0.5f * g * dt;
            return true;
        }

        /// <summary>Position after <paramref name="steps"/> implicit Euler steps (what the solver does with a free body).</summary>
        public static float3 Integrate(float3 from, float3 velocity, float g, float dt, int steps)
        {
            float3 accel = new float3(0, -g, 0);
            return from + velocity * (dt * steps) + accel * (dt * dt * steps * (steps + 1) * 0.5f);
        }

        /// <summary>Time of flight to the target's horizontal distance.</summary>
        public static float FlightTime(float3 from, float3 to, float3 velocity)
        {
            float dist = math.length((to - from).xz);
            float horizontal = math.length(velocity.xz);
            return horizontal > 1e-6f ? dist / horizontal : 0f;
        }
    }
}
