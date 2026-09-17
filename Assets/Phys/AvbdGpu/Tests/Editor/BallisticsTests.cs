using NUnit.Framework;
using Phys.AvbdGpu.Siege;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The ballistic solutions match the solver's implicit Euler: a launched box passes through its aim point.</summary>
    public class BallisticsTests
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
    }
}
