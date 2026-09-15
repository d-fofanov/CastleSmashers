using NUnit.Framework;
using Phys.AvbdGpu.Scenes;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The GPU solver against the CPU reference on the catalog scenes. The reference is run with the GPU's pair
    /// convention (lower index = body A); what remains is the Gauss-Seidel order (colour order on the GPU, creation order on
    /// the CPU) and float rounding, so the comparison is tolerance based on short runs. Measured errors are ~1e-7 for a single
    /// box, 1e-5 .. 1e-3 for stacks, ropes and friction, ~1e-2 for the pyramid after 30 steps.</summary>
    public class GpuVsReferenceTests
    {
        static void Check(int scene, int steps, float posTol, float rotTol, string label, int iterations = 10)
        {
            var c = GpuTestUtil.Compare(scene, steps, configure: (w, r) => { w.Params.Iterations = iterations; r.iterations = iterations; });
            GpuTestUtil.Log(label, c);
            Assert.Less(c.MaxPositionError, posTol, $"{label}: position error (body {c.WorstBody})");
            Assert.Less(c.MaxRotationError, rotTol, $"{label}: rotation error");
        }

        [Test] public void GroundDrop() => Check(AvbdScenes.Ground, 120, 1e-4f, 1e-4f, "ground drop");
        [Test] public void Spring() => Check(AvbdScenes.Spring, 120, 1e-4f, 1e-4f, "spring");
        /// <summary>Stiff springs (k = 10 000 against m/h^2 = 2 000) do not converge in 10 sweeps, so the sweep order dominates
        /// (0.9 m apart at 10 iterations); at 100 iterations both solvers reach the same implicit Euler solution.</summary>
        [Test] public void SpringsRatioConverged() => Check(AvbdScenes.SpringsRatio, 120, 2e-3f, 2e-3f, "springs ratio, 100 iterations", 100);
        [Test] public void Rope() => Check(AvbdScenes.Rope, 60, 1e-2f, 1e-2f, "rope");
        [Test] public void HeavyRope() => Check(AvbdScenes.HeavyRope, 60, 1e-2f, 1e-2f, "heavy rope");
        [Test] public void Stack() => Check(AvbdScenes.Stack, 120, 2e-2f, 1e-2f, "stack");
        [Test] public void StackRatio() => Check(AvbdScenes.StackRatio, 120, 1e-3f, 1e-3f, "stack ratio");
        [Test] public void Bridge() => Check(AvbdScenes.Bridge, 30, 5e-3f, 5e-3f, "bridge");
        [Test] public void SoftBody() => Check(AvbdScenes.SoftBody, 30, 1e-3f, 1e-3f, "soft body");
        [Test] public void Breakable() => Check(AvbdScenes.Breakable, 30, 2e-2f, 2e-2f, "breakable");
        [Test] public void DynamicFriction() => Check(AvbdScenes.DynamicFriction, 60, 1e-3f, 1e-3f, "dynamic friction");
        [Test] public void StaticFriction() => Check(AvbdScenes.StaticFriction, 60, 1e-3f, 1e-3f, "static friction");
        [Test] public void Pyramid() => Check(AvbdScenes.Pyramid, 30, 5e-2f, 1e-1f, "pyramid");
        /// <summary>Boxes tumbling onto hills: the terrain contacts are generated identically, the differences are the sweep order at the impacts.</summary>
        [Test] public void Terrain() => Check(AvbdScenes.Terrain, 90, 2e-2f, 5e-2f, "terrain");
    }
}
