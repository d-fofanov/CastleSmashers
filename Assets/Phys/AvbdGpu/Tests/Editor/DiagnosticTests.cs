using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdRef;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Logs solver internals for debugging; asserts nothing. Excluded from the default test runs.</summary>
    [Category("Diagnostic")]
    public class DiagnosticTests
    {
        [Test]
        public void SpringsRatioIterations()
        {
            foreach (int it in new[] { 10, 30, 100, 300 })
            {
                var c = GpuTestUtil.Compare(AvbdScenes.SpringsRatio, 120, configure: (w, r) => { w.Params.Iterations = it; r.iterations = it; });
                GpuTestUtil.Log($"springs ratio, {it} iterations", c);
            }
            foreach (int it in new[] { 10, 100 })
            {
                var c = GpuTestUtil.Compare(AvbdScenes.SpringsRatio, 20, configure: (w, r) => { w.Params.Iterations = it; r.iterations = it; });
                GpuTestUtil.Log($"springs ratio, {it} iterations, 20 steps", c);
            }
        }

        [Test]
        public void DynamicFrictionTrace()
        {
            using var world = GpuTestUtil.NewWorld();
            world.BuildScene(AvbdScenes.DynamicFriction);
            var reference = RefSceneBuilder.Build(AvbdScenes.DynamicFriction);
            const int body = 9;
            for (int step = 0; step < 60; step++)
            {
                world.Step();
                reference.Solver.Step();
                if (step % 5 != 0 && step > 10) continue;
                world.GetPosesSync(out var pos, out var rot);
                world.GetVelocitiesSync(out var vel, out _);
                var rb = reference.Bodies[body];
                var manifolds = world.GetManifoldsSync(out var contacts);
                string gpuContacts = "";
                for (int m = 0; m < manifolds.Length; m++)
                {
                    if (manifolds[m].BodyA != body && manifolds[m].BodyB != body) continue;
                    for (uint c = 0; c < manifolds[m].NumContacts; c++)
                    {
                        var ct = contacts[manifolds[m].ContactStart + c];
                        gpuContacts += $"\n    gpu c{c} feat {ct.FeatureKey:X} stick {ct.Stick} rA {ct.RA} C0 {ct.C0} pen {ct.Penalty} lam {ct.Lambda}";
                    }
                }
                string refContacts = "";
                for (Force f = reference.Solver.forces; f != null; f = f.next)
                {
                    if (!(f is Manifold rm) || (rm.bodyA != rb && rm.bodyB != rb)) continue;
                    for (int c = 0; c < rm.numContacts; c++)
                    {
                        var ct = rm.contacts[c];
                        refContacts += $"\n    ref c{c} feat {ct.feature:X} stick {ct.stick} rA {ct.rA} C0 {ct.C0} pen {ct.penalty} lam {ct.lambda}";
                    }
                }
                Debug.Log($"step {step}: gpu pos {pos[body].xyz} vel {vel[body].xyz} | ref pos {rb.positionLin} vel {rb.velocityLin} | err {math.length(pos[body].xyz - rb.positionLin):E3}{gpuContacts}{refContacts}");
            }
        }
    }
}
