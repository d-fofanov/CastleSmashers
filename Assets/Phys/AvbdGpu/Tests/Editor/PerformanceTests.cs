using System.Diagnostics;
using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Step times of the GPU solver (excluded from the default runs; `RunTests.ps1 -Filter Phys.AvbdGpu.Tests.PerformanceTests`).
    /// The GPU is synchronised once after the timed steps, so the numbers are amortised GPU step times.</summary>
    public class PerformanceTests
    {
        static void Measure(string label, int scene, int bodies, int iterations, int warmup, int steps, bool sleep = false) => Measure(label, w => w.BuildScene(scene), bodies, iterations, warmup, steps, sleep);

        /// <summary>With <paramref name="sleep"/> the scene is stepped until everything sleeps (at most 1800 steps) before the timed steps.</summary>
        static void Measure(string label, System.Action<AvbdGpuWorld> build, int bodies, int iterations, int warmup, int steps, bool sleep = false)
        {
            using var world = GpuTestUtil.NewWorld(bodies);
            world.Params.Iterations = iterations;
            world.Params.Sleep = sleep;
            build(world);
            for (int i = 0; i < warmup; i++) world.Step();
            if (sleep)
            {
                int dynamic = 0;
                for (int i = 0; i < world.BodyCount; i++) if (!world.GetBodyDef(i).IsStatic) dynamic++;
                int settled = 0;
                for (; settled < 1800; settled += 30)
                {
                    for (int i = 0; i < 30; i++) world.Step();
                    if (world.GetStatsSync().Sleeping == dynamic) break;
                }
                UnityEngine.Debug.Log($"PERF {label}: {world.GetStatsSync().Sleeping} of {dynamic} bodies asleep after {warmup + settled} steps");
            }
            world.GetStatsSync();
            var sw = Stopwatch.StartNew();
            for (int i = 0; i < steps; i++) world.Step();
            var stats = world.GetStatsSync();   // sync
            sw.Stop();
            GpuTestUtil.AssertFinite(world);
            UnityEngine.Debug.Log($"PERF {label}: {world.BodyCount} bodies, {iterations} iterations{(sleep ? ", sleeping" : "")}: {sw.Elapsed.TotalMilliseconds / steps:F2} ms/step, cpu submit {world.Stats.AvgStepMs:F2} ms " +
                $"(pairs {stats.Pairs}, manifolds {stats.Manifolds} ({stats.TerrainManifolds} terrain, {stats.CarriedManifolds} carried), contacts {stats.Contacts}, colours {stats.ColorsUsed}, overflow bodies {stats.OverflowBodies}, flags {stats.OverflowFlags}, asleep {stats.Sleeping})");
            Assert.AreEqual(0, stats.OverflowFlags, "capacity overflow");
        }

        [Test] public void Pyramid137() => Measure("pyramid 16", AvbdScenes.Pyramid, 4096, 10, 30, 120);
        [Test] public void Pyramid22k() => Measure("pyramid 40", AvbdScenes.PyramidLarge, 32768, 10, 60, 60);
        [Test] public void Pyramid22kFourIterations() => Measure("pyramid 40", AvbdScenes.PyramidLarge, 32768, 4, 60, 60);
        [Test] public void Pile50k() => Measure("pile 50 x 20 x 50", AvbdScenes.Pile, 65536, 10, 120, 60);
        [Test] public void Pile50kFourIterations() => Measure("pile 50 x 20 x 50", AvbdScenes.Pile, 65536, 4, 120, 60);
        [Test] public void Pyramid74k() => Measure("pyramid 60", AvbdScenes.PyramidHuge, 81920, 4, 60, 60);
        /// <summary>The terrain pass: the 22k pyramid on a flat heightfield instead of the ground box (1 600 base cubes on the terrain).</summary>
        [Test] public void Pyramid22kOnTerrain() => Measure("pyramid 40 on terrain", w => { w.Clear(); AvbdScenes.ScenePyramid3D(w, 40, terrain: true); }, 32768, 10, 60, 60);
        [Test] public void TerrainScene() => Measure("terrain scene", AvbdScenes.Terrain, 4096, 10, 60, 120);
        /// <summary>The quiet step: the same scenes once everything sleeps (the broadphase, the carried manifolds and the fixed dispatch
        /// overhead are what is left). The pile is not measured asleep: at 10 iterations it keeps popping cubes out for a minute.</summary>
        [Test] public void Pyramid22kAsleep() => Measure("pyramid 40", AvbdScenes.PyramidLarge, 32768, 10, 60, 60, sleep: true);
        [Test] public void Pyramid137Asleep() => Measure("pyramid 16", AvbdScenes.Pyramid, 4096, 10, 30, 120, sleep: true);
        [Test] public void StrongholdAtRest() => Measure("stronghold", BuildStronghold, 32768, 10, 60, 60, sleep: true);
        [Test] public void StrongholdAwake() => Measure("stronghold", BuildStronghold, 32768, 10, 60, 60);

        /// <summary>The 17k-brick castle of the castle demo (preset 7, no terrain, dry-stacked) at the demo scale and mass.</summary>
        static void BuildStronghold(AvbdGpuWorld w)
        {
            w.Clear();
            var plan = CastlePlan.Presets[7];
            var layout = BrickCastle.Generate(plan);
            const float scale = 5f;
            float2 c = BrickCastle.Center(plan) * Brick.Pitch * scale;
            float volume = Brick.Width * Brick.BodyHeight * Brick.Length * scale * scale * scale;
            w.AddBody(new float3(2000f, 1f, 2000f), 0f, 0.6f, new float3(0f, -0.5f, 0f), quaternion.identity, float3.zero);
            BrickCastle.Build(w, layout, new BrickSpec { Scale = scale, Density = 0.25f / volume, Friction = 0.6f, Margin = AvbdGpuConstants.CollisionMargin, Origin = new float3(-c.x, 0f, -c.y) });
            w.Params.Substeps = 1;
        }
    }
}
