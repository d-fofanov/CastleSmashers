using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdRef;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Helpers shared by the GPU tests: paired reference / GPU scene runs and error metrics.</summary>
    public static class GpuTestUtil
    {
        public static AvbdGpuWorld NewWorld(int bodies = 4096)
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("Compute shaders are not supported on this device");
            return new AvbdGpuWorld(AvbdGpuConfig.ForBodies(bodies));
        }

        public struct Comparison
        {
            public float MaxPositionError;
            public float MaxRotationError;   // length of the rotation vector between the two orientations
            public int WorstBody;
            public float GpuMaxSpeed;
            public float RefMaxSpeed;
        }

        /// <summary>Runs the scene on both solvers for the given number of steps and compares every body's pose.</summary>
        public static Comparison Compare(int scene, int steps, AvbdGpuWorld world = null, System.Action<AvbdGpuWorld, Solver> configure = null, bool lowIndexFirst = true)
        {
            bool own = world == null;
            world ??= NewWorld();
            try
            {
                world.BuildScene(scene);
                var reference = RefSceneBuilder.Build(scene);
                reference.Solver.LowIndexFirst = lowIndexFirst;
                configure?.Invoke(world, reference.Solver);
                for (int i = 0; i < steps; i++)
                {
                    world.Step();
                    reference.Solver.Step();
                }
                return Compare(world, reference);
            }
            finally
            {
                if (own) world.Dispose();
            }
        }

        public static Comparison Compare(AvbdGpuWorld world, RefSceneBuilder reference)
        {
            world.GetPosesSync(out var pos, out var rot);
            world.GetVelocitiesSync(out var vel, out _);
            var c = new Comparison();
            for (int i = 0; i < reference.Bodies.Count; i++)
            {
                var body = reference.Bodies[i];
                float pe = math.length(pos[i].xyz - body.positionLin);
                var q = new quaternion(rot[i]);
                var rq = body.positionAng.ToUnity();
                float3 dq = math.mul(q, math.inverse(rq)).value.xyz * 2f;
                float re = math.length(dq);
                if (pe > c.MaxPositionError) { c.MaxPositionError = pe; c.WorstBody = i; }
                c.MaxRotationError = math.max(c.MaxRotationError, re);
                if (body.mass > 0)
                {
                    c.GpuMaxSpeed = math.max(c.GpuMaxSpeed, math.length(vel[i].xyz));
                    c.RefMaxSpeed = math.max(c.RefMaxSpeed, math.length(body.velocityLin));
                }
            }
            return c;
        }

        public static float MaxSpeed(AvbdGpuWorld world)
        {
            world.GetVelocitiesSync(out var vel, out _);
            float m = 0f;
            for (int i = 0; i < world.BodyCount; i++) if (world.GetBodyDef(i).Mass > 0) m = math.max(m, math.length(vel[i].xyz));
            return m;
        }

        public static void AssertFinite(AvbdGpuWorld world)
        {
            world.GetPosesSync(out var pos, out var rot);
            for (int i = 0; i < world.BodyCount; i++)
            {
                Assert.IsTrue(math.all(math.isfinite(pos[i])), $"body {i} position not finite: {pos[i]}");
                Assert.IsTrue(math.all(math.isfinite(rot[i])), $"body {i} rotation not finite: {rot[i]}");
            }
        }

        public static void Log(string label, Comparison c)
        {
            Debug.Log($"{label}: max position error {c.MaxPositionError:E3} (body {c.WorstBody}), max rotation error {c.MaxRotationError:E3}, max speed gpu {c.GpuMaxSpeed:F3} ref {c.RefMaxSpeed:F3}");
        }
    }
}
