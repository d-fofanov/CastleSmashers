using NUnit.Framework;
using Phys.AvbdRef;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>External drives (constant force, force towards a point, velocity motor) and rotation control (locked rotation,
    /// heading, align to velocity) on the GPU and in the reference mirror. Drives enter the inertial pose like gravity, so a
    /// free body under a constant force follows the implicit Euler parabola exactly.</summary>
    public class DriveTests
    {
        const float Dt = 1f / 60f;

        /// <summary>A world and a reference solver with the same single 1 kg box (plus an optional ground) and matching drives.</summary>
        sealed class Pair : System.IDisposable
        {
            public AvbdGpuWorld World;
            public RefSceneBuilder Ref;
            public int Body;

            public Pair(float3 size, float3 position, float3 velocity, uint flags, GpuBodyDrive drive, bool ground, float friction = 0.5f, float3? gravity = null)
            {
                World = GpuTestUtil.NewWorld(1024);
                Ref = new RefSceneBuilder(new Solver { LowIndexFirst = true });
                float3 g = gravity ?? new float3(0, -10f, 0);
                World.Params.Gravity = g;
                Ref.Solver.gravity = g;
                if (ground)
                {
                    World.AddBody(new float3(100, 1, 100), 0f, friction, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);
                    Ref.AddBody(new float3(100, 1, 100), 0f, friction, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);
                }
                float density = 1f / (size.x * size.y * size.z);
                Body = World.AddBody(size, density, friction, position, quaternion.identity, velocity, flags, drive);
                Ref.AddBody(size, density, friction, position, quaternion.identity, velocity);
                var r = Ref.Bodies[Body];
                r.lockRotation = (flags & GpuBodyDef.FlagLockRotation) != 0;
                r.heading = (flags & GpuBodyDef.FlagHeading) != 0;
                r.alignVelocity = (flags & GpuBodyDef.FlagAlignVelocity) != 0;
                r.drive = new Drive { mode = (int)drive.Mode, target = drive.Target, mask = drive.Mask, limit = drive.Limit, yaw = drive.Yaw };
            }

            public void Step(int n)
            {
                for (int i = 0; i < n; i++) { World.Step(); Ref.Solver.Step(); }
            }

            public GpuTestUtil.Comparison Compare(string label, float tol)
            {
                var c = GpuTestUtil.Compare(World, Ref);
                GpuTestUtil.Log(label, c);
                Assert.Less(c.MaxPositionError, tol, $"{label}: GPU vs reference position");
                Assert.Less(c.MaxRotationError, tol, $"{label}: GPU vs reference rotation");
                return c;
            }

            public float3 Position() { World.GetPosesSync(out var p, out _); return p[Body].xyz; }
            public quaternion Rotation() { World.GetPosesSync(out _, out var r); return new quaternion(r[Body]); }
            public float3 Velocity() { World.GetVelocitiesSync(out var v, out _); return v[Body].xyz; }
            public void Dispose() => World.Dispose();
        }

        [Test]
        public void ConstantForceFollowsTheImplicitEulerParabola()
        {
            const int n = 90;
            float3 force = new float3(3f, 0f, -1f);
            using var p = new Pair(new float3(1, 1, 1), float3.zero, float3.zero, GpuBodyDef.FlagDriven, GpuBodyDrive.ConstantForce(force), ground: false);
            p.Step(n);
            // implicit Euler with a constant acceleration: x_n = x_0 + a dt^2 n (n + 1) / 2 (the velocities are reconstructed
            // from float positions, so the box starts at the origin to keep the round-off below the tolerance)
            float3 accel = new float3(0, -10f, 0) + force;   // 1 kg
            float3 expected = accel * (Dt * Dt * n * (n + 1) * 0.5f);
            float3 pos = p.Position();
            Assert.Less(math.distance(pos, expected), 1e-3f, $"driven box at {pos}, expected {expected}");
            p.Compare("constant force", 1e-4f);   // float round-off (fused multiply-adds on the GPU): ~1e-5 after 90 steps
        }

        [Test]
        public void ForceTowardsAPointAttracts()
        {
            float3 target = new float3(10, 5, 0);
            using var p = new Pair(new float3(1, 1, 1), float3.zero, float3.zero, GpuBodyDef.FlagDriven, GpuBodyDrive.TowardsPoint(target, 5f), ground: false, gravity: float3.zero);
            p.Step(30);
            float3 v = p.Velocity();
            Assert.Greater(math.dot(math.normalize(v), math.normalize(target)), 0.999f, "the box accelerates towards the point");
            // a = 5 m/s^2 for half a second: 0.625 m along the line (implicit Euler: slightly more)
            float d = math.length(p.Position());
            Assert.Greater(d, 0.6f); Assert.Less(d, 0.7f);
            p.Step(200);
            Assert.Greater(math.dot(p.Position(), math.normalize(target)), math.length(target), "the box overshoots the point (no drag)");
            p.Compare("force towards a point", 1e-3f);
        }

        [Test]
        public void MotorReachesTheTargetVelocityOnTheGround()
        {
            float3 vTarget = new float3(2f, 0f, 0f);
            var drive = GpuBodyDrive.Velocity(vTarget, 50f, new float3(1, 0, 1));
            using var p = new Pair(new float3(1, 1, 1), new float3(0, 0.5f, 0), float3.zero, GpuBodyDef.FlagDriven, drive, ground: true);
            p.Step(120);
            float3 v = p.Velocity();
            // the motor is a proportional controller with gain m / dt: under the steady friction load F = mu m g = 5 N it runs
            // F dt / m = 0.083 m/s below the target
            Assert.AreEqual(2f - 5f * Dt, v.x, 0.02f, "target speed less the friction droop");
            Assert.Less(math.abs(v.z), 0.01f);
            Assert.Less(math.abs(p.Position().y - 0.49f), 0.02f, "the box stays on the ground");
            p.Compare("motor", 2e-3f);
        }

        [Test]
        public void LockedRotationKeepsATallBoxUprightWhileAFreeOneTips()
        {
            var size = new float3(0.5f, 2f, 0.5f);
            var drive = GpuBodyDrive.Velocity(new float3(3f, 0f, 0f), 100f, new float3(1, 0, 1));
            using var locked = new Pair(size, new float3(0, 1f, 0), float3.zero, GpuBodyDef.FlagDriven | GpuBodyDef.FlagLockRotation, drive, ground: true, friction: 0.6f);
            using var free = new Pair(size, new float3(0, 1f, 0), float3.zero, GpuBodyDef.FlagDriven, drive, ground: true, friction: 0.6f);
            locked.Step(180);
            free.Step(180);
            float lockedTilt = math.length(math.mul(locked.Rotation(), math.inverse(quaternion.identity)).value.xyz) * 2f;
            float freeTilt = math.length(math.mul(free.Rotation(), math.inverse(quaternion.identity)).value.xyz) * 2f;
            Assert.Less(lockedTilt, 1e-5f, "locked: no rotation at all");
            Assert.Greater(locked.Position().x, 4f, "locked: the box walked");
            Assert.Greater(freeTilt, 0.5f, "free: the friction at the base tips the box (mu 0.6 > width / height 0.25)");
            locked.Compare("locked box", 5e-3f);
            GpuTestUtil.AssertFinite(free.World);
        }

        [Test]
        public void HeadingSetsTheOrientationEveryStep()
        {
            float yaw = 1.2f;
            var drive = GpuBodyDrive.Velocity(float3.zero, 10f, new float3(1, 0, 1), yaw);
            using var p = new Pair(new float3(1, 2, 1), new float3(0, 1f, 0), float3.zero, GpuBodyDef.FlagDriven | GpuBodyDef.FlagHeading, drive, ground: true);
            p.Step(30);
            var q = p.Rotation();
            var expected = quaternion.RotateY(yaw);
            float err = math.length(math.mul(q, math.inverse(expected)).value.xyz) * 2f;
            Assert.Less(err, 1e-5f, $"heading {yaw} rad: rotation {q.value}");
            p.Compare("heading", 1e-4f);
        }

        [Test]
        public void AlignToVelocityPointsTheBodyAlongItsMotion()
        {
            float3 v0 = new float3(10f, 10f, 3f);
            using var p = new Pair(new float3(0.2f, 0.05f, 1f), new float3(0, 20f, 0), v0, GpuBodyDef.FlagAlignVelocity, default, ground: false);
            for (int step = 0; step < 40; step++)
            {
                p.Step(1);
                float3 v = p.Velocity();
                float3 forward = math.mul(p.Rotation(), new float3(0, 0, 1));
                // the orientation is set at the start of the step from the velocity of the previous step
                float3 vPrev = v - new float3(0, -10f, 0) * Dt;
                Assert.Greater(math.dot(forward, math.normalize(vPrev)), 0.9999f, $"step {step}: +z {forward} along the velocity {vPrev}");
                Assert.Less(math.abs(math.mul(p.Rotation(), new float3(1, 0, 0)).y), 1e-4f, "the body's +x stays level (up reference)");
            }
            p.Compare("align to velocity", 1e-4f);
            Debug.Log($"aligned body after 40 steps: forward {math.mul(p.Rotation(), new float3(0, 0, 1))} velocity {p.Velocity()}");
        }

        [Test]
        public void DrivesLeaveOtherBodiesAlone()
        {
            // a driven box next to an undriven one: only the driven one moves sideways
            using var world = GpuTestUtil.NewWorld(1024);
            world.AddBody(new float3(100, 1, 100), 0f, 0.5f, new float3(0, -0.5f, 0), quaternion.identity, float3.zero);
            int plain = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 0.5f, 0), quaternion.identity, float3.zero);
            int driven = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 0.5f, 5), quaternion.identity, float3.zero, GpuBodyDef.FlagDriven, GpuBodyDrive.ConstantForce(new float3(20, 0, 0)));
            for (int i = 0; i < 60; i++) world.Step();
            world.GetPosesSync(out var pos, out _);
            Assert.Less(math.abs(pos[plain].x), 1e-4f);
            Assert.Greater(pos[driven].x, 5f);
            // switching the drive off stops the push
            world.SetBodyDrive(driven, default);
            world.GetVelocitiesSync(out var vel, out _);
            float vx = vel[driven].x;
            for (int i = 0; i < 60; i++) world.Step();
            world.GetVelocitiesSync(out vel, out _);
            Assert.Less(vel[driven].x, vx, "friction slows the box once the force is gone");
            GpuTestUtil.AssertFinite(world);
        }
    }
}
