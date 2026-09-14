using NUnit.Framework;
using Phys.AvbdRef;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Snap fracture of joints: the linear multiplier's pull along the snap axis (tension), its part across the axis
    /// (shear) and the anchor separation each break the joint past their own limit, compression never does. A unit cube B is
    /// held on a static cube A by one hard ball-socket joint with an angular lock and loaded through the gravity vector; the
    /// GPU solver and the reference must agree.</summary>
    public class SnapFractureTests
    {
        const int Steps = 120;

        [Test]
        public void TensionBreaksPastTheLimit()
        {
            float3 pullUp = new float3(0, 10, 0);                         // B weighs 1 kg: 10 N of tension along its +y
            Assert.IsTrue(Gpu(pullUp, float.PositiveInfinity, 5f, float.PositiveInfinity), "GPU: 10 N > 5 N breaks");
            Assert.IsTrue(Reference(pullUp, float.PositiveInfinity, 5f, float.PositiveInfinity), "reference: 10 N > 5 N breaks");
            Assert.IsFalse(Gpu(pullUp, float.PositiveInfinity, 20f, float.PositiveInfinity), "GPU: 10 N < 20 N holds");
            Assert.IsFalse(Reference(pullUp, float.PositiveInfinity, 20f, float.PositiveInfinity), "reference: 10 N < 20 N holds");
        }

        [Test]
        public void ShearBreaksPastTheLimitAndCompressionNever()
        {
            float3 sideways = new float3(30, -10, 0);                     // 30 N of shear, 10 N of compression
            Assert.IsTrue(Gpu(sideways, 20f, 5f, float.PositiveInfinity), "GPU: 30 N > 20 N shear breaks");
            Assert.IsTrue(Reference(sideways, 20f, 5f, float.PositiveInfinity), "reference: 30 N > 20 N shear breaks");
            Assert.IsFalse(Gpu(sideways, 40f, 5f, float.PositiveInfinity), "GPU: 30 N < 40 N holds although the compression exceeds the 5 N tension limit");
            Assert.IsFalse(Reference(sideways, 40f, 5f, float.PositiveInfinity), "reference: 30 N < 40 N holds although the compression exceeds the 5 N tension limit");
        }

        [Test]
        public void SeparationBreaksPastTheLimit()
        {
            // the penalty ramps up from its minimum, so the first steps of a 10 N pull open the joint by millimetres
            float3 pullUp = new float3(0, 10, 0);
            Assert.IsTrue(Gpu(pullUp, float.PositiveInfinity, float.PositiveInfinity, 0.001f), "GPU: 1 mm separation breaks");
            Assert.IsTrue(Reference(pullUp, float.PositiveInfinity, float.PositiveInfinity, 0.001f), "reference: 1 mm separation breaks");
            Assert.IsFalse(Gpu(pullUp, float.PositiveInfinity, float.PositiveInfinity, 1f), "GPU: 1 m separation holds");
            Assert.IsFalse(Reference(pullUp, float.PositiveInfinity, float.PositiveInfinity, 1f), "reference: 1 m separation holds");
        }

        /// <summary>Static A at the origin, B on top of it, one snap joint at the interface; returns whether the joint broke
        /// (and checks that B left when it did, stayed when it did not).</summary>
        static bool Gpu(float3 gravity, float lateral, float tension, float distance)
        {
            using var world = GpuTestUtil.NewWorld(1024);
            int a = world.AddBody(new float3(1, 1, 1), 0f, 0.5f, float3.zero, quaternion.identity, float3.zero);
            int b = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 1, 0), quaternion.identity, float3.zero);
            world.AddJointIndexed(a, b, new float3(0, 0.5f, 0), new float3(0, -0.5f, 0), float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity, lateral, tension, distance, 2);
            world.Params.Gravity = gravity;
            for (int i = 0; i < Steps; i++) world.Step();
            bool broken = world.GetJointStatesSync()[0].Broken != 0;
            world.GetPosesSync(out var pos, out _);
            GpuTestUtil.AssertFinite(world);
            Assert.AreEqual(broken, math.distance(pos[b].xyz, new float3(0, 1, 0)) > 0.5f, "GPU: B leaves exactly when the joint breaks");
            return broken;
        }

        static bool Reference(float3 gravity, float lateral, float tension, float distance)
        {
            var solver = new Solver { gravity = gravity };
            var a = new Rigid(solver, new float3(1, 1, 1), 0f, 0.5f, float3.zero, float3.zero);
            var b = new Rigid(solver, new float3(1, 1, 1), 1f, 0.5f, new float3(0, 1, 0), float3.zero);
            var joint = new Joint(solver, a, b, new float3(0, 0.5f, 0), new float3(0, -0.5f, 0), float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity)
                { fractureLateral = lateral, fractureTension = tension, breakDistance = distance, snapAxis = 2 };
            for (int i = 0; i < Steps; i++) solver.Step();
            Assert.AreEqual(joint.broken, math.distance(b.positionLin, new float3(0, 1, 0)) > 0.5f, "reference: B leaves exactly when the joint breaks");
            return joint.broken;
        }
    }
}
