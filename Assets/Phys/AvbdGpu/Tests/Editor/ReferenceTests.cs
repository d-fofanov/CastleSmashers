using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdRef;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Behaviour of the CPU reference port on the catalog scenes. These pin down what the GPU solver is compared against.</summary>
    public class ReferenceTests
    {
        static RefSceneBuilder Run(int scene, int steps)
        {
            var b = RefSceneBuilder.Build(scene);
            for (int i = 0; i < steps; i++) b.Solver.Step();
            return b;
        }

        static float MaxSpeed(RefSceneBuilder b)
        {
            float m = 0f;
            foreach (var body in b.Bodies) if (body.mass > 0) m = math.max(m, math.length(body.velocityLin));
            return m;
        }

        [Test]
        public void BoxRestsOnGroundCarryingItsWeight()
        {
            var b = Run(AvbdScenes.Ground, 180);
            var box = b.Bodies[1];
            Assert.Less(MaxSpeed(b), 0.02f, "box should be at rest");
            Assert.AreEqual(1.0f, box.positionLin.y, 0.05f, "box should sit on the ground (half size 0.5 above the ground top at 0.5)");
            Assert.Less(math.length(box.positionLin.xz), 0.01f);

            float normalForce = 0f;
            for (Force f = b.Solver.forces; f != null; f = f.next)
                if (f is Manifold m)
                    for (int i = 0; i < m.numContacts; i++) normalForce += -m.contacts[i].lambda.x;
            Assert.AreEqual(box.mass * 10f, normalForce, box.mass * 10f * 0.1f, "contact multipliers carry the weight");
        }

        [Test]
        public void StackStaysUp()
        {
            var b = Run(AvbdScenes.Stack, 240);
            Assert.Less(MaxSpeed(b), 0.1f);
            for (int i = 1; i < b.Bodies.Count; i++)
            {
                var c = b.Bodies[i];
                Assert.Less(math.length(c.positionLin.xz), 0.25f, $"cube {i} drifted sideways");
                Assert.AreEqual(i, c.positionLin.y, 0.25f, $"cube {i} height (ground top at 0.5, unit cubes)");
            }
        }

        [Test]
        public void StackRatioStaysUp()
        {
            var b = Run(AvbdScenes.StackRatio, 240);
            Assert.Less(MaxSpeed(b), 0.2f);
            float expectedTop = 0.5f;
            float size = 1f;
            for (int i = 1; i < b.Bodies.Count; i++)
            {
                Assert.AreEqual(expectedTop + size * 0.5f, b.Bodies[i].positionLin.y, 0.15f * size, $"cube {i} height");
                Assert.Less(math.length(b.Bodies[i].positionLin.xz), 0.2f * size, $"cube {i} drifted");
                expectedTop += size;
                size *= 2f;
            }
        }

        [Test]
        public void RopeHoldsTogether()
        {
            var b = Run(AvbdScenes.Rope, 240);
            foreach (var j in b.Joints)
            {
                float3 pa = RefMath.Transform(j.bodyA.positionLin, j.bodyA.positionAng, j.rA);
                float3 pb = RefMath.Transform(j.bodyB.positionLin, j.bodyB.positionAng, j.rB);
                Assert.Less(math.length(pa - pb), 0.05f, "joint gap");
            }
            Assert.Less(b.Bodies[b.Bodies.Count - 1].positionLin.y, 10f, "the free end swings below the anchor");
            Assert.Greater(b.Bodies[b.Bodies.Count - 1].positionLin.y, -19f, "the rope does not fall to the ground");
        }

        [Test]
        public void HeavyRopeHoldsTogether()
        {
            var b = Run(AvbdScenes.HeavyRope, 240);
            foreach (var j in b.Joints)
            {
                float3 pa = RefMath.Transform(j.bodyA.positionLin, j.bodyA.positionAng, j.rA);
                float3 pb = RefMath.Transform(j.bodyB.positionLin, j.bodyB.positionAng, j.rB);
                Assert.Less(math.length(pa - pb), 0.25f, "joint gap with a 1000:1 mass ratio");
            }
        }

        [Test]
        public void SpringHangsAtStaticExtension()
        {
            var b = RefSceneBuilder.Build(AvbdScenes.Spring);
            var block = b.Bodies[2];
            float weight = block.mass * 10f;            // 8 kg * 10
            float expected = 4f + weight / 100f;        // rest + mg/k
            // Average over several oscillation periods (T = 2 pi sqrt(m/k) ~ 1.8 s) after a transient.
            float sum = 0f; int n = 0;
            for (int i = 0; i < 900; i++)
            {
                b.Solver.Step();
                if (i >= 180) { sum += 14f - block.positionLin.y; n++; }
            }
            Assert.AreEqual(expected, sum / n, 0.15f * expected, "mean spring length = rest + mg/k");
        }

        [Test]
        public void StaticFrictionRampSortsByFriction()
        {
            var b = RefSceneBuilder.Build(AvbdScenes.StaticFriction);
            // The cubes start axis aligned on a tilted ramp, so they first rock onto a face; judge the motion of the
            // last second (steps 120..180) once they lie flat.
            for (int i = 0; i < 120; i++) b.Solver.Step();
            var mid = new float3[b.Bodies.Count];
            for (int i = 0; i < mid.Length; i++) mid[i] = b.Bodies[i].positionLin;
            for (int i = 0; i < 60; i++) b.Solver.Step();
            // cubes are bodies 2..12 with friction 0.25 .. 0.5 (combined with the ramp: sqrt(mu) = 0.5 .. 0.71, tan 30 = 0.577)
            float lowMove = math.length(b.Bodies[2].positionLin - mid[2]);
            float highMove = math.length(b.Bodies[12].positionLin - mid[12]);
            Assert.Greater(lowMove, 1f, "friction 0.25 keeps sliding");
            Assert.Less(highMove, 0.05f, "friction 0.5 sticks");
        }

        [Test]
        public void DynamicFrictionStopsInFrictionOrder()
        {
            var b = RefSceneBuilder.Build(AvbdScenes.DynamicFriction);
            var start = new float3[b.Bodies.Count];
            for (int i = 0; i < start.Length; i++) start[i] = b.Bodies[i].positionLin;
            for (int i = 0; i < 240; i++) b.Solver.Step();
            float prev = -1f;
            for (int i = 1; i <= 10; i++)  // friction 5 .. 0.5 (body 11 has friction 0 and never stops)
            {
                float d = b.Bodies[i].positionLin.x - start[i].x;
                Assert.Greater(d, prev - 0.05f, $"box {i} (less friction) travels at least as far");
                prev = d;
            }
            // combined friction sqrt(5 * 0.5) = 1.58: v^2 / (2 mu g) = 100 / 31.6 = 3.2 m
            Assert.Less(b.Bodies[1].positionLin.x - start[1].x, 4.0f, "friction 5 stops within ~3.2 m");
            Assert.Greater(b.Bodies[10].positionLin.x - start[10].x, 5f, "friction 0.5 travels far");
        }

        [Test]
        public void BreakableChainBreaks()
        {
            var b = Run(AvbdScenes.Breakable, 240);
            int broken = 0;
            foreach (var j in b.Joints) if (j.broken) broken++;
            Assert.Greater(broken, 0, "at least one joint fractured under the dropped boxes");
        }

        [Test]
        public void BridgeHoldsBoxes()
        {
            var b = Run(AvbdScenes.Bridge, 240);
            // planks are bodies 1..40; boxes 41..90 must rest on or near the bridge, not on the ground
            for (int i = 41; i < b.Bodies.Count; i++)
                Assert.Greater(b.Bodies[i].positionLin.y, 5f, $"box {i} fell through the bridge");
        }

        [Test]
        public void SoftBodyLatticesSurvive()
        {
            var b = Run(AvbdScenes.SoftBody, 240);
            Assert.Less(MaxSpeed(b), 5.0f, "undamped soft lattices keep jiggling but do not explode");
            foreach (var body in b.Bodies) if (body.mass > 0) Assert.Greater(body.positionLin.y, 0.5f, "no cube fell through the ground");
        }

        [Test]
        public void RunsAreDeterministic()
        {
            var a = Run(AvbdScenes.Pyramid, 60);
            var b = Run(AvbdScenes.Pyramid, 60);
            for (int i = 0; i < a.Bodies.Count; i++)
            {
                Assert.IsTrue(math.all(a.Bodies[i].positionLin == b.Bodies[i].positionLin), $"body {i} position differs");
                Assert.IsTrue(a.Bodies[i].positionAng.x == b.Bodies[i].positionAng.x && a.Bodies[i].positionAng.w == b.Bodies[i].positionAng.w, $"body {i} rotation differs");
            }
        }

        [Test]
        public void PyramidStands()
        {
            var b = Run(AvbdScenes.Pyramid, 300);
            Assert.Less(MaxSpeed(b), 0.3f, "pyramid settles");
            // rows start 0.85 apart and compact onto 0.5 m boxes: the top box (last created) ends near 15 * 0.5 + 0.25
            var top = b.Bodies[b.Bodies.Count - 1];
            Assert.Greater(top.positionLin.y, 7.0f, "top box still on top");
            Assert.Less(math.abs(top.positionLin.x - (-0.5f)), 1.0f, "top box centred");
        }

        [Test]
        public void CollideFaceContactGivesFourPoints()
        {
            var solver = new Solver();
            var ground = new Rigid(solver, new float3(100, 1, 100), 0f, 0.5f, float3.zero);
            var box = new Rigid(solver, new float3(1, 1, 1), 1f, 0.5f, new float3(0, 0.99f, 0));
            var contacts = new Manifold.Contact[Manifold.MaxContacts];
            int n = RefCollide.Collide(ground, box, contacts, out Mat3 basis);
            Assert.AreEqual(4, n);
            // normal points from B (box) to A (ground): -y
            Assert.AreEqual(-1f, basis[0].y, 1e-5f);
            for (int i = 0; i < n; i++)
            {
                Assert.AreEqual(0.5f, contacts[i].rA.y, 1e-4f, "point on the ground's top face");
                Assert.AreEqual(-0.5f, contacts[i].rB.y, 1e-4f, "point on the box's bottom face");
            }
        }
    }
}
