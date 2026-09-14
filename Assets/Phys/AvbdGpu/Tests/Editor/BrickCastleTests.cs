using System.Collections.Generic;
using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The castle layout on the stud grid: no overlaps (the layout rejects them), every brick well supported, every
    /// preset builds, and the body conversion places the boxes where the bricks are.</summary>
    public class BrickCastleTests
    {
        [Test]
        public void EveryPresetGeneratesAndIsSupported([Range(0, 2)] int preset)
        {
            var plan = CastlePlan.Presets[preset];
            var layout = BrickCastle.Generate(plan);
            Assert.Greater(layout.Bricks.Count, 500, "brick count");
            int minX = int.MaxValue, maxX = int.MinValue, minZ = int.MaxValue, maxZ = int.MinValue;
            for (int i = 0; i < layout.Bricks.Count; i++)
            {
                var b = layout.Bricks[i];
                // a corbelled stretcher rests on two of its three studs; everything else is fully supported
                Assert.GreaterOrEqual(layout.Support(i), 0.66f, $"preset {preset} brick {i} at ({b.X}, {b.Z}) course {b.Layer} support");
                minX = math.min(minX, b.X); maxX = math.max(maxX, b.X + b.W);
                minZ = math.min(minZ, b.Z); maxZ = math.max(maxZ, b.Z + b.D);
            }
            Assert.AreEqual(0, minX); Assert.AreEqual(plan.Side, maxX, "footprint width");
            Assert.AreEqual(0, minZ); Assert.AreEqual(plan.Side, maxZ, "footprint depth");
            Assert.AreEqual(plan.KeepCourses + 1, layout.Layers, "the keep's merlons are the top course");
            UnityEngine.Debug.Log($"{plan.Name}: {layout.Bricks.Count} bricks, {layout.Layers} courses");
        }

        [Test]
        public void WallsTowersAndGateAreClosed()
        {
            var plan = CastlePlan.Presets[1];
            var layout = BrickCastle.Generate(plan);
            int S = plan.Side, T = BrickCastle.TowerSize, face = BrickCastle.TowerOut;
            // every course of the curtain walls is full between the towers (the English bond tiles flush)
            for (int layer = 0; layer < plan.WallCourses; layer++)
                for (int a = T; a < S - T; a++)
                    for (int d = 0; d < BrickCastle.WallDepth; d++)
                    {
                        Assert.GreaterOrEqual(layout.BrickAt(face + d, a, layer), 0, $"west wall cell ({face + d}, {a}) course {layer}");
                        Assert.GreaterOrEqual(layout.BrickAt(S - face - 1 - d, a, layer), 0, $"east wall cell course {layer}");
                        Assert.GreaterOrEqual(layout.BrickAt(a, S - face - 1 - d, layer), 0, $"back wall cell course {layer}");
                    }
            // the gate passage is open through the gatehouse and closed above it
            int g0 = S / 2 - BrickCastle.GateWidth / 2;
            for (int layer = 0; layer < BrickCastle.GateCourses; layer++)
                for (int x = g0; x < g0 + BrickCastle.GateWidth; x++)
                    for (int z = face; z < face + BrickCastle.GateHouseDepth; z++)
                        Assert.AreEqual(-1, layout.BrickAt(x, z, layer), $"passage cell ({x}, {z}) course {layer} must be free");
            int closed = BrickCastle.GateCourses + BrickCastle.GateWidth / 2;
            for (int layer = closed; layer < plan.GateCourses; layer++)
                for (int x = (S - BrickCastle.GateHouseWidth) / 2; x < (S + BrickCastle.GateHouseWidth) / 2; x++)
                    for (int z = face; z < face + BrickCastle.GateHouseDepth; z++)
                        Assert.GreaterOrEqual(layout.BrickAt(x, z, layer), 0, $"gatehouse cell ({x}, {z}) course {layer} above the arch");
            // tower rings: the hole stays open and the walls are full on both course patterns
            for (int layer = 0; layer < plan.TowerCourses; layer++)
                for (int x = 0; x < T; x++)
                    for (int z = 0; z < T; z++)
                    {
                        bool hole = x >= 3 && x < T - 3 && z >= 3 && z < T - 3;
                        Assert.AreEqual(!hole, layout.BrickAt(x, z, layer) >= 0, $"tower cell ({x}, {z}) course {layer}");
                    }
        }

        [Test]
        public void BodiesMatchTheLayout()
        {
            var layout = BrickCastle.Generate(CastlePlan.Presets[0]);
            var spec = BrickSpec.Default;
            var builder = new RecordingBuilder();
            int first = BrickCastle.Build(builder, layout, spec);
            Assert.AreEqual(0, first);
            Assert.AreEqual(layout.Bricks.Count, builder.Bodies.Count);
            float3 size = spec.BoxSize;
            Assert.AreEqual(Brick.Width, size.x, 1e-6f);
            Assert.AreEqual(Brick.BodyHeight + BrickSpec.CollisionMargin, size.y, 1e-6f, "the box is one margin taller than the brick body");
            Assert.AreEqual(Brick.Length, size.z, 1e-6f);
            for (int i = 0; i < layout.Bricks.Count; i++)
            {
                var b = layout.Bricks[i];
                var (pos, rot) = builder.Bodies[i];
                // the model pivot (bottom-face centre) sits exactly on the top of the course below
                float3 pivot = pos + math.mul(rot, spec.MeshOffset);
                Assert.AreEqual(b.Layer * Brick.BodyHeight, pivot.y, 1e-5f, $"brick {i} pivot height");
                // the footprint in world space is the brick's cells
                float3 half = math.abs(math.mul(rot, size * 0.5f));
                Assert.AreEqual(b.W * Brick.Pitch * 0.5f, half.x, 0.002f, $"brick {i} world half width");
                Assert.AreEqual(b.D * Brick.Pitch * 0.5f, half.z, 0.002f, $"brick {i} world half depth");
                Assert.AreEqual((b.X + b.W * 0.5f) * Brick.Pitch, pos.x, 1e-5f);
                Assert.AreEqual((b.Z + b.D * 0.5f) * Brick.Pitch, pos.z, 1e-5f);
            }
        }

        [Test]
        public void SnapJointsConnectEveryBrickDownwards()
        {
            var layout = BrickCastle.Generate(CastlePlan.Presets[0]);
            var spec = BrickSpec.Default;
            var builder = new RecordingBuilder();
            int first = BrickCastle.Build(builder, layout, spec);
            int joints = BrickCastle.AddSnapJoints(builder, layout, first, spec, 20f);
            Assert.AreEqual(joints, builder.Joints.Count);
            var joined = new HashSet<int>();
            foreach (var (a, b, rA, rB) in builder.Joints)
            {
                joined.Add(b);
                var pb = layout.Bricks[b - first];
                Assert.IsTrue(a == -1 ? pb.Layer == 0 : layout.Bricks[a - first].Layer == pb.Layer - 1, "joints go to the course below or, on the ground course, to the world");
                // both local anchors describe the same world point on the interface plane
                var (posB, rotB) = builder.Bodies[b - first];
                float3 world = posB + math.mul(rotB, rB);
                float3 worldA = a == -1 ? rA : builder.Bodies[a - first].pos + math.mul(builder.Bodies[a - first].rot, rA);
                Assert.Less(math.distance(world, worldA), 1e-4f, "anchor consistency");
                Assert.AreEqual(pb.Layer * Brick.BodyHeight, world.y, 1e-4f, "anchor on the interface plane");
            }
            Assert.AreEqual(layout.Bricks.Count, joined.Count, "every brick is snapped");
            Assert.GreaterOrEqual(joints, 4 * layout.Bricks.Count, "four joints per overlap");
        }

        sealed class RecordingBuilder : ISceneBuilder
        {
            public readonly List<(float3 pos, quaternion rot)> Bodies = new List<(float3, quaternion)>();
            public readonly List<(int a, int b, float3 rA, float3 rB)> Joints = new List<(int, int, float3, float3)>();
            public int AddBody(float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity) { Bodies.Add((position, rotation)); return Bodies.Count - 1; }
            public void AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture) => Joints.Add((bodyA, bodyB, rA, rB));
            public void AddSpring(int bodyA, int bodyB, float3 rA, float3 rB, float stiffness, float rest) { }
            public void AddIgnoreCollision(int bodyA, int bodyB) { }
        }
    }
}
