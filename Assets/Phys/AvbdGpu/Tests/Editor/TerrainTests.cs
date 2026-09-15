using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdRef;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The heightfield: bilinear sampling and its normal, the flat continuation beyond the border, the max mip, the
    /// plateau editing and the procedural presets; then the terrain contacts of the CPU reference (the box's lattice points
    /// against the surface), which the GPU kernel is compared against.</summary>
    public class TerrainTests
    {
        static Heightfield Plane(float a, float b, float c, int res = 17, float cell = 2f)
        {
            var f = new Heightfield(res, res, new float2(cell, cell), new float2(-10f, -6f));
            for (int z = 0; z < res; z++)
                for (int x = 0; x < res; x++)
                {
                    float2 p = f.Origin + new float2(x * cell, z * cell);
                    f[x, z] = a * p.x + b * p.y + c;
                }
            f.BuildMaxMip();
            return f;
        }

        [Test]
        public void FlatFieldSamplesTheSameHeightEverywhere()
        {
            var f = Heightfield.Flat(9, 5, new float2(1.5f, 2f), new float2(3f, -4f), 2.5f);
            foreach (var p in new[] { new float2(3f, -4f), new float2(7.3f, 1.1f), new float2(15f, 4f), new float2(-100f, 100f), new float2(1e5f, -1e5f) })
            {
                f.Sample(p, out float h, out float3 n);
                Assert.AreEqual(2.5f, h, 1e-6f, $"height at {p}");
                Assert.AreEqual(1f, n.y, 1e-6f, $"normal at {p}");
            }
            Assert.AreEqual(2.5f, f.MinHeight); Assert.AreEqual(2.5f, f.MaxHeight);
            Assert.AreEqual(new float2(12f, 8f), f.Extent);
        }

        [Test]
        public void InclinedPlaneIsReproducedExactlyWithItsNormal()
        {
            const float a = 0.3f, b = -0.2f, c = 1.5f;
            var f = Plane(a, b, c);
            float3 expected = math.normalize(new float3(-a, 1f, -b));
            var rng = new Random(7u);
            for (int i = 0; i < 200; i++)
            {
                float2 p = f.Origin + rng.NextFloat2() * f.Extent;
                f.Sample(p, out float h, out float3 n);
                Assert.AreEqual(a * p.x + b * p.y + c, h, 1e-4f, $"height at {p}");
                Assert.Less(math.distance(n, expected), 1e-5f, $"normal at {p}");
            }
        }

        [Test]
        public void BeyondTheBorderTheTerrainContinuesFlat()
        {
            var f = Plane(0.3f, -0.2f, 1.5f);
            float2 edge = f.Origin + f.Extent;                   // the far corner sample
            f.Sample(edge + new float2(50f, 0f), out float h, out float3 n);
            Assert.AreEqual(f[f.ResX - 1, f.ResZ - 1], h, 1e-5f, "past the x border: the edge height");
            Assert.AreEqual(0f, n.x, 1e-6f, "no slope across x outside");
            Assert.Greater(n.z, 0f, "the z slope of the edge row remains (b < 0: dh/dz < 0, normal.z = -dh/dz > 0)");
            f.Sample(f.Origin - new float2(3f, 3f), out h, out n);
            Assert.AreEqual(f[0, 0], h, 1e-5f, "past both borders: the corner height");
            Assert.AreEqual(1f, n.y, 1e-6f, "flat outside both borders");
        }

        [Test]
        public void MaxMipBoundsEverySampleOfTheRectangle()
        {
            var f = Heightfield.Generate(TerrainPreset.Hills, 3u, 65, 1.5f, 6f, 20f);
            Assert.AreEqual(8, f.MipX); Assert.AreEqual(8, f.MipZ);
            var rng = new Random(11u);
            for (int i = 0; i < 300; i++)
            {
                float2 lo = f.Origin - 5f + rng.NextFloat2() * (f.Extent + 10f);
                float2 hi = lo + rng.NextFloat2() * 20f;
                float bound = f.MaxOver(lo, hi);
                // every sample inside the rectangle (and the bilinear surface, whose maximum is at a sample) stays below the bound
                for (int z = 0; z < f.ResZ; z++)
                    for (int x = 0; x < f.ResX; x++)
                    {
                        float2 p = f.Origin + new float2(x * f.Cell.x, z * f.Cell.y);
                        if (math.any(p < lo) || math.any(p > hi)) continue;
                        Assert.LessOrEqual(f[x, z], bound, $"sample ({x}, {z}) above the bound of [{lo}, {hi}]");
                    }
                for (int k = 0; k < 20; k++)
                    Assert.LessOrEqual(f.Height(math.lerp(lo, hi, rng.NextFloat2())), bound + 1e-5f);
            }
            Assert.AreEqual(f.MaxHeight, f.MaxOver(f.Origin - 1000f, f.Origin + 1000f), "a rectangle spanning the field answers with the maximum");
        }

        [Test]
        public void FlattenMakesAPlateauAndBlendsOverTheSkirt()
        {
            var f = Heightfield.Generate(TerrainPreset.Hills, 5u, 129, 1f, 8f, 30f);
            var before = (float[])f.Heights.Clone();
            float2 lo = new float2(-12f, -8f), hi = new float2(12f, 8f);
            float plateau = f.MeanHeight(lo, hi);
            f.Flatten(lo, hi, plateau, 6f);
            Assert.AreEqual(plateau, f.MeanHeight(lo, hi), 1e-5f);
            for (int z = 0; z < f.ResZ; z++)
                for (int x = 0; x < f.ResX; x++)
                {
                    float2 p = f.Origin + new float2(x * f.Cell.x, z * f.Cell.y);
                    float2 d = math.max(math.max(lo - p, p - hi), 0f);
                    float dist = math.length(d);
                    if (dist == 0f) Assert.AreEqual(plateau, f[x, z], 1e-6f, $"inside at {p}");
                    else if (dist >= 6f) Assert.AreEqual(before[z * f.ResX + x], f[x, z], $"outside the skirt at {p}");
                    else
                    {
                        float a = math.min(plateau, before[z * f.ResX + x]), b = math.max(plateau, before[z * f.ResX + x]);
                        Assert.That(f[x, z], Is.InRange(a - 1e-5f, b + 1e-5f), $"skirt at {p}");
                    }
                }
            // the mip follows the edit: the blocks lying wholly inside the plateau (8 m blocks from -64) report exactly its height
            Assert.AreEqual(plateau, f.MaxOver(new float2(-7f, -7f), new float2(7f, 7f)), 1e-5f, "the mip follows the edit");
            Assert.GreaterOrEqual(f.MaxOver(lo, hi), plateau, "blocks reaching into the skirt bound it from above");
        }

        [Test]
        public void PresetsAreDeterministicAndBounded()
        {
            foreach (var preset in new[] { TerrainPreset.Hills, TerrainPreset.Valley, TerrainPreset.Ridge })
            {
                var a = Heightfield.Generate(preset, 42u, 65, 2f, 10f);
                var b = Heightfield.Generate(preset, 42u, 65, 2f, 10f);
                var c = Heightfield.Generate(preset, 43u, 65, 2f, 10f);
                CollectionAssert.AreEqual(a.Heights, b.Heights, $"{preset}: the same seed gives the same field");
                CollectionAssert.AreNotEqual(a.Heights, c.Heights, $"{preset}: another seed gives another field");
                Assert.Greater(a.MaxHeight - a.MinHeight, 2f, $"{preset} has relief");
                Assert.Less(a.MaxHeight - a.MinHeight, 16f, $"{preset} stays near its amplitude");
                Assert.AreEqual(float2.zero, a.Center, $"{preset} is centred on the origin");
            }
            var flat = Heightfield.Generate(TerrainPreset.None, 1u, 33, 1f, 10f);
            Assert.AreEqual(0f, flat.MaxHeight); Assert.AreEqual(0f, flat.MinHeight);
        }

        // ------------------------------------------------------------------------------------------------ reference contacts

        static Manifold TerrainManifold(RefSceneBuilder b, Rigid body)
        {
            for (Force f = b.Solver.forces; f != null; f = f.next)
                if (f is Manifold m && m.bodyA == body && m.bodyB.terrain) return m;
            return null;
        }

        static void Run(RefSceneBuilder b, int steps) { for (int i = 0; i < steps; i++) b.Solver.Step(); }

        [Test]
        public void LatticeHasCornersEdgesAndFaceCentresInThatOrder()
        {
            var seen = new System.Collections.Generic.HashSet<float3>();
            for (int k = 0; k < RefCollide.LatticePoints; k++)
            {
                float3 p = RefCollide.LatticePoint(k);
                Assert.IsTrue(seen.Add(p), $"lattice point {k} repeats {p}");
                int zeros = (p.x == 0f ? 1 : 0) + (p.y == 0f ? 1 : 0) + (p.z == 0f ? 1 : 0);
                Assert.AreEqual(k < 8 ? 0 : k < 20 ? 1 : 2, zeros, $"lattice point {k} = {p}");
                Assert.IsTrue(math.all(math.abs(p) <= 1f));
            }
        }

        [Test]
        public void ReferenceBoxRestsOnAFlatFieldOneMarginDeepOnEightPoints()
        {
            var b = new RefSceneBuilder(new Solver());
            int terrain = b.SetTerrain(Heightfield.Flat(17, 17, new float2(4f, 4f), new float2(-32f, -32f), 2f), 0.5f);
            // set down 2 cm above the field: a drop from height is caught up to a step of travel deep and creeps back up over
            // seconds, on the terrain as on the ground box (alpha leaves 99 % of the initial error per step)
            int box = b.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0.3f, 2.52f, -0.2f), quaternion.identity, float3.zero);
            Assert.AreEqual(0, terrain); Assert.AreEqual(1, box);
            Run(b, 300);
            var body = b.Bodies[box];
            Assert.Less(math.length(body.velocityLin), 0.02f, "at rest");
            Assert.AreEqual(2f + 0.5f - RefConstants.CollisionMargin, body.positionLin.y, 3e-3f, "resting one margin deep in the field");
            Assert.Less(math.length(body.positionLin.xz - new float2(0.3f, -0.2f)), 1e-3f, "no drift on the flat");
            var m = TerrainManifold(b, body);
            Assert.IsNotNull(m, "a terrain manifold");
            Assert.AreEqual(8, m.numContacts, "the four bottom corners and the four bottom edge midpoints (ties keep the corners)");
            float weight = 0f;
            for (int i = 0; i < m.numContacts; i++)
            {
                int key = m.contacts[i].feature;
                Assert.AreEqual(RefCollide.AXIS_TERRAIN, key >> 24, "terrain feature prefix");
                int k = key & 0xFF;
                Assert.Less(k, 20, "corners and edge midpoints only");
                Assert.AreEqual(-1f, RefCollide.LatticePoint(k).y, "all on the bottom face");
                Assert.IsTrue(m.contacts[i].stick, "a resting contact sticks");
                weight += -m.contacts[i].lambda.x;
            }
            Assert.AreEqual(10f, weight, 1f, "the multipliers carry the weight");
            Assert.AreEqual(1f, m.basis.r0.y, 1e-6f, "the manifold normal points up");
        }

        /// <summary>The static-friction scene as a heightfield: a 30 degree slope of friction 1, cubes of friction 0.25 (combined 0.5,
        /// below tan 30 = 0.58: slides) and 0.5 (combined 0.71: sticks).</summary>
        [Test]
        public void ReferenceCubesStickOrSlideOnASlopeByTheirFriction()
        {
            float slope = math.tan(math.radians(30f));
            var field = new Heightfield(41, 41, new float2(1f, 1f), new float2(-20f, -20f));
            for (int z = 0; z < 41; z++) for (int x = 0; x < 41; x++) field[x, z] = (x - 20) * slope * -1f;   // rising towards -x
            field.BuildMaxMip();
            var b = new RefSceneBuilder(new Solver());
            b.SetTerrain(field, 1f);
            quaternion tilt = quaternion.RotateZ(-math.radians(30f));   // the bottom face of the cube on the slope
            int slider = b.AddBody(new float3(1, 1, 1), 1f, 0.25f, new float3(0f, 0.5f / math.cos(math.radians(30f)) + 0.02f, -3f), tilt, float3.zero);
            int sticker = b.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0f, 0.5f / math.cos(math.radians(30f)) + 0.02f, 3f), tilt, float3.zero);
            Run(b, 180);
            var sl = b.Bodies[slider]; var st = b.Bodies[sticker];
            Assert.Greater(sl.positionLin.x, 1.5f, "the low-friction cube slid down the slope (towards +x)");
            Assert.Less(math.abs(st.positionLin.x), 0.05f, "the high-friction cube stayed");
            Assert.Less(math.length(st.velocityLin), 0.02f, "and is at rest");
            var m = TerrainManifold(b, st);
            Assert.IsNotNull(m);
            float3 expected = math.normalize(new float3(slope, 1f, 0f));
            Assert.Less(math.distance(m.basis.r0, expected), 1e-4f, "the manifold normal is the normal of the slope");
        }

        /// <summary>A long box laid across a rounded ridge rests on the crest through its mid-face points, not sunk down to
        /// where its corners would touch (the corners hang free half a metre above the flanks).</summary>
        [Test]
        public void ReferenceLongBoxRestsOnTheCrestOfARidge()
        {
            const float A = 1f, w = 2.5f;
            var field = new Heightfield(65, 65, new float2(0.5f, 0.5f), new float2(-16f, -16f));
            for (int z = 0; z < 65; z++)
                for (int x = 0; x < 65; x++)
                {
                    float px = -16f + x * 0.5f;
                    field[x, z] = A * math.exp(-(px / w) * (px / w));   // a ridge along z
                }
            field.BuildMaxMip();
            var b = new RefSceneBuilder(new Solver());
            b.SetTerrain(field, 0.6f);
            int box = b.AddBody(new float3(4f, 0.5f, 1f), 1f, 0.6f, new float3(0f, A + 0.25f + 0.3f, 0f), quaternion.identity, float3.zero);
            Run(b, 240);
            var body = b.Bodies[box];
            Assert.Less(math.length(body.velocityLin), 0.02f, "at rest");
            Assert.AreEqual(A + 0.25f - RefConstants.CollisionMargin, body.positionLin.y, 0.02f, "the bottom face centre rests on the crest");
            Assert.Less(math.length(body.positionAng - Quat.Identity), 0.02f, "and the box lies level");
            var m = TerrainManifold(b, body);
            Assert.IsNotNull(m);
            for (int i = 0; i < m.numContacts; i++)
            {
                int k = m.contacts[i].feature & 0xFF;
                Assert.AreEqual(0f, RefCollide.LatticePoint(k).x, $"contact {i} (lattice {k}) lies on the crest line");
            }
            Assert.AreEqual(3, m.numContacts, "the face centre and the two long-edge midpoints");
        }

        [Test]
        public void ReferenceTerrainSceneRunsAndSettles()
        {
            var b = RefSceneBuilder.Build(AvbdScenes.Terrain);
            Assert.AreEqual(101, b.Bodies.Count);
            Assert.IsTrue(b.Bodies[0].terrain);
            Run(b, 300);
            var field = AvbdScenes.TerrainField();
            int resting = 0;
            for (int i = 1; i < b.Bodies.Count; i++)
            {
                var body = b.Bodies[i];
                Assert.IsTrue(math.all(math.isfinite(body.positionLin)), $"body {i} finite");
                float h = field.Height(body.positionLin.xz);
                Assert.Greater(body.positionLin.y, h - 0.5f, $"body {i} did not fall through the terrain");
                Assert.Less(body.positionLin.y, h + 4f, $"body {i} came down");
                if (math.length(body.velocityLin) < 0.05f) resting++;
            }
            Assert.Greater(resting, 60, "most boxes have come to rest on the hills");
        }

        [Test]
        public void NormalizedRoundTripUsesUnitysLayout()
        {
            var n = new float[3, 4];   // [z, x]: 3 rows of 4
            for (int z = 0; z < 3; z++) for (int x = 0; x < 4; x++) n[z, x] = (z * 4 + x) / 11f;
            var f = Heightfield.FromNormalized(n, new float2(1f, 1f), float2.zero, 22f, baseHeight: 5f);
            Assert.AreEqual(4, f.ResX); Assert.AreEqual(3, f.ResZ);
            Assert.AreEqual(5f + 2f * 7f, f[3, 1], 1e-5f, "[z = 1, x = 3] is sample (3, 1)");
            var back = f.ToNormalized(22f);
            for (int z = 0; z < 3; z++) for (int x = 0; x < 4; x++) Assert.AreEqual(n[z, x], back[z, x], 1e-6f);
        }
    }
}
