using System.Collections.Generic;
using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdRef;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The heightfield: bilinear sampling and its normal, the flat continuation beyond the border, the max mip, the
    /// plateau editing and the procedural presets; then the terrain contacts of the CPU reference (the box's lattice points
    /// against the surface), and the GPU's CollideTerrain kernel against that reference.</summary>
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
        public void CraterDropsTheSamplesWithinTheRadiusByAParaboloid()
        {
            var f = Heightfield.Generate(TerrainPreset.Hills, 5u, 129, 1f, 8f, 30f);
            var before = (float[])f.Heights.Clone();
            float2 centre = new float2(3f, -5f);
            f.Sample(centre, out _, out float3 slope);
            f.Crater(centre, 6f, 4f);
            for (int z = 0; z < f.ResZ; z++)
                for (int x = 0; x < f.ResX; x++)
                {
                    float2 p = f.Origin + new float2(x * f.Cell.x, z * f.Cell.y);
                    float r = math.distance(p, centre) / 6f;
                    float expected = r >= 1f ? before[z * f.ResX + x] : before[z * f.ResX + x] - 4f * (1f - r * r);
                    Assert.AreEqual(expected, f[x, z], 1e-5f, $"at {p}");
                }
            f.Sample(centre, out float h, out float3 n);
            Assert.AreEqual(before[(int)(centre.y - f.Origin.y) * f.ResX + (int)(centre.x - f.Origin.x)] - 4f, h, 1e-5f, "the centre (on a sample) dropped by the depth");
            Assert.Less(math.distance(n, slope), 1e-4f, "the paraboloid is level at its centre: the hill's own slope remains");
            float lowest = float.PositiveInfinity;
            foreach (float s in f.Heights) lowest = math.min(lowest, s);
            Assert.AreEqual(lowest, f.MinHeight, "the bounds follow the edit");
            var untouched = (float[])f.Heights.Clone();
            f.Crater(centre, 0f, 4f); f.Crater(centre, 6f, 0f);
            CollectionAssert.AreEqual(untouched, f.Heights, "a crater without a radius or a depth changes nothing");
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

        /// <summary>A unit cube set down 2 cm above a flat field at height 2 (a drop from height is caught up to a step of travel
        /// deep and creeps back up over seconds, on the terrain as on the ground box: alpha leaves 99 % of the initial error per step).</summary>
        static void BuildFlatRest(ISceneBuilder s)
        {
            int terrain = s.SetTerrain(Heightfield.Flat(17, 17, new float2(4f, 4f), new float2(-32f, -32f), 2f), 0.5f);
            int box = s.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0.3f, 2.52f, -0.2f), quaternion.identity, float3.zero);
            Assert.AreEqual(0, terrain); Assert.AreEqual(1, box);
        }

        /// <summary>The static-friction scene as a heightfield: a 30 degree slope of friction 1, cubes of friction 0.25 (combined 0.5,
        /// below tan 30 = 0.58: slides, body 1) and 0.5 (combined 0.71: sticks, body 2).</summary>
        static void BuildSlope(ISceneBuilder s)
        {
            float slope = math.tan(math.radians(30f));
            var field = new Heightfield(41, 41, new float2(1f, 1f), new float2(-20f, -20f));
            for (int z = 0; z < 41; z++) for (int x = 0; x < 41; x++) field[x, z] = (x - 20) * slope * -1f;   // rising towards -x
            field.BuildMaxMip();
            s.SetTerrain(field, 1f);
            quaternion tilt = quaternion.RotateZ(-math.radians(30f));   // the bottom face of the cube on the slope
            s.AddBody(new float3(1, 1, 1), 1f, 0.25f, new float3(0f, 0.5f / math.cos(math.radians(30f)) + 0.02f, -3f), tilt, float3.zero);
            s.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0f, 0.5f / math.cos(math.radians(30f)) + 0.02f, 3f), tilt, float3.zero);
        }

        const float RidgeHeight = 1f, RidgeWidth = 2.5f;

        /// <summary>A 4 x 0.5 x 1 box laid across a rounded ridge (a Gaussian along z), body 1.</summary>
        static void BuildRidge(ISceneBuilder s)
        {
            var field = new Heightfield(65, 65, new float2(0.5f, 0.5f), new float2(-16f, -16f));
            for (int z = 0; z < 65; z++)
                for (int x = 0; x < 65; x++)
                {
                    float px = -16f + x * 0.5f;
                    field[x, z] = RidgeHeight * math.exp(-(px / RidgeWidth) * (px / RidgeWidth));
                }
            field.BuildMaxMip();
            s.SetTerrain(field, 0.6f);
            s.AddBody(new float3(4f, 0.5f, 1f), 1f, 0.6f, new float3(0f, RidgeHeight + 0.25f + 0.3f, 0f), quaternion.identity, float3.zero);
        }

        [Test]
        public void ReferenceBoxRestsOnAFlatFieldOneMarginDeepOnEightPoints()
        {
            var b = new RefSceneBuilder(new Solver());
            BuildFlatRest(b);
            const int box = 1;
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
            var b = new RefSceneBuilder(new Solver());
            BuildSlope(b);
            const int slider = 1, sticker = 2;
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
            var b = new RefSceneBuilder(new Solver());
            BuildRidge(b);
            const int box = 1;
            Run(b, 240);
            var body = b.Bodies[box];
            Assert.Less(math.length(body.velocityLin), 0.02f, "at rest");
            Assert.AreEqual(RidgeHeight + 0.25f - RefConstants.CollisionMargin, body.positionLin.y, 0.02f, "the bottom face centre rests on the crest");
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

        // ------------------------------------------------------------------------------------------------ GPU

        /// <summary>The same custom scene on the GPU world and on the reference (in the GPU's pair convention), stepped together.</summary>
        static GpuTestUtil.Comparison RunBoth(System.Action<ISceneBuilder> build, int steps, AvbdGpuWorld world, out RefSceneBuilder reference)
        {
            world.Clear();
            build(world);
            reference = new RefSceneBuilder(new Solver());
            reference.Solver.LowIndexFirst = true;
            build(reference);
            for (int i = 0; i < steps; i++) { world.Step(); reference.Solver.Step(); }
            return GpuTestUtil.Compare(world, reference);
        }

        [Test]
        public void GpuBoxRestsOnAFlatFieldLikeTheReference()
        {
            using var world = GpuTestUtil.NewWorld();
            var c = RunBoth(BuildFlatRest, 300, world, out _);
            GpuTestUtil.Log("terrain flat rest", c);
            Assert.Less(c.MaxPositionError, 1e-4f, $"position (body {c.WorstBody})");
            Assert.Less(c.MaxRotationError, 1e-4f, "rotation");
            var stats = world.GetStatsSync();
            Assert.AreEqual(1, stats.Manifolds); Assert.AreEqual(1, stats.TerrainManifolds); Assert.AreEqual(0, stats.OverflowFlags);
            var manifolds = world.GetManifoldsSync(out var contacts);
            Assert.AreEqual(1u, manifolds[0].BodyA, "the box is A"); Assert.AreEqual((uint)world.TerrainBody, manifolds[0].BodyB, "the terrain slot is B");
            Assert.AreEqual(8, (int)manifolds[0].NumContacts);
            Assert.AreEqual(1f, manifolds[0].Normal.y, 1e-6f);
            float weight = 0f;
            for (uint i = 0; i < 8; i++)
            {
                var ct = contacts[manifolds[0].ContactStart + i];
                Assert.AreEqual((uint)RefCollide.AXIS_TERRAIN, ct.FeatureKey >> 24, "terrain feature prefix");
                Assert.Less(ct.FeatureKey & 0xFFu, 20u, "corners and edge midpoints");
                Assert.IsTrue(ct.Stick, "a resting contact sticks");
                weight += -ct.Lambda.x;
            }
            Assert.AreEqual(10f, weight, 1f, "the multipliers carry the weight");
        }

        [Test]
        public void GpuCubesOnTheSlopeMatchTheReference()
        {
            using var world = GpuTestUtil.NewWorld();
            var c = RunBoth(BuildSlope, 180, world, out var reference);
            GpuTestUtil.Log("terrain slope", c);
            Assert.Less(c.MaxPositionError, 2e-3f, $"position (body {c.WorstBody})");
            Assert.Less(c.MaxRotationError, 2e-3f, "rotation");
            world.GetPosesSync(out var pos, out _);
            Assert.Greater(pos[1].x, 1.5f, "the low-friction cube slid");
            Assert.Less(math.abs(pos[2].x), 0.05f, "the high-friction cube stayed");
        }

        [Test]
        public void GpuLongBoxOnTheRidgeMatchesTheReference()
        {
            using var world = GpuTestUtil.NewWorld();
            var c = RunBoth(BuildRidge, 240, world, out _);
            GpuTestUtil.Log("terrain ridge", c);
            Assert.Less(c.MaxPositionError, 1e-3f, $"position (body {c.WorstBody})");
            Assert.Less(c.MaxRotationError, 1e-3f, "rotation");
            var manifolds = world.GetManifoldsSync(out var contacts);
            Assert.AreEqual(1, world.GetStatsSync().TerrainManifolds);
            Assert.AreEqual(3, (int)manifolds[0].NumContacts, "the face centre and the two long-edge midpoints");
            for (uint i = 0; i < 3; i++)
                Assert.AreEqual(0f, RefCollide.LatticePoint((int)(contacts[manifolds[0].ContactStart + i].FeatureKey & 0xFFu)).x, "on the crest line");
        }

        /// <summary>Only bodies whose AABB reaches the surface's max mip get a terrain manifold: hovering boxes (no gravity) none,
        /// boxes set on the field one each.</summary>
        [Test]
        public void GpuTerrainEarlyOutSkipsBodiesAboveTheSurface()
        {
            using var world = GpuTestUtil.NewWorld();
            world.Params.Gravity = float3.zero;
            world.SetTerrain(Heightfield.Generate(TerrainPreset.Hills, 3u, 65, 2f, 5f), 0.5f);
            var field = world.Terrain;
            for (int i = 0; i < 10; i++) world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(-20f + i * 4f, field.MaxHeight + 3f, 5f), quaternion.identity, float3.zero);
            for (int i = 0; i < 5; i++)
            {
                float2 xz = new float2(-16f + i * 8f, -7f);
                world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(xz.x, field.Height(xz) + 0.5f - 0.005f, xz.y), quaternion.identity, float3.zero);
            }
            world.Step();
            var stats = world.GetStatsSync();
            Assert.AreEqual(5, stats.TerrainManifolds, "one manifold per box on the surface, none for the hovering ones");
            Assert.AreEqual(5, stats.Manifolds);
            Assert.AreEqual(0, stats.Pairs, "the terrain slot pairs with nothing in the grid");
            Assert.AreEqual(0, stats.LargeBodies, "and is not a large body");
            GpuTestUtil.AssertFinite(world);
        }

        [Test]
        public void GpuTerrainSceneIsBitwiseDeterministic()
        {
            using var a = GpuTestUtil.NewWorld();
            using var b = GpuTestUtil.NewWorld();
            a.BuildScene(AvbdScenes.Terrain);
            b.BuildScene(AvbdScenes.Terrain);
            for (int i = 0; i < 90; i++) { a.Step(); b.Step(); }
            a.GetPosesSync(out var pa, out var ra);
            b.GetPosesSync(out var pb, out var rb);
            for (int i = 0; i < a.BodyCount; i++)
            {
                Assert.IsTrue(math.all(pa[i] == pb[i]), $"body {i} position differs: {pa[i]} vs {pb[i]}");
                Assert.IsTrue(math.all(ra[i] == rb[i]), $"body {i} rotation differs");
            }
            Assert.Greater(a.GetStatsSync().TerrainManifolds, 50, "most boxes are down on the hills after 1.5 s");
        }

        // ------------------------------------------------------------------------------------------------ tiled view layout

        /// <summary>The tiled look: tiles of three sizes (2 m within 24 m of the centre, 4 m within 48 m, 8 m beyond) cover the
        /// field exactly once, every top is the field's height at the tile centre rounded to the step, and wherever two tiles meet
        /// the higher one's sides reach down to the lower one (plus the chamfer), so no level change shows a gap.</summary>
        [Test]
        public void TileLayoutCoversTheFieldWithRoundedTopsAndClosedRisers()
        {
            var field = Heightfield.Generate(TerrainPreset.Hills, 3u, 65, 2f, 8f, 30f);   // 128 x 128 m
            var s = Phys.AvbdGpu.Presentation.TerrainTiles.Settings.Default;
            s.TileSize = 2f; s.Step = 0.5f; s.DetailRadius = 24f; s.Bevel = 0.05f;
            var tiles = Phys.AvbdGpu.Presentation.TerrainTiles.Layout(field, s);
            int fine = 0, mid = 0, coarse = 0;
            double area = 0;
            var cover = new Dictionary<(int, int), int>();   // fine cell -> tile index
            for (int i = 0; i < tiles.Length; i++)
            {
                var t = tiles[i];
                if (t.Size == 2f) fine++; else if (t.Size == 4f) mid++; else { Assert.AreEqual(8f, t.Size); coarse++; }
                area += t.Size * t.Size;
                float expected = math.round(field.Height(t.Position.xz) / s.Step) * s.Step;
                Assert.AreEqual(expected, t.Position.y, 1e-4f, $"tile {i} top is the rounded height at its centre");
                Assert.LessOrEqual(math.abs(t.Position.y - field.Height(t.Position.xz)), s.Step * 0.5f + 1e-4f);
                Assert.GreaterOrEqual(t.Depth, s.Bevel * t.Size - 1e-5f, $"tile {i} sides at least a chamfer deep");
                int n = (int)math.round(t.Size / 2f);
                int x0 = (int)math.round((t.Position.x - t.Size * 0.5f - field.Origin.x) / 2f), z0 = (int)math.round((t.Position.z - t.Size * 0.5f - field.Origin.y) / 2f);
                for (int dz = 0; dz < n; dz++)
                    for (int dx = 0; dx < n; dx++)
                    {
                        Assert.IsFalse(cover.ContainsKey((x0 + dx, z0 + dz)), $"fine cell ({x0 + dx}, {z0 + dz}) covered twice");
                        cover[(x0 + dx, z0 + dz)] = i;
                    }
            }
            Assert.AreEqual(64 * 64, cover.Count, "every fine cell covered once");
            Assert.AreEqual(128.0 * 128.0, area, 1e-3, "the tiles cover the field exactly");
            Assert.AreEqual(24 * 24, fine, "a 48 x 48 m detail square of 2 m tiles");
            Assert.AreEqual(24 * 24 - 12 * 12, mid, "4 m tiles out to 96 x 96 m");
            Assert.AreEqual(16 * 16 - 12 * 12, coarse, "8 m tiles everywhere else");
            // risers: across every fine-cell edge the higher tile reaches down to the lower one
            int risers = 0;
            foreach (var kv in cover)
            {
                var (x, z) = kv.Key;
                foreach (var (nx, nz) in new[] { (x + 1, z), (x, z + 1) })
                {
                    if (!cover.TryGetValue((nx, nz), out int j) || j == kv.Value) continue;
                    var a = tiles[kv.Value]; var b = tiles[j];
                    var hi = a.Position.y >= b.Position.y ? a : b; var lo = a.Position.y >= b.Position.y ? b : a;
                    Assert.GreaterOrEqual(hi.Depth, hi.Position.y - lo.Position.y + s.Bevel * hi.Size - 1e-4f, $"tile at {hi.Position} reaches down to its neighbour at {lo.Position}");
                    if (hi.Position.y > lo.Position.y) risers++;
                }
            }
            Assert.Greater(risers, 100, "the hills have level changes");
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
