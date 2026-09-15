using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The heightfield: bilinear sampling and its normal, the flat continuation beyond the border, the max mip, the
    /// plateau editing and the procedural presets.</summary>
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
