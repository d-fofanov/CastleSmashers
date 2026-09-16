// The terrain both solvers collide with: a regular grid of heights over the xz plane. The surface between samples is the
// bilinear patch of the cell (Unity's TerrainData.GetInterpolatedHeight); the normal comes from the samples' central-difference
// gradients interpolated the same way (Unity's GetInterpolatedNormal), so it is continuous across the cell borders where the
// bilinear surface itself has a crease - a contact point on a sample line would otherwise get the one-sided slope of one
// neighbouring cell. Beyond the border the terrain continues flat at the edge height (nothing falls off the world).
// AvbdTerrain.hlsl mirrors Sample() and MaxOver() operation for operation.

using System;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Scenes
{
    /// <summary>Procedural terrains of the demos and tests (<see cref="Heightfield.Generate"/>).</summary>
    public enum TerrainPreset { None = 0, Hills = 1, Valley = 2, Ridge = 3 }

    /// <summary>A heightfield: ResX x ResZ world-unit heights (row-major, [z * ResX + x]) spaced Cell apart from Origin, plus a
    /// max-height mip of <see cref="MipBlock"/>-cell blocks for the broadphase early-out.</summary>
    public sealed class Heightfield
    {
        /// <summary>Cells per max-mip block along each axis (AvbdTerrain.hlsl TERRAIN_MIP_BLOCK).</summary>
        public const int MipBlock = 8;
        /// <summary>Blocks of the mip a body may span before the query gives up and answers with the global maximum.</summary>
        public const int MaxMipSpan = 4;

        public readonly int ResX, ResZ;
        /// <summary>Sample spacing (m) along x and z.</summary>
        public readonly float2 Cell;
        /// <summary>World xz of sample (0, 0).</summary>
        public readonly float2 Origin;
        /// <summary>World y of every sample, [z * ResX + x].</summary>
        public readonly float[] Heights;
        /// <summary>Maximum sample height of every block of MipBlock x MipBlock cells (the blocks share their edge samples), [bz * MipX + bx].</summary>
        public readonly float[] MaxMip;
        public readonly int MipX, MipZ;
        public float MinHeight { get; private set; }
        public float MaxHeight { get; private set; }

        public Heightfield(int resX, int resZ, float2 cell, float2 origin, float[] heights = null)
        {
            if (resX < 2 || resZ < 2) throw new ArgumentException("Heightfield: at least 2 x 2 samples");
            if (!(cell.x > 0f) || !(cell.y > 0f)) throw new ArgumentException("Heightfield: the cell size must be positive");
            if (heights != null && heights.Length != resX * resZ) throw new ArgumentException("Heightfield: heights must hold ResX x ResZ samples");
            ResX = resX; ResZ = resZ; Cell = cell; Origin = origin;
            Heights = heights ?? new float[resX * resZ];
            MipX = (resX - 2) / MipBlock + 1;
            MipZ = (resZ - 2) / MipBlock + 1;
            MaxMip = new float[MipX * MipZ];
            BuildMaxMip();
        }

        /// <summary>World extent covered by the samples (the last sample lies at Origin + Extent).</summary>
        public float2 Extent => new float2((ResX - 1) * Cell.x, (ResZ - 1) * Cell.y);
        public float2 Center => Origin + Extent * 0.5f;
        public int SampleCount => ResX * ResZ;

        public float this[int x, int z]
        {
            get => Heights[z * ResX + x];
            set => Heights[z * ResX + x] = value;
        }

        /// <summary>A flat field at the given height.</summary>
        public static Heightfield Flat(int resX, int resZ, float2 cell, float2 origin, float height)
        {
            var h = new float[resX * resZ];
            for (int i = 0; i < h.Length; i++) h[i] = height;
            return new Heightfield(resX, resZ, cell, origin, h);
        }

        /// <summary>From normalised samples in Unity's TerrainData.GetHeights layout ([z, x], 0 .. 1 of <paramref name="heightScale"/>);
        /// <paramref name="baseHeight"/> is the terrain's world y.</summary>
        public static Heightfield FromNormalized(float[,] normalized, float2 cell, float2 origin, float heightScale, float baseHeight = 0f)
        {
            int resZ = normalized.GetLength(0), resX = normalized.GetLength(1);
            var h = new float[resX * resZ];
            for (int z = 0; z < resZ; z++)
                for (int x = 0; x < resX; x++)
                    h[z * resX + x] = normalized[z, x] * heightScale + baseHeight;
            return new Heightfield(resX, resZ, cell, origin, h);
        }

        /// <summary>The samples as Unity's normalised [z, x] array (0 = <see cref="MinHeight"/>, 1 = MinHeight + <paramref name="heightScale"/>).</summary>
        public float[,] ToNormalized(float heightScale)
        {
            var n = new float[ResZ, ResX];
            float inv = heightScale > 0f ? 1f / heightScale : 0f;
            for (int z = 0; z < ResZ; z++)
                for (int x = 0; x < ResX; x++)
                    n[z, x] = math.saturate((Heights[z * ResX + x] - MinHeight) * inv);
            return n;
        }

        // ------------------------------------------------------------------------------------------------ sampling

        /// <summary>Height and unit normal of the surface at a world xz (bilinear patch, interpolated sample gradients; flat
        /// continuation beyond the border).</summary>
        public void Sample(float2 xz, out float height, out float3 normal)
        {
            float2 u = (xz - Origin) / Cell;
            int ix = math.clamp((int)math.floor(u.x), 0, ResX - 2);
            int iz = math.clamp((int)math.floor(u.y), 0, ResZ - 2);
            float fx = math.clamp(u.x - ix, 0f, 1f);
            float fz = math.clamp(u.y - iz, 0f, 1f);
            float h00 = Heights[iz * ResX + ix], h10 = Heights[iz * ResX + ix + 1];
            float h01 = Heights[(iz + 1) * ResX + ix], h11 = Heights[(iz + 1) * ResX + ix + 1];
            height = math.lerp(math.lerp(h00, h10, fx), math.lerp(h01, h11, fx), fz);
            // the border continues flat: no slope across an axis the point lies outside of
            float inX = u.x >= 0f && u.x <= ResX - 1 ? 1f : 0f;
            float inZ = u.y >= 0f && u.y <= ResZ - 1 ? 1f : 0f;
            float dhdx = math.lerp(math.lerp(GradX(ix, iz), GradX(ix + 1, iz), fx), math.lerp(GradX(ix, iz + 1), GradX(ix + 1, iz + 1), fx), fz) * inX;
            float dhdz = math.lerp(math.lerp(GradZ(ix, iz), GradZ(ix + 1, iz), fx), math.lerp(GradZ(ix, iz + 1), GradZ(ix + 1, iz + 1), fx), fz) * inZ;
            normal = math.normalize(new float3(-dhdx, 1f, -dhdz));
        }

        /// <summary>dh/dx at a sample: the central difference, one-sided on the border.</summary>
        float GradX(int x, int z)
        {
            int x0 = math.max(x - 1, 0), x1 = math.min(x + 1, ResX - 1);
            return (Heights[z * ResX + x1] - Heights[z * ResX + x0]) / ((x1 - x0) * Cell.x);
        }

        float GradZ(int x, int z)
        {
            int z0 = math.max(z - 1, 0), z1 = math.min(z + 1, ResZ - 1);
            return (Heights[z1 * ResX + x] - Heights[z0 * ResX + x]) / ((z1 - z0) * Cell.y);
        }

        public float Height(float2 xz)
        {
            Sample(xz, out float h, out _);
            return h;
        }

        /// <summary>Upper bound of the surface height over the xz rectangle, from the mip (the global maximum when the rectangle
        /// spans more than <see cref="MaxMipSpan"/> blocks per axis).</summary>
        public float MaxOver(float2 min, float2 max)
        {
            float2 block = Cell * MipBlock;
            int bx0 = math.clamp((int)math.floor((min.x - Origin.x) / block.x), 0, MipX - 1);
            int bx1 = math.clamp((int)math.floor((max.x - Origin.x) / block.x), 0, MipX - 1);
            int bz0 = math.clamp((int)math.floor((min.y - Origin.y) / block.y), 0, MipZ - 1);
            int bz1 = math.clamp((int)math.floor((max.y - Origin.y) / block.y), 0, MipZ - 1);
            if (bx1 - bx0 >= MaxMipSpan || bz1 - bz0 >= MaxMipSpan) return MaxHeight;
            float m = float.NegativeInfinity;
            for (int bz = bz0; bz <= bz1; bz++)
                for (int bx = bx0; bx <= bx1; bx++)
                    m = math.max(m, MaxMip[bz * MipX + bx]);
            return m;
        }

        /// <summary>Recomputes the mip and the height bounds after the samples changed.</summary>
        public void BuildMaxMip()
        {
            float lo = float.PositiveInfinity, hi = float.NegativeInfinity;
            for (int bz = 0; bz < MipZ; bz++)
                for (int bx = 0; bx < MipX; bx++)
                {
                    int x0 = bx * MipBlock, x1 = math.min(x0 + MipBlock, ResX - 1);
                    int z0 = bz * MipBlock, z1 = math.min(z0 + MipBlock, ResZ - 1);
                    float m = float.NegativeInfinity;
                    for (int z = z0; z <= z1; z++)
                        for (int x = x0; x <= x1; x++)
                            m = math.max(m, Heights[z * ResX + x]);
                    MaxMip[bz * MipX + bx] = m;
                    hi = math.max(hi, m);
                }
            for (int i = 0; i < Heights.Length; i++) lo = math.min(lo, Heights[i]);
            MinHeight = lo; MaxHeight = hi;
        }

        /// <summary>The same surface sampled on another grid over the same extent (bilinear: exact when refining a bilinear surface).</summary>
        public Heightfield Resampled(int resX, int resZ)
        {
            var cell = new float2(Extent.x / (resX - 1), Extent.y / (resZ - 1));
            var h = new float[resX * resZ];
            for (int z = 0; z < resZ; z++)
                for (int x = 0; x < resX; x++)
                    h[z * resX + x] = Height(Origin + new float2(x * cell.x, z * cell.y));
            return new Heightfield(resX, resZ, cell, Origin, h);
        }

        // ------------------------------------------------------------------------------------------------ editing

        /// <summary>Mean sample height over the xz rectangle (the samples inside it, or the nearest one).</summary>
        public float MeanHeight(float2 min, float2 max)
        {
            int x0 = math.clamp((int)math.ceil((min.x - Origin.x) / Cell.x), 0, ResX - 1), x1 = math.clamp((int)math.floor((max.x - Origin.x) / Cell.x), 0, ResX - 1);
            int z0 = math.clamp((int)math.ceil((min.y - Origin.y) / Cell.y), 0, ResZ - 1), z1 = math.clamp((int)math.floor((max.y - Origin.y) / Cell.y), 0, ResZ - 1);
            if (x1 < x0 || z1 < z0) return Height((min + max) * 0.5f);
            double sum = 0; int n = 0;
            for (int z = z0; z <= z1; z++)
                for (int x = x0; x <= x1; x++) { sum += Heights[z * ResX + x]; n++; }
            return (float)(sum / n);
        }

        /// <summary>Lowest and highest sample over the xz rectangle (the samples inside it, or the nearest one): exact where
        /// <see cref="MaxOver"/> answers for whole mip blocks. Ground is level over the rectangle when the two are equal.</summary>
        public void SampleRange(float2 min, float2 max, out float lo, out float hi)
        {
            int x0 = math.clamp((int)math.ceil((min.x - Origin.x) / Cell.x), 0, ResX - 1), x1 = math.clamp((int)math.floor((max.x - Origin.x) / Cell.x), 0, ResX - 1);
            int z0 = math.clamp((int)math.ceil((min.y - Origin.y) / Cell.y), 0, ResZ - 1), z1 = math.clamp((int)math.floor((max.y - Origin.y) / Cell.y), 0, ResZ - 1);
            if (x1 < x0 || z1 < z0) { lo = hi = Height((min + max) * 0.5f); return; }
            lo = float.PositiveInfinity; hi = float.NegativeInfinity;
            for (int z = z0; z <= z1; z++)
                for (int x = x0; x <= x1; x++) { float h = Heights[z * ResX + x]; lo = math.min(lo, h); hi = math.max(hi, h); }
        }

        /// <summary>Levels the xz rectangle at <paramref name="height"/> (a plateau for a castle) and blends back into the terrain
        /// over <paramref name="skirt"/> metres outside it. Only the samples within the skirt are visited (a scene may cut a
        /// hundred small terraces).</summary>
        public void Flatten(float2 min, float2 max, float height, float skirt)
        {
            int x0 = math.clamp((int)math.floor((min.x - skirt - Origin.x) / Cell.x), 0, ResX - 1), x1 = math.clamp((int)math.ceil((max.x + skirt - Origin.x) / Cell.x), 0, ResX - 1);
            int z0 = math.clamp((int)math.floor((min.y - skirt - Origin.y) / Cell.y), 0, ResZ - 1), z1 = math.clamp((int)math.ceil((max.y + skirt - Origin.y) / Cell.y), 0, ResZ - 1);
            for (int z = z0; z <= z1; z++)
                for (int x = x0; x <= x1; x++)
                {
                    float2 p = Origin + new float2(x * Cell.x, z * Cell.y);
                    float2 d = math.max(math.max(min - p, p - max), 0f);   // distance to the rectangle, 0 inside
                    float dist = math.length(d);
                    if (dist >= skirt) continue;
                    float t = skirt > 0f ? math.smoothstep(0f, 1f, dist / skirt) : 0f;
                    Heights[z * ResX + x] = math.lerp(height, Heights[z * ResX + x], t);
                }
            BuildMaxMip();
        }

        // ------------------------------------------------------------------------------------------------ generation

        /// <summary>A procedural terrain centred on the world origin: <paramref name="res"/> samples per axis <paramref name="cell"/> metres
        /// apart, relief of about <paramref name="amplitude"/> metres with features <paramref name="featureSize"/> metres across.
        /// Deterministic in the seed (simplex noise of Unity.Mathematics).</summary>
        public static Heightfield Generate(TerrainPreset preset, uint seed, int res, float cell, float amplitude, float featureSize = 40f)
        {
            float extent = (res - 1) * cell;
            var field = new Heightfield(res, res, new float2(cell, cell), new float2(-extent * 0.5f, -extent * 0.5f));
            if (preset == TerrainPreset.None) return field;
            float2 shift = new float2(seed % 977u * 3.7f, seed / 977u % 991u * 5.3f);   // a different patch of the noise field per seed
            for (int z = 0; z < res; z++)
                for (int x = 0; x < res; x++)
                {
                    float2 p = field.Origin + new float2(x * cell, z * cell);
                    float rolling = Fbm(p / featureSize + shift, 4);   // roughly -1 .. 1
                    float h;
                    switch (preset)
                    {
                        case TerrainPreset.Valley:
                        {
                            // a valley floor along x, slopes rising to a rim on both sides, hills on top
                            float side = math.smoothstep(0.15f, 0.55f, math.abs(p.y) / (extent * 0.5f));
                            h = amplitude * (0.9f * side + 0.3f * rolling);
                            break;
                        }
                        case TerrainPreset.Ridge:
                        {
                            // a ridge along z through the centre, hills around it
                            float rx = p.x / (0.18f * extent);
                            float crest = math.exp(-rx * rx);
                            h = amplitude * (0.9f * crest + 0.3f * rolling);
                            break;
                        }
                        default:
                            h = amplitude * 0.6f * rolling;
                            break;
                    }
                    field.Heights[z * res + x] = h;
                }
            field.BuildMaxMip();
            return field;
        }

        static float Fbm(float2 p, int octaves)
        {
            float sum = 0f, amp = 0.5f, norm = 0f;
            for (int i = 0; i < octaves; i++)
            {
                sum += amp * noise.snoise(p);
                norm += amp;
                p = p * 2.03f + new float2(17.1f, 9.7f);
                amp *= 0.5f;
            }
            return sum / norm;
        }
    }
}
