// The heightfield on the GPU: the mirror of Heightfield.cs (Sample, MaxOver) and of RefCollide.LatticePoint, operation for
// operation, so that the terrain contacts of the GPU solver and of the CPU reference agree. Parameters come from the
// AvbdParams constant buffer (_Terrain*); the samples and the max mip are plain float buffers uploaded by AvbdGpuWorld.
#ifndef AVBD_TERRAIN_INCLUDED
#define AVBD_TERRAIN_INCLUDED

#include "AvbdCommon.hlsl"

#define TERRAIN_MIP_BLOCK 8         // cells per mip block along each axis (Heightfield.MipBlock)
#define TERRAIN_MAX_MIP_SPAN 4      // blocks a query may span before it answers with the global maximum (Heightfield.MaxMipSpan)
#define LATTICE_POINTS 26           // the corners, edge midpoints and face centres of a box (RefCollide.LatticePoints)
#define AXIS_TERRAIN 3              // feature key prefix of terrain contacts (RefCollide.AXIS_TERRAIN)
#define NO_TERRAIN 0xFFFFFFFFu

StructuredBuffer<float> _TerrainHeights;    // [z * _TerrainResX + x], world y
StructuredBuffer<float> _TerrainMaxMip;     // [bz * _TerrainMipX + bx], the highest sample of the block

float terrainHeightAt(int x, int z) { return _TerrainHeights[z * (int)_TerrainResX + x]; }

// dh/dx and dh/dz at a sample: the central difference, one-sided on the border (Heightfield.GradX / GradZ)
float terrainGradX(int x, int z)
{
    int x0 = max(x - 1, 0), x1 = min(x + 1, (int)_TerrainResX - 1);
    return (terrainHeightAt(x1, z) - terrainHeightAt(x0, z)) / ((x1 - x0) * _TerrainCell.x);
}

float terrainGradZ(int x, int z)
{
    int z0 = max(z - 1, 0), z1 = min(z + 1, (int)_TerrainResZ - 1);
    return (terrainHeightAt(x, z1) - terrainHeightAt(x, z0)) / ((z1 - z0) * _TerrainCell.y);
}

// Height and unit normal of the surface at a world xz: the bilinear patch of the cell, the normal from the samples' gradients
// interpolated the same way; beyond the border the terrain continues flat at the edge height (Heightfield.Sample).
void terrainSample(float2 xz, out float height, out float3 normal)
{
    float2 u = (xz - _TerrainOrigin) / _TerrainCell;
    int ix = clamp((int)floor(u.x), 0, (int)_TerrainResX - 2);
    int iz = clamp((int)floor(u.y), 0, (int)_TerrainResZ - 2);
    float fx = clamp(u.x - ix, 0.0, 1.0);
    float fz = clamp(u.y - iz, 0.0, 1.0);
    float h00 = terrainHeightAt(ix, iz), h10 = terrainHeightAt(ix + 1, iz);
    float h01 = terrainHeightAt(ix, iz + 1), h11 = terrainHeightAt(ix + 1, iz + 1);
    height = lerp(lerp(h00, h10, fx), lerp(h01, h11, fx), fz);
    float inX = u.x >= 0.0 && u.x <= (float)(_TerrainResX - 1) ? 1.0 : 0.0;
    float inZ = u.y >= 0.0 && u.y <= (float)(_TerrainResZ - 1) ? 1.0 : 0.0;
    float dhdx = lerp(lerp(terrainGradX(ix, iz), terrainGradX(ix + 1, iz), fx), lerp(terrainGradX(ix, iz + 1), terrainGradX(ix + 1, iz + 1), fx), fz) * inX;
    float dhdz = lerp(lerp(terrainGradZ(ix, iz), terrainGradZ(ix + 1, iz), fx), lerp(terrainGradZ(ix, iz + 1), terrainGradZ(ix + 1, iz + 1), fx), fz) * inZ;
    normal = normalize(float3(-dhdx, 1.0, -dhdz));
}

// Upper bound of the surface height over an xz rectangle (Heightfield.MaxOver): the max mip of the blocks the rectangle
// touches, or the global maximum when it spans more than TERRAIN_MAX_MIP_SPAN blocks per axis.
float terrainMaxOver(float2 mn, float2 mx)
{
    float2 block = _TerrainCell * TERRAIN_MIP_BLOCK;
    int bx0 = clamp((int)floor((mn.x - _TerrainOrigin.x) / block.x), 0, (int)_TerrainMipX - 1);
    int bx1 = clamp((int)floor((mx.x - _TerrainOrigin.x) / block.x), 0, (int)_TerrainMipX - 1);
    int bz0 = clamp((int)floor((mn.y - _TerrainOrigin.y) / block.y), 0, (int)_TerrainMipZ - 1);
    int bz1 = clamp((int)floor((mx.y - _TerrainOrigin.y) / block.y), 0, (int)_TerrainMipZ - 1);
    if (bx1 - bx0 >= TERRAIN_MAX_MIP_SPAN || bz1 - bz0 >= TERRAIN_MAX_MIP_SPAN) return _TerrainMaxHeight;
    float m = -3.4e38;
    for (int bz = bz0; bz <= bz1; bz++)
        for (int bx = bx0; bx <= bx1; bx++)
            m = max(m, _TerrainMaxMip[bz * (int)_TerrainMipX + bx]);
    return m;
}

// Lattice point k of a box in units of its half extents: the 8 corners (bits x, y, z), then the 12 edge midpoints (zero along
// axis (k - 8) / 4, the other two coordinates from the two low bits), then the 6 face centres (+-1 along axis (k - 20) / 2).
// The order breaks depth ties in the contact selection, so a face resting flat keeps its corners (RefCollide.LatticePoint).
float3 latticePoint(int k)
{
    if (k < 8)
        return float3((k & 1) != 0 ? 1.0 : -1.0, (k & 2) != 0 ? 1.0 : -1.0, (k & 4) != 0 ? 1.0 : -1.0);
    if (k < 20)
    {
        int e = k - 8, axis = e >> 2;
        float a = (e & 1) != 0 ? 1.0 : -1.0, b = (e & 2) != 0 ? 1.0 : -1.0;
        return axis == 0 ? float3(0.0, a, b) : axis == 1 ? float3(a, 0.0, b) : float3(a, b, 0.0);
    }
    int f = k - 20, faceAxis = f >> 1;
    float sgn = (f & 1) != 0 ? 1.0 : -1.0;
    return faceAxis == 0 ? float3(sgn, 0.0, 0.0) : faceAxis == 1 ? float3(0.0, sgn, 0.0) : float3(0.0, 0.0, sgn);
}

#endif
