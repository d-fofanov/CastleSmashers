// Shared by the passes of AvbdTiles.shader: the tile buffer and the vertex decode. Every draw is Graphics.RenderMeshPrimitives
// with one instance of the bevelled tile mesh per tile; the mesh's x, z are in tile units, its height comes from uv.x
// (0 the top, 1 the chamfer's lower ring at -bevel, 2 the bottom of the sides at -depth) and uv.y shades the chamfer.
#ifndef AVBD_TILE_INSTANCING_INCLUDED
#define AVBD_TILE_INSTANCING_INCLUDED

struct Tile
{
    float3 pos;         // centre of the top
    float size;         // edge (m)
    float depth;        // how far the sides reach down (m)
    uint color;         // RGBA8
    float2 pad;
};

StructuredBuffer<Tile> _Tiles;

// Per-draw (MaterialPropertyBlock)
uint _InstanceOffset;
float _TileBevel;       // chamfer width and depth as a fraction of the edge

void tileVertex(uint instanceID, float3 positionOS, float3 normalOS, float2 uv, out float3 positionWS, out float3 normalWS, out float3 color)
{
    Tile t = _Tiles[instanceID + _InstanceOffset];
    float y = uv.x < 0.5 ? 0.0 : uv.x < 1.5 ? -_TileBevel * t.size : -t.depth;
    positionWS = t.pos + float3(positionOS.x * t.size, y, positionOS.z * t.size);
    normalWS = normalOS;
    color = float3(t.color & 255u, (t.color >> 8) & 255u, (t.color >> 16) & 255u) / 255.0 * uv.y;
}

#endif
