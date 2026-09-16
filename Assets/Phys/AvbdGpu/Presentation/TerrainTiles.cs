using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu.Presentation
{
    /// <summary>The heightfield drawn as if built from flat tiles: every tile's top is the field's height at its centre rounded to a
    /// step, a chamfer runs around the top (two neighbouring chamfers make the groove between pieces) and the sides reach down to
    /// the lowest neighbour, so a level change shows as a riser. One bevelled-box mesh drawn once per tile from an instance
    /// buffer (AvbdTiles.shader); tiles of the set size within the detail radius of the field's centre, twice as large out to
    /// twice that radius, four times as large beyond. The collision surface stays the smooth heightfield; this is only a look.</summary>
    public sealed class TerrainTiles : IDisposable
    {
        [Serializable]
        public struct Settings
        {
            /// <summary>Tile edge (m); the heightfield's cell when 0.</summary>
            public float TileSize;
            /// <summary>Height step the tile tops are rounded to (m).</summary>
            public float Step;
            /// <summary>Half-width of the square around the field's centre drawn with tiles of <see cref="TileSize"/>; out to twice it
            /// the tiles are twice as large, beyond that four times.</summary>
            public float DetailRadius;
            /// <summary>Chamfer width and depth as a fraction of the tile edge.</summary>
            public float Bevel;
            /// <summary>Tile colour by height: <see cref="Low"/> at the bottom of the field's range, <see cref="High"/> blended in from
            /// <see cref="HighFrom"/> to <see cref="HighTo"/> of it, with a per-tile brightness variation of +-<see cref="Variation"/>.</summary>
            public Color Low, High;
            public float HighFrom, HighTo, Variation;

            public static Settings Default => new Settings
            {
                TileSize = 0f, Step = 0.25f, DetailRadius = 200f, Bevel = 0.05f,
                Low = new Color32(74, 116, 52, 255), High = new Color32(132, 122, 70, 255), HighFrom = 0.45f, HighTo = 0.95f, Variation = 0.06f,
            };
        }

        /// <summary>One tile (AvbdTileInstancing.hlsl Tile): the centre of its top, its edge, how far its sides reach down, its colour.</summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct Tile
        {
            public float3 Position;
            public float Size;
            public float Depth;
            public uint Color;
            public float Pad0, Pad1;
            public const int Stride = 32;
        }

        readonly Mesh m_Mesh;
        readonly Material m_Material;
        readonly MaterialPropertyBlock m_Props = new MaterialPropertyBlock();
        GraphicsBuffer m_Buffer;
        Tile[] m_Tiles = new Tile[0];
        float m_Bevel;

        static readonly int s_Tiles = Shader.PropertyToID("_Tiles");
        static readonly int s_TileBevel = Shader.PropertyToID("_TileBevel");
        static readonly int s_InstanceOffset = Shader.PropertyToID("_InstanceOffset");

        public int Count => m_Tiles.Length;
        /// <summary>The tiles of the last <see cref="Build"/>.</summary>
        public Tile[] Tiles => m_Tiles;
        public int Layer;

        public TerrainTiles()
        {
            var shader = Resources.Load<Shader>("AvbdGpu/AvbdTiles");
            if (shader == null) throw new InvalidOperationException("AvbdGpu/AvbdTiles shader not found in Resources");
            m_Material = new Material(shader) { name = "AvbdTiles" };
            m_Mesh = new Mesh { name = "AvbdTile" };
        }

        public void Dispose()
        {
            m_Buffer?.Dispose(); m_Buffer = null;
            if (m_Mesh != null) UnityEngine.Object.Destroy(m_Mesh);
            if (m_Material != null) UnityEngine.Object.Destroy(m_Material);
        }

        /// <summary>Lays the tiles out for the field and uploads them.</summary>
        public void Build(Heightfield field, Settings s)
        {
            m_Tiles = Layout(field, s);
            m_Bevel = s.Bevel;
            BuildMesh(m_Mesh, s.Bevel);
            if (m_Buffer == null || m_Buffer.count < m_Tiles.Length)
            {
                m_Buffer?.Dispose();
                m_Buffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, math.max(m_Tiles.Length, 1), Tile.Stride);
            }
            if (m_Tiles.Length > 0) m_Buffer.SetData(m_Tiles);
        }

        /// <summary>Draws the tiles (once per frame).</summary>
        public void Render(Camera camera, bool shadows)
        {
            if (m_Tiles.Length == 0 || m_Buffer == null) return;
            m_Props.SetBuffer(s_Tiles, m_Buffer);
            m_Props.SetFloat(s_TileBevel, m_Bevel);
            m_Props.SetInteger(s_InstanceOffset, 0);
            var rp = new RenderParams(m_Material)
            {
                worldBounds = new Bounds(Vector3.zero, Vector3.one * 100000f),
                shadowCastingMode = shadows ? ShadowCastingMode.On : ShadowCastingMode.Off,
                receiveShadows = shadows,
                layer = Layer,
                camera = camera,
                matProps = m_Props,
            };
            Graphics.RenderMeshPrimitives(rp, m_Mesh, 0, m_Tiles.Length);
        }

        // ------------------------------------------------------------------------------------------------ layout

        /// <summary>The tiles of a field (pure: no GPU objects). A quadtree two levels deep: a coarse cell (four tile edges) whose
        /// centre lies within twice the detail radius of the field's centre splits into four, and a cell of two tile edges whose
        /// centre lies within the detail radius splits into four tiles, so nothing overlaps and the neighbours of a tile are found
        /// through the same rule; every tile's top is the rounded height at its centre and its sides reach down to the lowest
        /// neighbouring tile along each edge (plus the chamfer).</summary>
        public static Tile[] Layout(Heightfield field, Settings s)
        {
            float t = s.TileSize > 0f ? s.TileSize : math.min(field.Cell.x, field.Cell.y);
            float c = t * 4f;
            float step = math.max(s.Step, 1e-4f);
            int nx = math.max((int)math.floor(field.Extent.x / c + 1e-4f), 1), nz = math.max((int)math.floor(field.Extent.y / c + 1e-4f), 1);
            float range = math.max(field.MaxHeight - field.MinHeight, 1e-3f);
            var layout = new LayoutRule { Field = field, Origin = field.Origin, Centre = field.Center, Fine = t, Nx = nx, Nz = nz, DetailRadius = s.DetailRadius, Step = step };

            var tiles = new List<Tile>();
            for (int cj = 0; cj < nz; cj++)
                for (int ci = 0; ci < nx; ci++)
                {
                    float2 cc = layout.CellCentre(ci, cj, 4);
                    if (!layout.Within(cc, 2f * s.DetailRadius)) { tiles.Add(layout.Make(cc, c, s, range, (uint)(ci * 4), (uint)(cj * 4))); continue; }
                    for (int mj = 0; mj < 2; mj++)
                        for (int mi = 0; mi < 2; mi++)
                        {
                            int i2 = ci * 2 + mi, j2 = cj * 2 + mj;
                            float2 mc = layout.CellCentre(i2, j2, 2);
                            if (!layout.Within(mc, s.DetailRadius)) { tiles.Add(layout.Make(mc, 2f * t, s, range, (uint)(i2 * 2), (uint)(j2 * 2))); continue; }
                            for (int fj = 0; fj < 2; fj++)
                                for (int fi = 0; fi < 2; fi++)
                                {
                                    int i1 = i2 * 2 + fi, j1 = j2 * 2 + fj;
                                    tiles.Add(layout.Make(layout.CellCentre(i1, j1, 1), t, s, range, (uint)i1, (uint)j1));
                                }
                        }
                }
            return tiles.ToArray();
        }

        struct LayoutRule
        {
            public Heightfield Field;
            public float2 Origin, Centre;
            public float Fine, Step, DetailRadius;
            public int Nx, Nz;

            /// <summary>Centre of cell (i, j) of the grid whose cells are <paramref name="factor"/> tile edges wide.</summary>
            public float2 CellCentre(int i, int j, int factor) => Origin + new float2((i + 0.5f) * Fine * factor, (j + 0.5f) * Fine * factor);

            public bool Within(float2 p, float radius) => math.all(math.abs(p - Centre) <= radius);

            public float Quantize(float h) => math.round(h / Step) * Step;

            /// <summary>Top height of the tile covering an xz (NaN outside the tiled area): the quadtree rule of <see cref="Layout"/>.</summary>
            public float HeightAt(float2 p)
            {
                float2 u = (p - Origin) / (Fine * 4f);
                int ci = (int)math.floor(u.x), cj = (int)math.floor(u.y);
                if (ci < 0 || cj < 0 || ci >= Nx || cj >= Nz) return float.NaN;
                float2 centre = CellCentre(ci, cj, 4);
                if (Within(centre, 2f * DetailRadius))
                {
                    float2 m = math.floor((p - Origin) / (Fine * 2f));
                    centre = CellCentre((int)m.x, (int)m.y, 2);
                    if (Within(centre, DetailRadius))
                    {
                        float2 f = math.floor((p - Origin) / Fine);
                        centre = CellCentre((int)f.x, (int)f.y, 1);
                    }
                }
                return Quantize(Field.Height(centre));
            }

            public Tile Make(float2 centre, float size, Settings s, float range, uint ix, uint iz)
            {
                float h = Quantize(Field.Height(centre));
                // the lowest neighbouring tile along the four edges, sampled a fine tile apart
                float lowest = h;
                int samples = math.max((int)math.round(size / Fine), 1);
                for (int side = 0; side < 4; side++)
                {
                    float2 outward = side == 0 ? new float2(1, 0) : side == 1 ? new float2(-1, 0) : side == 2 ? new float2(0, 1) : new float2(0, -1);
                    float2 along = new float2(outward.y, outward.x);
                    for (int k = 0; k < samples; k++)
                    {
                        float2 p = centre + outward * (size * 0.5f + Fine * 0.5f) + along * (-size * 0.5f + Fine * (k + 0.5f));
                        float n = HeightAt(p);
                        if (!float.IsNaN(n)) lowest = math.min(lowest, n);
                    }
                }
                float bevel = s.Bevel * size;
                float tt = math.smoothstep(s.HighFrom, s.HighTo, (h - Field.MinHeight) / range);
                Color col = Color.Lerp(s.Low, s.High, tt);
                uint hash = Hash(ix * 73856093u ^ iz * 19349663u);
                float shade = 1f + s.Variation * ((hash & 1023u) / 511.5f - 1f);
                var c32 = (Color32)(col * shade);
                return new Tile
                {
                    Position = new float3(centre.x, h, centre.y), Size = size, Depth = h - lowest + bevel,
                    Color = (uint)c32.r | ((uint)c32.g << 8) | ((uint)c32.b << 16) | (255u << 24),
                };
            }

            static uint Hash(uint s)
            {
                s = (s ^ 61u) ^ (s >> 16); s *= 9u; s = s ^ (s >> 4); s *= 0x27d4eb2du; s = s ^ (s >> 15);
                return s;
            }
        }

        // ------------------------------------------------------------------------------------------------ mesh

        /// <summary>The bevelled box in tile units: x, z in -0.5 .. 0.5 (scaled by the tile edge), y encoded in uv.x (0 the top, 1 the
        /// chamfer's lower ring at -bevel, 2 the bottom of the sides at -depth), uv.y the shade (the chamfer a little darker).</summary>
        public static void BuildMesh(Mesh mesh, float bevel)
        {
            var verts = new List<Vector3>();
            var normals = new List<Vector3>();
            var uvs = new List<Vector2>();
            var tris = new List<int>();
            float inner = 0.5f - bevel;

            void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, float codeA, float codeB, float codeC, float codeD, Vector3 n, float shade)
            {
                int i = verts.Count;
                verts.Add(a); verts.Add(b); verts.Add(c); verts.Add(d);
                for (int k = 0; k < 4; k++) normals.Add(n);
                uvs.Add(new Vector2(codeA, shade)); uvs.Add(new Vector2(codeB, shade)); uvs.Add(new Vector2(codeC, shade)); uvs.Add(new Vector2(codeD, shade));
                tris.Add(i); tris.Add(i + 1); tris.Add(i + 2); tris.Add(i); tris.Add(i + 2); tris.Add(i + 3);
            }

            // the top (clockwise seen from above: Unity's front faces are clockwise)
            Quad(new Vector3(-inner, 0, -inner), new Vector3(-inner, 0, inner), new Vector3(inner, 0, inner), new Vector3(inner, 0, -inner), 0, 0, 0, 0, Vector3.up, 1f);
            // the chamfer ring: from the inner square (code 0) down and out to the outer square (code 1)
            float s45 = 0.70710678f;
            Quad(new Vector3(inner, 0, -inner), new Vector3(inner, 0, inner), new Vector3(0.5f, 0, 0.5f), new Vector3(0.5f, 0, -0.5f), 0, 0, 1, 1, new Vector3(s45, s45, 0), 0.82f);
            Quad(new Vector3(-inner, 0, inner), new Vector3(-inner, 0, -inner), new Vector3(-0.5f, 0, -0.5f), new Vector3(-0.5f, 0, 0.5f), 0, 0, 1, 1, new Vector3(-s45, s45, 0), 0.82f);
            Quad(new Vector3(inner, 0, inner), new Vector3(-inner, 0, inner), new Vector3(-0.5f, 0, 0.5f), new Vector3(0.5f, 0, 0.5f), 0, 0, 1, 1, new Vector3(0, s45, s45), 0.82f);
            Quad(new Vector3(-inner, 0, -inner), new Vector3(inner, 0, -inner), new Vector3(0.5f, 0, -0.5f), new Vector3(-0.5f, 0, -0.5f), 0, 0, 1, 1, new Vector3(0, s45, -s45), 0.82f);
            // the sides: from the outer square (code 1) down to the bottom (code 2)
            Quad(new Vector3(0.5f, 0, -0.5f), new Vector3(0.5f, 0, 0.5f), new Vector3(0.5f, 0, 0.5f), new Vector3(0.5f, 0, -0.5f), 1, 1, 2, 2, Vector3.right, 0.9f);
            Quad(new Vector3(-0.5f, 0, 0.5f), new Vector3(-0.5f, 0, -0.5f), new Vector3(-0.5f, 0, -0.5f), new Vector3(-0.5f, 0, 0.5f), 1, 1, 2, 2, Vector3.left, 0.9f);
            Quad(new Vector3(0.5f, 0, 0.5f), new Vector3(-0.5f, 0, 0.5f), new Vector3(-0.5f, 0, 0.5f), new Vector3(0.5f, 0, 0.5f), 1, 1, 2, 2, Vector3.forward, 0.9f);
            Quad(new Vector3(-0.5f, 0, -0.5f), new Vector3(0.5f, 0, -0.5f), new Vector3(0.5f, 0, -0.5f), new Vector3(-0.5f, 0, -0.5f), 1, 1, 2, 2, Vector3.back, 0.9f);

            mesh.Clear();
            mesh.SetVertices(verts);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetTriangles(tris, 0);
            mesh.bounds = new Bounds(Vector3.zero, new Vector3(1f, 2f, 1f));
        }
    }
}
