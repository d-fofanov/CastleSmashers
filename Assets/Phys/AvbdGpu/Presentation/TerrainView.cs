using System;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu.Presentation
{
    /// <summary>How a heightfield is drawn: the smooth surface with Unity's terrain renderer, or as if built from flat tiles
    /// (<see cref="TerrainTiles"/>: tops rounded to a height step, grooves between the pieces, risers at level changes).</summary>
    public enum TerrainStyle { Smooth = 0, Tiled = 1 }

    /// <summary>Draws a <see cref="Heightfield"/> and imports terrains and heightmaps into heightfields. Smooth style: the view either
    /// owns a runtime Terrain object (two flat-coloured layers, grass and dry hilltop, blended by height, so it renders in URP
    /// without any asset) or shows the field on a scene-authored Terrain, whose TerrainData is cloned so that the edits (a
    /// castle's plateau) never touch the asset. Tiled style: the field drawn as instanced tiles by <see cref="TerrainTiles"/>
    /// (<see cref="Render"/> once per frame); a scene terrain is hidden meanwhile. The collision surface is the same either way.</summary>
    public sealed class TerrainView : IDisposable
    {
        public TerrainStyle Style = TerrainStyle.Smooth;
        /// <summary>The tiled style's tile size, height step, detail radius, chamfer and colours.</summary>
        public TerrainTiles.Settings TileSettings = TerrainTiles.Settings.Default;
        /// <summary>Colour of the runtime terrain's low ground (the demos' ground tint, a shade lighter for the PBR terrain shader).</summary>
        public Color GroundColor = new Color32(108, 148, 78, 255);
        /// <summary>Colour of the runtime terrain's high ground, blended in from <see cref="HighFrom"/> to <see cref="HighTo"/> of the
        /// field's height range (so the hills read even under flat lighting).</summary>
        public Color HighColor = new Color32(156, 150, 92, 255);
        public float HighFrom = 0.45f, HighTo = 0.95f;
        /// <summary>Metres per tile of the flat-colour textures (any value: the textures are one colour).</summary>
        public float TileSize = 8f;
        public bool CastShadows = true;

        GameObject m_Root;
        Terrain m_Terrain;
        TerrainData m_Data;
        TerrainLayer m_Layer, m_HighLayer;
        Texture2D m_Texture, m_HighTexture;
        Terrain m_SceneTerrain;
        TerrainData m_SceneOriginal, m_SceneClone;
        Vector3 m_ScenePosition;
        TerrainTiles m_Tiles;
        bool m_TilesVisible;
        Terrain m_HiddenSceneTerrain;   // a scene terrain switched off while the tiles stand in for it

        /// <summary>The Terrain currently showing the field (the view's own or the scene's), or null (also in the tiled style).</summary>
        public Terrain Terrain => m_SceneTerrain != null ? m_SceneTerrain : m_Terrain;
        /// <summary>The tiles of the tiled style (null until it was shown).</summary>
        public TerrainTiles Tiles => m_Tiles;

        /// <summary>The data a scene terrain had before the view showed a field on it (its own data while it shows none).</summary>
        public TerrainData OriginalData(Terrain sceneTerrain) => sceneTerrain == m_SceneTerrain && m_SceneOriginal != null ? m_SceneOriginal : sceneTerrain.terrainData;
        public bool Visible => m_TilesVisible || (Terrain != null && Terrain.gameObject.activeSelf);

        /// <summary>Shows the field: as tiles in the tiled style (hiding <paramref name="sceneTerrain"/> if given), otherwise on
        /// <paramref name="sceneTerrain"/> when given (its data cloned on first use, restored by <see cref="Hide"/>), or on a Terrain
        /// object of the view's own, created on first use.</summary>
        public void Show(Heightfield field, Terrain sceneTerrain = null)
        {
            if (field == null) { Hide(); return; }
            if (Style == TerrainStyle.Tiled)
            {
                if (m_Root != null) m_Root.SetActive(false);
                RestoreSceneTerrain();
                if (sceneTerrain != null) { m_HiddenSceneTerrain = sceneTerrain; sceneTerrain.gameObject.SetActive(false); }
                m_Tiles ??= new TerrainTiles();
                m_Tiles.Build(field, TileSettings);
                m_TilesVisible = true;
                return;
            }
            m_TilesVisible = false;
            UnhideSceneTerrain();
            if (sceneTerrain != null)
            {
                if (m_Terrain != null) m_Terrain.gameObject.SetActive(false);
                if (m_SceneTerrain != sceneTerrain)
                {
                    RestoreSceneTerrain();
                    m_SceneTerrain = sceneTerrain; m_SceneOriginal = sceneTerrain.terrainData; m_ScenePosition = sceneTerrain.transform.position;
                    m_SceneClone = UnityEngine.Object.Instantiate(m_SceneOriginal);
                    m_SceneClone.name = m_SceneOriginal.name + " (view)";
                }
                Apply(m_SceneClone, field);
                Assign(sceneTerrain, m_SceneClone);
                Place(sceneTerrain, field);
                sceneTerrain.gameObject.SetActive(true);
                return;
            }
            RestoreSceneTerrain();
            if (m_Terrain == null)
            {
                m_Texture = FlatTexture("AvbdTerrainGround", GroundColor);
                m_HighTexture = FlatTexture("AvbdTerrainHigh", HighColor);
                m_Layer = new TerrainLayer { name = "AvbdTerrainGround", diffuseTexture = m_Texture, tileSize = new Vector2(TileSize, TileSize) };
                m_HighLayer = new TerrainLayer { name = "AvbdTerrainHigh", diffuseTexture = m_HighTexture, tileSize = new Vector2(TileSize, TileSize) };
                m_Data = new TerrainData { name = "AvbdTerrain" };
                m_Root = Terrain.CreateTerrainGameObject(m_Data);
                m_Root.name = "AvbdTerrain";
                var collider = m_Root.GetComponent<TerrainCollider>();
                if (collider != null) UnityEngine.Object.Destroy(collider);   // the solver collides, not PhysX
                m_Terrain = m_Root.GetComponent<Terrain>();
                m_Terrain.drawInstanced = true;
                m_Terrain.heightmapPixelError = 3f;
                // the URP terrain material shipped in Resources: a player strips the pipeline default when no scene has a terrain
                // (the terrain then draws flat and grey, its instanced patches undisplaced)
                var material = Resources.Load<Material>("AvbdGpu/AvbdTerrainLit");
                if (material != null) m_Terrain.materialTemplate = material;
            }
            m_Terrain.shadowCastingMode = CastShadows ? ShadowCastingMode.On : ShadowCastingMode.Off;
            Apply(m_Data, field);
            m_Data.terrainLayers = new[] { m_Layer, m_HighLayer };
            // layers assigned from script get no weight (the control map stays zero and the terrain draws black): paint the ground in
            // full and blend the high colour in with the height
            int alphaRes = math.clamp(math.max(field.ResX, field.ResZ), 16, 256);
            m_Data.alphamapResolution = alphaRes;
            var weights = new float[alphaRes, alphaRes, 2];
            float range = math.max(field.MaxHeight - field.MinHeight, 1e-3f);
            for (int z = 0; z < alphaRes; z++)
                for (int x = 0; x < alphaRes; x++)
                {
                    float2 xz = field.Origin + field.Extent * new float2((x + 0.5f) / alphaRes, (z + 0.5f) / alphaRes);
                    float t = (field.Height(xz) - field.MinHeight) / range;
                    float high = math.smoothstep(HighFrom, HighTo, t);
                    weights[z, x, 0] = 1f - high;
                    weights[z, x, 1] = high;
                }
            m_Data.SetAlphamaps(0, 0, weights);
            Assign(m_Terrain, m_Data);
            Place(m_Terrain, field);
            m_Root.SetActive(true);
        }

        /// <summary>Hides the view's terrain and tiles and gives a scene terrain its own data back.</summary>
        public void Hide()
        {
            if (m_Root != null) m_Root.SetActive(false);
            m_TilesVisible = false;
            RestoreSceneTerrain();
            UnhideSceneTerrain();
        }

        /// <summary>Draws the tiles of the tiled style (once per frame; the smooth style's terrain draws itself).</summary>
        public void Render(Camera camera = null)
        {
            if (m_TilesVisible && m_Tiles != null) m_Tiles.Render(camera, CastShadows);
        }

        void UnhideSceneTerrain()
        {
            if (m_HiddenSceneTerrain != null) m_HiddenSceneTerrain.gameObject.SetActive(true);
            m_HiddenSceneTerrain = null;
        }

        void RestoreSceneTerrain()
        {
            if (m_SceneTerrain == null) return;
            if (m_SceneOriginal != null) { Assign(m_SceneTerrain, m_SceneOriginal); m_SceneTerrain.transform.position = m_ScenePosition; }
            if (m_SceneClone != null) UnityEngine.Object.Destroy(m_SceneClone);
            m_SceneTerrain = null; m_SceneOriginal = null; m_SceneClone = null;
        }

        static void Assign(Terrain terrain, TerrainData data)
        {
            terrain.terrainData = data;
            var collider = terrain.GetComponent<TerrainCollider>();
            if (collider != null) collider.terrainData = data;
            terrain.Flush();
        }

        static void Place(Terrain terrain, Heightfield field)
        {
            terrain.transform.position = new Vector3(field.Origin.x, field.MinHeight, field.Origin.y);
        }

        public void Dispose()
        {
            RestoreSceneTerrain();
            UnhideSceneTerrain();
            m_Tiles?.Dispose(); m_Tiles = null; m_TilesVisible = false;
            if (m_Root != null) UnityEngine.Object.Destroy(m_Root);
            if (m_Data != null) UnityEngine.Object.Destroy(m_Data);
            if (m_Layer != null) UnityEngine.Object.Destroy(m_Layer);
            if (m_HighLayer != null) UnityEngine.Object.Destroy(m_HighLayer);
            if (m_Texture != null) UnityEngine.Object.Destroy(m_Texture);
            if (m_HighTexture != null) UnityEngine.Object.Destroy(m_HighTexture);
            m_Root = null; m_Terrain = null; m_Data = null; m_Layer = null; m_HighLayer = null; m_Texture = null; m_HighTexture = null;
        }

        static Texture2D FlatTexture(string name, Color color)
        {
            var tex = new Texture2D(2, 2, TextureFormat.RGBA32, false) { name = name, wrapMode = TextureWrapMode.Repeat };
            tex.SetPixels(new[] { color, color, color, color });
            tex.Apply();
            return tex;
        }

        // ------------------------------------------------------------------------------------------------ conversions

        /// <summary>Writes the field into a TerrainData: a square power-of-two-plus-one resolution (the field is resampled onto it
        /// when it has another; bilinear resampling of a bilinear surface is exact when refining), the extent and height range as
        /// the size, the samples normalised to the range. The terrain must then stand at (Origin.x, MinHeight, Origin.y).</summary>
        public static void Apply(TerrainData data, Heightfield field)
        {
            int res = ValidResolution(math.max(field.ResX, field.ResZ));
            if (field.ResX != res || field.ResZ != res) field = field.Resampled(res, res);
            float range = math.max(field.MaxHeight - field.MinHeight, 1f);
            data.heightmapResolution = res;
            data.size = new Vector3(field.Extent.x, range, field.Extent.y);
            data.SetHeights(0, 0, field.ToNormalized(range));
        }

        /// <summary>The smallest terrain heightmap resolution (2^n + 1, 33 .. 4097) holding <paramref name="samples"/> per axis.</summary>
        public static int ValidResolution(int samples)
        {
            int res = 33;
            while (res < samples && res < 4097) res = (res - 1) * 2 + 1;
            return res;
        }

        /// <summary>The heightfield of a Unity terrain (world-unit heights, the terrain's position included), taking every
        /// second sample repeatedly while the resolution exceeds <paramref name="maxResolution"/>.</summary>
        public static Heightfield FromTerrain(Terrain terrain, int maxResolution = 2049) => FromTerrainData(terrain.terrainData, terrain.transform.position, maxResolution);

        /// <summary>The heightfield of terrain data placed at <paramref name="position"/>.</summary>
        public static Heightfield FromTerrainData(TerrainData td, Vector3 position, int maxResolution = 2049)
        {
            int res = td.heightmapResolution;
            float[,] n = td.GetHeights(0, 0, res, res);
            int stride = 1;
            while ((res - 1) / stride + 1 > maxResolution && (res - 1) % (stride * 2) == 0) stride *= 2;
            int outRes = (res - 1) / stride + 1;
            var cell = new float2(td.size.x / (res - 1) * stride, td.size.z / (res - 1) * stride);
            Vector3 pos = position;
            var h = new float[outRes * outRes];
            for (int z = 0; z < outRes; z++)
                for (int x = 0; x < outRes; x++)
                    h[z * outRes + x] = n[z * stride, x * stride] * td.size.y + pos.y;
            return new Heightfield(outRes, outRes, cell, new float2(pos.x, pos.z), h);
        }

        /// <summary>A heightfield from a readable heightmap texture (R16 and RFloat read exactly, other formats through the red channel;
        /// row 0 of the texture is z = 0), <paramref name="heightScale"/> metres per unit of the normalised value.</summary>
        public static Heightfield FromTexture(Texture2D heightmap, float2 cell, float2 origin, float heightScale, float baseHeight = 0f)
        {
            int w = heightmap.width, hgt = heightmap.height;
            var n = new float[hgt, w];
            if (heightmap.format == TextureFormat.R16)
            {
                var data = heightmap.GetPixelData<ushort>(0);
                for (int z = 0; z < hgt; z++) for (int x = 0; x < w; x++) n[z, x] = data[z * w + x] / 65535f;
            }
            else if (heightmap.format == TextureFormat.RFloat)
            {
                var data = heightmap.GetPixelData<float>(0);
                for (int z = 0; z < hgt; z++) for (int x = 0; x < w; x++) n[z, x] = data[z * w + x];
            }
            else
            {
                var px = heightmap.GetPixels();
                for (int z = 0; z < hgt; z++) for (int x = 0; x < w; x++) n[z, x] = px[z * w + x].r;
            }
            return Heightfield.FromNormalized(n, cell, origin, heightScale, baseHeight);
        }
    }
}
