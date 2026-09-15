using System;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu.Presentation
{
    /// <summary>Draws a <see cref="Heightfield"/> with Unity's terrain renderer and imports terrains and heightmaps into heightfields.
    /// The view either owns a runtime Terrain object (one flat-coloured layer, so it renders in URP without any asset) or shows the
    /// field on a scene-authored Terrain, whose TerrainData is cloned so that the edits (a castle's plateau) never touch the asset.</summary>
    public sealed class TerrainView : IDisposable
    {
        /// <summary>Colour of the runtime terrain's single layer (the demos' ground tint).</summary>
        public Color GroundColor = new Color32(78, 112, 58, 255);
        /// <summary>Metres per tile of the flat-colour texture (any value: the texture is one colour).</summary>
        public float TileSize = 8f;
        public bool CastShadows = true;

        GameObject m_Root;
        Terrain m_Terrain;
        TerrainData m_Data;
        TerrainLayer m_Layer;
        Texture2D m_Texture;
        Terrain m_SceneTerrain;
        TerrainData m_SceneOriginal, m_SceneClone;

        /// <summary>The Terrain currently showing the field (the view's own or the scene's), or null.</summary>
        public Terrain Terrain => m_SceneTerrain != null ? m_SceneTerrain : m_Terrain;
        public bool Visible => Terrain != null && Terrain.gameObject.activeSelf;

        /// <summary>Shows the field: on <paramref name="sceneTerrain"/> when given (its data cloned on first use, restored by
        /// <see cref="Hide"/>), otherwise on a Terrain object of the view's own, created on first use.</summary>
        public void Show(Heightfield field, Terrain sceneTerrain = null)
        {
            if (field == null) { Hide(); return; }
            if (sceneTerrain != null)
            {
                if (m_Terrain != null) m_Terrain.gameObject.SetActive(false);
                if (m_SceneTerrain != sceneTerrain) { RestoreSceneTerrain(); m_SceneTerrain = sceneTerrain; m_SceneOriginal = sceneTerrain.terrainData; m_SceneClone = UnityEngine.Object.Instantiate(m_SceneOriginal); m_SceneClone.name = m_SceneOriginal.name + " (view)"; }
                Apply(m_SceneClone, field);
                Assign(sceneTerrain, m_SceneClone);
                Place(sceneTerrain, field);
                sceneTerrain.gameObject.SetActive(true);
                return;
            }
            RestoreSceneTerrain();
            if (m_Terrain == null)
            {
                m_Texture = new Texture2D(2, 2, TextureFormat.RGBA32, false) { name = "AvbdTerrainGround", wrapMode = TextureWrapMode.Repeat };
                m_Texture.SetPixels(new[] { GroundColor, GroundColor, GroundColor, GroundColor });
                m_Texture.Apply();
                m_Layer = new TerrainLayer { name = "AvbdTerrainGround", diffuseTexture = m_Texture, tileSize = new Vector2(TileSize, TileSize) };
                m_Data = new TerrainData { name = "AvbdTerrain" };
                m_Root = Terrain.CreateTerrainGameObject(m_Data);
                m_Root.name = "AvbdTerrain";
                var collider = m_Root.GetComponent<TerrainCollider>();
                if (collider != null) UnityEngine.Object.Destroy(collider);   // the solver collides, not PhysX
                m_Terrain = m_Root.GetComponent<Terrain>();
                m_Terrain.drawInstanced = true;
                m_Terrain.heightmapPixelError = 3f;
            }
            m_Terrain.shadowCastingMode = CastShadows ? ShadowCastingMode.On : ShadowCastingMode.Off;
            Apply(m_Data, field);
            m_Data.terrainLayers = new[] { m_Layer };
            Assign(m_Terrain, m_Data);
            Place(m_Terrain, field);
            m_Root.SetActive(true);
        }

        /// <summary>Hides the view's terrain and gives a scene terrain its own data back.</summary>
        public void Hide()
        {
            if (m_Root != null) m_Root.SetActive(false);
            RestoreSceneTerrain();
        }

        void RestoreSceneTerrain()
        {
            if (m_SceneTerrain == null) return;
            if (m_SceneOriginal != null) Assign(m_SceneTerrain, m_SceneOriginal);
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
            if (m_Root != null) UnityEngine.Object.Destroy(m_Root);
            if (m_Data != null) UnityEngine.Object.Destroy(m_Data);
            if (m_Layer != null) UnityEngine.Object.Destroy(m_Layer);
            if (m_Texture != null) UnityEngine.Object.Destroy(m_Texture);
            m_Root = null; m_Terrain = null; m_Data = null; m_Layer = null; m_Texture = null;
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
        public static Heightfield FromTerrain(Terrain terrain, int maxResolution = 2049)
        {
            var td = terrain.terrainData;
            int res = td.heightmapResolution;
            float[,] n = td.GetHeights(0, 0, res, res);
            int stride = 1;
            while ((res - 1) / stride + 1 > maxResolution && (res - 1) % (stride * 2) == 0) stride *= 2;
            int outRes = (res - 1) / stride + 1;
            var cell = new float2(td.size.x / (res - 1) * stride, td.size.z / (res - 1) * stride);
            Vector3 pos = terrain.transform.position;
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
