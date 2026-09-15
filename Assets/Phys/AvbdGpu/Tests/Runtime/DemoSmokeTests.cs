using System.Collections;
using NUnit.Framework;
using Phys.AvbdGpu;
using Phys.AvbdGpu.Scenes;
using Phys.Demo;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.TestTools;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>PlayMode: the demo bootstrap (GPU world + renderer + HUD) runs every catalog scene for a few frames without errors.</summary>
    public class DemoSmokeTests
    {
        GameObject m_Root;

        [UnitySetUp]
        public IEnumerator SetUp()
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("no compute");
            m_Root = new GameObject("DemoSmoke");
            var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
            camGo.transform.SetParent(m_Root.transform);
            camGo.AddComponent<Camera>();
            camGo.AddComponent<DemoCamera>();
            var light = new GameObject("Light").AddComponent<Light>();
            light.type = LightType.Directional;
            light.transform.SetParent(m_Root.transform);
            light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            var demo = m_Root.AddComponent<DemoBootstrap>();
            demo.StartScene = AvbdScenes.Ground;
            demo.MaxBodies = 8192;
            yield return null;
        }

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            if (m_Root != null) Object.Destroy(m_Root);
            yield return null;
        }

        [UnityTest]
        public IEnumerator EveryReferenceSceneRuns()
        {
            var demo = m_Root.GetComponent<DemoBootstrap>();
            for (int scene = 0; scene <= AvbdScenes.Breakable; scene++)
            {
                demo.Load(scene);
                for (int f = 0; f < 30; f++) yield return null;
                demo.World.GetPosesSync(out var pos, out var rot);
                for (int i = 0; i < demo.World.BodyCount; i++)
                {
                    Assert.IsTrue(math.all(math.isfinite(pos[i])), $"scene {scene} body {i} position");
                    Assert.IsTrue(math.all(math.isfinite(rot[i])), $"scene {scene} body {i} rotation");
                }
                var stats = demo.World.GetStatsSync();
                Assert.AreEqual(0, stats.OverflowFlags, $"scene {scene}: capacity overflow {stats.OverflowFlags}");
            }
        }

        /// <summary>The Terrain scene: the world gets a heightfield, the view draws it with a Unity terrain of the same extent and
        /// height range, the boxes land on it, and the terrain imports back as the same field (16-bit heightmap quantisation).</summary>
        [UnityTest]
        public IEnumerator TerrainSceneIsDrawnAndImportsBack()
        {
            var demo = m_Root.GetComponent<DemoBootstrap>();
            demo.Load(AvbdScenes.Terrain);
            yield return null;
            var field = demo.World.Terrain;
            Assert.IsNotNull(field, "the scene set a terrain");
            var terrain = demo.TerrainView.Terrain;
            Assert.IsNotNull(terrain, "a Terrain object draws it");
            Assert.IsTrue(terrain.gameObject.activeInHierarchy);
            Assert.AreEqual(field.Extent.x, terrain.terrainData.size.x, 1e-3f);
            Assert.AreEqual(field.MinHeight, terrain.transform.position.y, 1e-5f);
            var back = Phys.AvbdGpu.Presentation.TerrainView.FromTerrain(terrain);
            Assert.AreEqual(field.ResX, back.ResX);
            for (int z = 0; z < field.ResZ; z += 7)
                for (int x = 0; x < field.ResX; x += 5)
                    Assert.AreEqual(field[x, z], back[x, z], 1e-3f, $"sample ({x}, {z}) survives the round trip");
            for (int f = 0; f < 90; f++) yield return null;
            var stats = demo.World.GetStatsSync();
            Assert.Greater(stats.TerrainManifolds, 50, "the boxes have landed on the hills");
            Assert.AreEqual(0, stats.OverflowFlags);
            demo.World.GetPosesSync(out var pos, out _);
            for (int i = 1; i < demo.World.BodyCount; i++)
                Assert.Greater(pos[i].y, field.Height(pos[i].xz) - 0.5f, $"body {i} stays above the surface");
            demo.Load(AvbdScenes.Ground);
            yield return null;
            Assert.IsFalse(demo.TerrainView.Visible, "a scene without terrain hides it");
        }

        [UnityTest]
        public IEnumerator ShootingAndDraggingWork()
        {
            var demo = m_Root.GetComponent<DemoBootstrap>();
            demo.Load(AvbdScenes.Ground);
            yield return null;
            int before = demo.World.BodyCount;
            demo.World.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 6, 0), quaternion.identity, new float3(0, 0, 5));
            int joint = demo.World.AddJointIndexed(-1, 1, new float3(0, 6, 0), float3.zero, 5000f, 0f);
            for (int f = 0; f < 20; f++) { demo.World.SetJointAnchor(joint, new float3(0, 6 + f * 0.05f, 0)); yield return null; }
            demo.World.RemoveJoint(joint);
            for (int f = 0; f < 10; f++) yield return null;
            Assert.AreEqual(before + 1, demo.World.BodyCount);
            demo.World.GetPosesSync(out var pos, out _);
            Assert.Greater(pos[1].y, 2f, "the dragged box was lifted");
        }
    }
}
