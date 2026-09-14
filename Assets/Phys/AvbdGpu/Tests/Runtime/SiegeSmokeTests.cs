using System.Collections;
using NUnit.Framework;
using Phys.AvbdGpu;
using Phys.AvbdGpu.Siege;
using Phys.Demo;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.TestTools;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>PlayMode: the castle demo under siege (armies spawned at load, asynchronous readbacks driving the units, volleys,
    /// purges) runs for a while without overflowing or blowing up, and the pools get used and freed.</summary>
    public class SiegeSmokeTests
    {
        GameObject m_Root;
        CastleDemo m_Demo;

        [UnitySetUp]
        public IEnumerator SetUp()
        {
            if (!AvbdGpuKernels.Supported) Assert.Ignore("no compute");
            m_Root = new GameObject("SiegeSmoke");
            var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
            camGo.transform.SetParent(m_Root.transform);
            camGo.AddComponent<Camera>();
            camGo.AddComponent<DemoCamera>();
            var light = new GameObject("Light").AddComponent<Light>();
            light.type = LightType.Directional;
            light.transform.SetParent(m_Root.transform);
            light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            m_Demo = m_Root.AddComponent<CastleDemo>();
            m_Demo.StartScene = 0;
            m_Demo.MaxBodies = 40960;
            m_Demo.SiegeOnLoad = true;
            var settings = SiegeSettings.Default;
            settings.VolleyInterval = 90;      // 1.5 s
            settings.RetireDelay = 120;        // 2 s
            settings.MarchDistance = 6f;
            m_Demo.SiegeParams = settings;
#if UNITY_EDITOR
            m_Demo.BrickMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx");
            m_Demo.FigureMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorFigure/ConstructorFigure.fbx");
            m_Demo.ArrowMesh = UnityEditor.AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorArrow/ConstructorArrow.fbx");
#endif
            yield return null;
        }

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            if (m_Root != null) Object.Destroy(m_Root);
            yield return null;
        }

        [UnityTest]
        public IEnumerator ArmiesBesiegeTheOutpost()
        {
            Assert.IsTrue(m_Demo.SiegeActive, "armies spawned at load");
            var siege = m_Demo.Siege;
            Assert.AreEqual(3, m_Demo.Renderer.MeshRanges.Count, "bricks, figures and arrows are drawn with their models");
            int attackers = siege.Alive[SiegeSystem.Attackers], defenders = siege.Alive[SiegeSystem.Defenders];
            Assert.Greater(attackers, 50);
            Assert.Greater(defenders, 0);
            Assert.AreEqual(attackers + defenders, siege.Units.Alive);

            var log = new System.Text.StringBuilder();
            for (int f = 1; f <= 720; f++)   // 12 s: marching in, several volleys and purges
            {
                yield return null;
                if (f % 120 == 0)
                {
                    var stats = m_Demo.World.GetStatsSync();
                    Assert.AreEqual(0, stats.OverflowFlags, $"capacity overflow {stats.OverflowFlags} at frame {f}");
                    log.Append($"  frame {f}: {siege.Summary()}\n");
                }
            }
            Debug.Log("siege smoke:\n" + log);
            Assert.Greater(siege.Volleys, 3, "volleys were fired");
            Assert.Greater(siege.ShotsFired, 100, "arrows flew");
            Assert.Greater(siege.Retired, 0, "the dead and the spent were retired after their cooldown");
            Assert.Less(siege.ProjectileList.Count, siege.ShotsFired, "spent projectiles were retired");
            int marching = 0;
            foreach (var u in siege.UnitList) if (u.State == UnitState.Marching) marching++;
            Assert.AreEqual(0, marching, "every attacker reached its firing line");
            Assert.Greater(siege.Dead[SiegeSystem.Attackers] + siege.Dead[SiegeSystem.Defenders], 0, "the volleys hit somebody");
            m_Demo.World.GetPosesSync(out var pos, out _);
            for (int i = 0; i < m_Demo.World.BodyCount; i++) Assert.IsTrue(math.all(math.isfinite(pos[i])), $"body {i} position");

            // clearing the armies retires everything; the pools are free again after a step
            m_Demo.ToggleSiege();
            Assert.IsFalse(m_Demo.SiegeActive);
            yield return null;
            yield return null;
            Assert.AreEqual(0, siege.Units.Alive);
            Assert.AreEqual(siege.Units.Capacity, siege.Units.Free);
            Assert.AreEqual(siege.Arrows.Capacity, siege.Arrows.Free);
        }
    }
}
