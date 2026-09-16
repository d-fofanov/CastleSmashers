using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.Demo
{
    /// <summary>Selects, builds and drives the catalog scenes on the GPU solver (Demo.unity).</summary>
    public class DemoBootstrap : DemoBase
    {
        void Reset()
        {
            StartScene = AvbdScenes.Pyramid;
            MaxBodies = 81920;   // the largest catalog scene has 73 811 bodies
        }

        protected override int SceneCount => AvbdScenes.Count;
        protected override string SceneName(int index) => AvbdScenes.All[index].Name;

        protected override void BuildScene(int index, out float3 cameraTarget, out float cameraDistance)
        {
            AvbdScenes.Build(m_World, index);
            var info = AvbdScenes.All[index];
            cameraTarget = info.CameraTarget;
            cameraDistance = info.CameraDistance;
        }

        protected override string HudText()
        {
            var info = AvbdScenes.All[m_Scene];
            return
                $"<b>[{m_Scene + 1}] {info.Name}</b>{PausedText}\n" +
                $"{info.Description}\n\n" +
                StatsText() + "\n\n" +
                "1-0 scene  , . prev/next scene  R reset  Space pause  N step  F1 contacts  F2 colour mode  F3 post-stabilise  F4 rotated inertia  F5 joints  F8 sleep on/off\n" +
                "+/- iterations  [ ] substeps  B/Enter shoot box  G gravity  H hide HUD  LMB drag body  RMB orbit  MMB pan  wheel / Q E zoom  W A S D orbit";
        }
    }
}
