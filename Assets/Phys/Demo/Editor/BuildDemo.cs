using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace Phys.Demo.Editor
{
    /// <summary>Batch build of the demo player: Unity.exe -batchmode -quit -executeMethod Phys.Demo.Editor.BuildDemo.Build
    /// [-buildPath C:/out/Demo.exe]. Menu: Phys / Build Demo Player.</summary>
    public static class BuildDemo
    {
        [MenuItem("Phys/Build Demo Player")]
        public static void Build()
        {
            string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Build", "Demo.exe");
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++) if (args[i] == "-buildPath") path = args[i + 1];
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            var options = new BuildPlayerOptions
            {
                scenes = new[] { "Assets/Phys/Demo/Demo.unity" },
                locationPathName = path,
                target = BuildTarget.StandaloneWindows64,
                options = BuildOptions.None,
            };
            BuildReport report = BuildPipeline.BuildPlayer(options);
            var s = report.summary;
            Debug.Log($"BuildDemo: {s.result}, {s.totalSize / (1024 * 1024)} MB, {s.totalErrors} errors, {s.totalWarnings} warnings -> {path}");
            if (s.result != BuildResult.Succeeded && Application.isBatchMode) EditorApplication.Exit(1);
        }
    }
}
