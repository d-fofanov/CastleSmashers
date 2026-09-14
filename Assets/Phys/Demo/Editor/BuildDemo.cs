using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace Phys.Demo.Editor
{
    /// <summary>Batch build of the demo players: Unity.exe -batchmode -quit -executeMethod Phys.Demo.Editor.BuildDemo.Build
    /// [-buildScene Demo|Castle] [-buildPath C:/out/Demo.exe]. Menu: Phys / Build Demo Player, Phys / Build Castle Player.</summary>
    public static class BuildDemo
    {
        [MenuItem("Phys/Build Demo Player")]
        public static void Build() => BuildPlayer("Demo");

        [MenuItem("Phys/Build Castle Player")]
        public static void BuildCastle() => BuildPlayer("Castle");

        static void BuildPlayer(string scene)
        {
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++) if (args[i] == "-buildScene") scene = args[i + 1];
            string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Build", scene + ".exe");
            for (int i = 0; i < args.Length - 1; i++) if (args[i] == "-buildPath") path = args[i + 1];
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            var options = new BuildPlayerOptions
            {
                scenes = new[] { $"Assets/Phys/Demo/{scene}.unity" },
                locationPathName = path,
                target = BuildTarget.StandaloneWindows64,
                options = BuildOptions.None,
            };
            BuildReport report = BuildPipeline.BuildPlayer(options);
            var s = report.summary;
            Debug.Log($"BuildDemo: {scene} {s.result}, {s.totalSize / (1024 * 1024)} MB, {s.totalErrors} errors, {s.totalWarnings} warnings -> {path}");
            if (s.result != BuildResult.Succeeded && Application.isBatchMode) EditorApplication.Exit(1);
        }

        /// <summary>Logs the guid / local id of the brick mesh (the reference serialised in Castle.unity).</summary>
        [MenuItem("Phys/Log Brick Mesh Reference")]
        public static void LogBrickMeshReference()
        {
            var mesh = AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx");
            if (mesh == null) { Debug.LogError("brick mesh not found"); return; }
            AssetDatabase.TryGetGUIDAndLocalFileIdentifier(mesh, out string guid, out long localId);
            Debug.Log($"BrickMesh: name '{mesh.name}' vertices {mesh.vertexCount} triangles {mesh.triangles.Length / 3} bounds {mesh.bounds} guid {guid} fileID {localId}");
        }
    }
}
