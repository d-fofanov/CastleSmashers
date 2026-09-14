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

        /// <summary>Assigns the brick, figure and arrow meshes to the CastleDemo of Castle.unity and saves the scene (the player
        /// needs the references serialised; the editor falls back to loading them by path).</summary>
        [MenuItem("Phys/Assign Castle Meshes")]
        public static void AssignCastleMeshes()
        {
            var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene("Assets/Phys/Demo/Castle.unity");
            var demo = UnityEngine.Object.FindFirstObjectByType<CastleDemo>();
            if (demo == null) { Debug.LogError("Castle.unity has no CastleDemo"); return; }
            demo.BrickMesh = AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx");
            demo.FigureMesh = AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorFigure/ConstructorFigure.fbx");
            demo.ArrowMesh = AssetDatabase.LoadAssetAtPath<Mesh>("Assets/Models/ConstructorArrow/ConstructorArrow.fbx");
            if (demo.SiegeParams.VolleyInterval <= 0) demo.SiegeParams = Phys.AvbdGpu.Siege.SiegeSettings.Default;
            EditorUtility.SetDirty(demo);
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(scene);
            UnityEditor.SceneManagement.EditorSceneManager.SaveScene(scene);
            Debug.Log($"AssignCastleMeshes: brick {demo.BrickMesh}, figure {demo.FigureMesh}, arrow {demo.ArrowMesh}");
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
