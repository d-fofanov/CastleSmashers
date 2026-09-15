using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace Phys.Demo.Editor
{
    /// <summary>Batch build of the demo players: Unity.exe -batchmode -quit -executeMethod Phys.Demo.Editor.BuildDemo.Build
    /// [-buildScene Demo|Castle|Preview] [-buildPath C:/out/Demo.exe]. Menu: Phys / Build Demo Player, Build Castle Player,
    /// Build Preview Player.</summary>
    public static class BuildDemo
    {
        [MenuItem("Phys/Build Demo Player")]
        public static void Build() => BuildPlayer("Demo");

        [MenuItem("Phys/Build Castle Player")]
        public static void BuildCastle() => BuildPlayer("Castle");

        [MenuItem("Phys/Build Preview Player")]
        public static void BuildPreview() => BuildPlayer("Preview");

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
            demo.SiegeParams = demo.SiegeParams.WithDefaults();
            EditorUtility.SetDirty(demo);
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(scene);
            UnityEditor.SceneManagement.EditorSceneManager.SaveScene(scene);
            Debug.Log($"AssignCastleMeshes: brick {demo.BrickMesh}, figure {demo.FigureMesh}, arrow {demo.ArrowMesh}");
        }

        /// <summary>Assigns the 27 piece meshes of the construction pack, in catalog order, to the PreviewDemo of Preview.unity and saves
        /// the scene (the player needs the references serialised; the editor falls back to loading them by path).</summary>
        [MenuItem("Phys/Assign Preview Meshes")]
        public static void AssignPreviewMeshes()
        {
            var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene("Assets/Phys/Demo/Preview.unity");
            var demo = UnityEngine.Object.FindFirstObjectByType<PreviewDemo>();
            if (demo == null) { Debug.LogError("Preview.unity has no PreviewDemo"); if (Application.isBatchMode) EditorApplication.Exit(1); return; }
            var pieces = Phys.AvbdGpu.Scenes.PieceCatalog.Pieces;
            demo.PieceMeshes = new Mesh[pieces.Length];
            int missing = 0;
            for (int i = 0; i < pieces.Length; i++)
            {
                demo.PieceMeshes[i] = AssetDatabase.LoadAssetAtPath<Mesh>($"Assets/Models/construction_pieces/{pieces[i].Id}.fbx");
                if (demo.PieceMeshes[i] == null) { missing++; Debug.LogError($"AssignPreviewMeshes: no mesh for {pieces[i].Id}"); }
            }
            EditorUtility.SetDirty(demo);
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(scene);
            UnityEditor.SceneManagement.EditorSceneManager.SaveScene(scene);
            Debug.Log($"AssignPreviewMeshes: {pieces.Length - missing} of {pieces.Length} piece meshes assigned");
            if (missing > 0 && Application.isBatchMode) EditorApplication.Exit(1);
        }

        /// <summary>Writes <see cref="TerrainSettings.CastleScene"/> into Castle.unity (hills on) and Preview.unity (the same field, flat
        /// ground on) and saves both scenes, so that the players open on the configured terrain.</summary>
        [MenuItem("Phys/Set Demo Terrains")]
        public static void SetDemoTerrains()
        {
            var castle = UnityEditor.SceneManagement.EditorSceneManager.OpenScene("Assets/Phys/Demo/Castle.unity");
            var castleDemo = UnityEngine.Object.FindFirstObjectByType<CastleDemo>();
            if (castleDemo == null) { Debug.LogError("Castle.unity has no CastleDemo"); if (Application.isBatchMode) EditorApplication.Exit(1); return; }
            castleDemo.TerrainParams = TerrainSettings.CastleScene;
            EditorUtility.SetDirty(castleDemo);
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(castle);
            UnityEditor.SceneManagement.EditorSceneManager.SaveScene(castle);

            var preview = UnityEditor.SceneManagement.EditorSceneManager.OpenScene("Assets/Phys/Demo/Preview.unity");
            var previewDemo = UnityEngine.Object.FindFirstObjectByType<PreviewDemo>();
            if (previewDemo == null) { Debug.LogError("Preview.unity has no PreviewDemo"); if (Application.isBatchMode) EditorApplication.Exit(1); return; }
            var flat = TerrainSettings.CastleScene;
            flat.Preset = Phys.AvbdGpu.Scenes.TerrainPreset.None;
            previewDemo.TerrainParams = flat;
            EditorUtility.SetDirty(previewDemo);
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(preview);
            UnityEditor.SceneManagement.EditorSceneManager.SaveScene(preview);
            Debug.Log($"SetDemoTerrains: Castle.unity {castleDemo.TerrainParams.Preset} seed {castleDemo.TerrainParams.Seed}, Preview.unity {previewDemo.TerrainParams.Preset}");
        }

        /// <summary>Writes the castle planner's Outpost (the smallest preset, 1 001 bricks) as a brick-assembly document into
        /// Resources/Castles, where the preview demo picks it up: a castle known to stand, dry-stacked and snapped.</summary>
        [MenuItem("Phys/Export Outpost as Brick Assembly")]
        public static void ExportOutpostAssembly()
        {
            var plan = Phys.AvbdGpu.Scenes.CastlePlan.Presets[0];
            var layout = Phys.AvbdGpu.Scenes.BrickCastle.Generate(plan);
            string path = "Assets/Resources/Castles/outpost.json";
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            File.WriteAllText(path, Phys.AvbdGpu.Scenes.BrickAssembly.WriteLayout(layout, plan.Name));
            AssetDatabase.ImportAsset(path);
            Debug.Log($"ExportOutpostAssembly: {layout.Bricks.Count} bricks -> {path}");
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
