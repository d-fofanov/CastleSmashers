using System.Collections.Generic;
using Phys.AvbdGpu;
using Phys.AvbdGpu.Presentation;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Phys.Demo
{
    /// <summary>Castles described as brick-assembly documents (Assets/Phys/BRICK_ASSEMBLY.md) on the GPU solver (Preview.unity): every
    /// piece is a box body, sloped and round pieces included, drawn with its model from the construction-piece pack. The documents
    /// are the JSON files of Resources/Castles and, after them, the trees of Resources/Trees (keys 1-0 and , . choose between
    /// them); snap, cannonball, camera and player flags as in the castle demo. No siege: the armies need the procedural plan's
    /// wall geometry.</summary>
    public class PreviewDemo : DemoBase
    {
        [Tooltip("The 27 piece models in catalog order (PieceCatalog.Pieces; Phys / Assign Preview Meshes fills them in). Unset pieces are drawn as boxes.")]
        public Mesh[] PieceMeshes;
        [Tooltip("Resources folders holding the brick-assembly JSON documents: the castles, then the trees the castle demo plants.")]
        public string CastleFolder = "Castles";
        public string TreeFolder = "Trees";
        [Tooltip("Solver metres per model metre. 1 simulates the true 0.1 m stud pitch; the default 5 puts the pieces in the metre / kilogram " +
                 "regime the solver's penalty ramp is tuned for (and slows the motion accordingly).")]
        public float BrickScale = 5f;
        [Tooltip("Mass of a 2 x 3 brick (kg); every piece gets the same density.")]
        public float BrickMass = 0.25f;
        [Tooltip("Mass of a 2 x 3 brick of a tree document (kg), a quarter of the castle brick, as in the castle demo: a crown stands on the few contacts of its trunk top.")]
        public float TreeMass = 0.25f;
        public float BrickFriction = 0.6f;
        [Tooltip("Clearance of the collision boxes from the pieces' footprints, per side, in model metres (real bricks: about 1.25 % of the pitch). " +
                 "Neighbouring boxes that touch pass loads that snapped bricks are not built to take; the models are drawn at full size.")]
        public float Clearance = AssemblySpec.DefaultClearance;
        [Tooltip("Snap the pieces together with hard ball-socket joints that break under load (also the -avbd-snap flag).")]
        public bool Snap;
        [Tooltip("Sideways force (N) that breaks a snap connection.")]
        public float SnapFractureLateral = 300f;
        [Tooltip("Upward pull (N) that breaks a snap connection; a snap also comes apart once the pieces separate by half the stud height.")]
        public float SnapFractureTension = 50f;
        [Tooltip("Snap the pieces on the ground to the world as well (a castle on its base plate); a tree stands on the ground by friction either way.")]
        public bool WorldSnaps = true;
        public bool Shadows = true;
        [Tooltip("Cast the shadows of the pieces from their collision boxes instead of the models (-avbd-meshshadows turns it off), as in the castle demo.")]
        public bool BoxShadows = true;
        [Tooltip("Cannonball: cube size (model metres), mass (kg) and speed (model metres per second).")]
        public float ShotCube = 0.12f;
        public float ShotMass = 30f;
        public float ShotVelocity = 24f;

        TextAsset[] m_Documents = new TextAsset[0];
        int m_TreeCount;
        string m_StartCastle;
        BrickAssembly m_Assembly;
        AssemblyBodies m_Bodies;
        AssemblySpec m_Spec;
        AssemblyBuilder.Diagnostics m_Diagnostics;
        string m_Error;
        int m_SnapJoints;
        float m_Plateau;
        uint[] m_Tints;
        static readonly Color32 s_Iron = new Color32(70, 72, 78, 255);

        void Reset()
        {
            MaxBodies = 40960;
        }

        /// <summary>The documents of the Resources folder, by file name.</summary>
        public TextAsset[] Documents => m_Documents;
        /// <summary>The loaded assembly (null when the current document was rejected or the folder is empty).</summary>
        public BrickAssembly Assembly => m_Assembly;
        public AssemblyBodies Bodies => m_Bodies;
        public AssemblySpec Spec => m_Spec;
        /// <summary>Floating, poorly supported and intersecting pieces of the current document (reported, never corrected).</summary>
        public AssemblyBuilder.Diagnostics Diagnostics => m_Diagnostics;
        /// <summary>Why the current document could not be loaded, or null.</summary>
        public string Error => m_Error;
        public int PartCount => m_Assembly?.Parts.Count ?? 0;
        public int SnapJoints => m_SnapJoints;
        /// <summary>Ground height under the castle (the plateau levelled into the terrain; 0 on the flat ground).</summary>
        public float Plateau => m_Plateau;

        protected override int SceneCount => math.max(1, m_Documents.Length);
        protected override string SceneName(int index) => index < m_Documents.Length ? m_Documents[index].name : "(no documents)";
        protected override float HudHeight => 283f;
        protected override float3 ShotSize => ShotCube * BrickScale;
        protected override float ShotDensity => ShotMass / math.pow(ShotCube * BrickScale, 3f);
        /// <summary>Speeds scale with the square root of lengths under the same gravity (dynamic similarity).</summary>
        protected override float ShotSpeed => ShotVelocity * math.sqrt(BrickScale);
        protected override float ShotDistance => 0.5f * BrickScale;
        /// <summary>Same frequency as the reference's 5000 N/m on a 1 kg box.</summary>
        protected override float DragStiffness => 5000f * math.max(m_Spec.BrickMass, 0.01f);
        /// <summary>Tiled terrain steps of one plate (a third of a brick) at the brick scale.</summary>
        protected override float DefaultTileStep => Brick.BodyHeight / 3f * BrickScale;
        /// <summary>The contact penalties ramp up from their minimum over the first steps and the stacks sink a few centimetres
        /// meanwhile; settle that before showing the castle (single substeps: nothing moves fast yet).</summary>
        protected override int SettleSteps => 180;
        protected override int SettleSubsteps => 1;

        protected override AvbdGpuConfig CreateConfig()
        {
            var cfg = AvbdGpuConfig.ForBodies(MaxBodies);
            cfg.MaxJoints = MaxBodies * 12;                   // four snap joints per overlap (about ten per piece) plus drag joints
            cfg.MaxLinks = cfg.MaxJoints * 2 + 4096;
            return cfg;
        }

        /// <summary>Whether the document is one of the tree folder (they are listed after the castles).</summary>
        public bool IsTree(int index) => index >= m_Documents.Length - m_TreeCount;

        /// <summary>Index of the document with the file name (without extension), or -1.</summary>
        public int IndexOf(string castle)
        {
            for (int i = 0; i < m_Documents.Length; i++) if (m_Documents[i].name == castle) return i;
            return -1;
        }

        protected override void ParseArgs(string[] args)
        {
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == "-avbd-snap") Snap = true;
                if (args[i] == "-avbd-castle" && i + 1 < args.Length) m_StartCastle = args[i + 1];
                if (args[i] == "-avbd-noshadows") Shadows = false;
                if (args[i] == "-avbd-meshshadows") BoxShadows = false;
            }
        }

        protected override void Configure()
        {
            m_Renderer.Shadows = Shadows;
            m_Renderer.BoxShadows = BoxShadows;
            m_Renderer.DrawJoints = false;
            LoadDocuments();
            if (m_StartCastle != null)
            {
                int index = IndexOf(m_StartCastle);
                if (index >= 0) StartScene = index;
                else Debug.LogWarning($"PreviewDemo: no document '{m_StartCastle}' in Resources/{CastleFolder} or Resources/{TreeFolder}");
            }
        }

        /// <summary>Reads the folders again (new files show up after R): the castles by file name, then the trees by file name.</summary>
        public void LoadDocuments()
        {
            var docs = new List<TextAsset>();
            m_TreeCount = 0;
            foreach (var folder in new[] { CastleFolder, TreeFolder })
            {
                if (string.IsNullOrEmpty(folder)) continue;
                var found = new List<TextAsset>(Resources.LoadAll<TextAsset>(folder));
                found.Sort((a, b) => string.CompareOrdinal(a.name, b.name));
                docs.AddRange(found);
                if (folder == TreeFolder) m_TreeCount = found.Count;
            }
            m_Documents = docs.ToArray();
            if (m_Documents.Length == 0) Debug.LogWarning($"PreviewDemo: no documents in Resources/{CastleFolder} or Resources/{TreeFolder}");
        }

        protected override void BuildScene(int index, out float3 cameraTarget, out float cameraDistance)
        {
            m_Assembly = null; m_Bodies = null; m_Error = null; m_SnapJoints = 0; m_Diagnostics = default; m_Plateau = 0f;
            cameraTarget = new float3(0f, 2f * BrickScale, 0f);
            cameraDistance = 20f * BrickScale;
            if (m_Tints == null || m_Tints.Length < 1024) m_Tints = new uint[1024];
            if (index < m_Documents.Length)
            {
                try { m_Assembly = BrickAssembly.Parse(m_Documents[index].text); }
                catch (BrickAssemblyException e)
                {
                    m_Error = e.Message;
                    Debug.LogError($"PreviewDemo: {m_Documents[index].name}.json rejected: {e.Message}");
                }
            }
            else m_Error = $"no documents in Resources/{CastleFolder} or Resources/{TreeFolder}";

            // the terrain (or the flat ground), with a plateau under the castle's footprint and two studs around it
            float2 half = m_Assembly != null ? (m_Assembly.Extent.xz * 0.5f) * (PieceCatalog.GridToUnity * BrickScale) : new float2(4f * BrickScale);
            m_Plateau = AddGround(CreateTerrain(), BrickFriction, half, 2f * PieceCatalog.GridToUnity * BrickScale, out int groundBody);
            m_Tints[groundBody] = AvbdGpuRenderer.Tint(new Color32(78, 112, 58, 255));
            m_Renderer.SetTints(m_Tints, groundBody, 1);
            if (m_Assembly == null) return;
            m_Diagnostics = AssemblyBuilder.Diagnose(m_Assembly);
            if (!m_Diagnostics.Clean)
                Debug.LogWarning($"PreviewDemo: {m_Documents[index].name}.json: {m_Diagnostics.Floating} floating, {m_Diagnostics.PoorlySupported} poorly supported, {m_Diagnostics.Intersections} intersecting pieces ({m_Diagnostics.Sample})");
            float unit = PieceCatalog.GridToUnity * BrickScale;
            float3 centre = (m_Assembly.Min + m_Assembly.Max) * 0.5f;
            m_Spec = new AssemblySpec
            {
                Scale = BrickScale, Density = (IsTree(index) ? TreeMass : BrickMass) / (2f * 3f * 1.2f * unit * unit * unit), Friction = BrickFriction, Margin = AvbdGpuConstants.CollisionMargin,
                Clearance = Clearance, Origin = new float3(-centre.x * unit, m_Plateau, -centre.z * unit),   // the footprint centred on the world origin, on the plateau
            };
            m_Bodies = AssemblyBuilder.Build(m_World, m_Assembly, m_Spec);
            if (Snap) m_SnapJoints = AssemblyBuilder.AddSnapJoints(m_World, m_Assembly, m_Bodies, m_Spec, SnapFractureLateral, SnapFractureTension, WorldSnaps && !IsTree(index));

            // colours: one tint per body from its part's colour
            int n = m_Bodies.Count;
            if (m_Tints.Length < m_Bodies.First + n) System.Array.Resize(ref m_Tints, m_Bodies.First + n);
            for (int i = 0; i < n; i++)
            {
                uint rgb = m_Assembly.Parts[m_Bodies.PartOfBody[i]].Rgb;
                m_Tints[m_Bodies.First + i] = AvbdGpuRenderer.Tint(new Color32((byte)(rgb >> 16), (byte)(rgb >> 8), (byte)rgb, 255));
            }
            m_Renderer.SetTints(m_Tints, 0, m_Bodies.First + n);
            foreach (var g in m_Bodies.Groups)
            {
                var mesh = PieceMesh(ref PieceMeshes, g.Piece);
                if (mesh != null) m_Renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = mesh, Scale = BrickScale, Offset = g.MeshOffset, Start = g.Start, Count = g.Count });
            }

            // fewer substeps for the big castles: their cannonball crosses a fraction of the wall thickness per step even at one
            m_World.Params.Substeps = n > 20000 ? 1 : n > 8000 ? 2 : 3;
            float3 extent = m_Assembly.Extent * unit;
            cameraTarget = new float3(0f, m_Plateau + 0.4f * extent.y, 0f);
            cameraDistance = 1.4f * math.max(math.max(extent.x, extent.z), extent.y);
        }

        protected override void OnShot(int body)
        {
            if (m_Tints.Length <= body) System.Array.Resize(ref m_Tints, math.max(body + 1, 1024));
            m_Tints[body] = AvbdGpuRenderer.Tint(s_Iron);
            m_Renderer.SetTints(m_Tints, body, 1);
        }

        protected override void HandleSceneKeys()
        {
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb.jKey.wasPressedThisFrame) { Snap = !Snap; Load(m_Scene); }
            if (kb.f6Key.wasPressedThisFrame) m_Renderer.DrawCollisionBoxes = !m_Renderer.DrawCollisionBoxes;
            if (kb.f7Key.wasPressedThisFrame) { Shadows = !Shadows; m_Renderer.Shadows = Shadows; }
            if (kb.tKey.wasPressedThisFrame) CycleTerrain();
#endif
        }

        protected override string HudText()
        {
            string title = m_Documents.Length > 0 ? $"<b>[{m_Scene + 1}/{m_Documents.Length}] {m_Assembly?.Name ?? m_Documents[m_Scene].name}</b> ({m_Documents[m_Scene].name}.json)" : "<b>no documents</b>";
            string castle;
            if (m_Assembly == null)
                castle = $"<color=#ff5555>{m_Error}</color>\n\n";
            else
            {
                float unit = PieceCatalog.GridToUnity * BrickScale;
                var counts = m_Assembly.PieceCounts();
                var pieces = new System.Text.StringBuilder();
                for (int i = 0; i < counts.Count && i < 6; i++) pieces.Append(i > 0 ? ", " : "").Append(counts[i].count).Append(' ').Append(PieceCatalog.Pieces[counts[i].piece].Id);
                if (counts.Count > 6) pieces.Append(", ...");
                float3 e = m_Assembly.Extent;
                castle =
                    $"{PartCount} pieces of {counts.Count} kinds: {pieces}\n" +
                    $"{e.x:F0} x {e.z:F0} studs, {e.y:F1} tall = {e.x * unit:F1} x {e.z * unit:F1} x {e.y * unit:F1} m at model x {BrickScale:G3}; 2 x 3 brick {m_Spec.BrickMass:F2} kg\n" +
                    (Snap ? $"<color=#88ddff>snapped</color>: {m_SnapJoints} joints; a snap breaks at {SnapFractureLateral:F0} N sideways, {SnapFractureTension:F0} N upward or {m_Spec.SnapBreakDistance * 100f:F1} cm apart  -  cannonball {ShotMass:F0} kg\n"
                          : $"dry-stacked (friction only)  -  cannonball {ShotMass:F0} kg\n") +
                    (m_Diagnostics.Clean ? "every piece rests on at least half its footprint, nothing intersects\n"
                          : $"<color=#ffcc55>{m_Diagnostics.Floating} floating, {m_Diagnostics.PoorlySupported} on less than half their footprint, {m_Diagnostics.Intersections} intersecting</color> ({m_Diagnostics.Sample})\n");
            }
            return
                $"{title}{PausedText}\n" + castle + TerrainText(m_Plateau) + "\n\n" +
                StatsText() + "\n\n" +
                $"1-0 document (Resources/{CastleFolder}, then {TreeFolder})  , . prev/next  R rebuild  J snap pieces on/off  T terrain  Space pause  N step\n" +
                "F1 contacts  F2 colour mode  F5 joints  F6 collision boxes  F7 shadows  F8 sleep on/off  +/- iterations  [ ] substeps  B/Enter cannonball  G gravity  H hide HUD\n" +
                "LMB drag  RMB orbit  MMB pan  wheel / Q E zoom  W A S D orbit";
        }
    }
}
