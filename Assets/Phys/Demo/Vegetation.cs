using System;
using System.Collections.Generic;
using Phys.AvbdGpu;
using Phys.AvbdGpu.Presentation;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.Demo
{
    /// <summary>Trees and foliage of the construction-piece pack (the brick-assembly documents of Resources/Trees and Resources/Foliage)
    /// scattered on the ground around a castle and built asleep, each an island of its own: shared by the castle and the siege demos.
    /// <see cref="LoadDocuments"/> once, then per scene <see cref="Scatter"/> (places them and levels the ground under them; the caller
    /// uploads the terrain when it was cut) and <see cref="Build"/> (bodies, snaps, tints, mesh ranges).</summary>
    public sealed class Vegetation
    {
        [Serializable]
        public struct Settings
        {
            /// <summary>Resources folders holding the tree and the foliage documents.</summary>
            public string TreeFolder, FoliageFolder;
            /// <summary>Trees scattered (0: none), bodies reserved for them, clumps of foliage per tree, bodies reserved for those.</summary>
            public int Trees, TreeBodies, FoliagePerTree, FoliageBodies;
            /// <summary>Seed of the trees' kinds, places and turns (the foliage draws its own from it).</summary>
            public uint Seed;
            /// <summary>Mass of a 2 x 3 brick of a tree / of the foliage (kg).</summary>
            public float TreeMass, FoliageMass;
            /// <summary>Solver metres per model metre, the pieces' friction, and the snaps (joints breaking at these limits).</summary>
            public float Scale, Friction;
            public bool Snap;
            public float SnapFractureLateral, SnapFractureTension;
        }

        /// <summary>The ground the trees and the foliage keep clear of: the castle's plateau core (half the width of a square about the
        /// origin), rectangular bands on the ground (xz min / max: where armies stand and march), the outlying copies' cores, and how far
        /// out the trees and the foliage go.</summary>
        public struct Exclusion
        {
            public float CoreHalf, TreeOuter, FoliageOuter, OutlyingCore;
            public float2[] OutlyingCentres;
            public List<float4> Bands;

            /// <summary>A footprint of half width <paramref name="r"/> at <paramref name="c"/> lies off the core, off every band and clear
            /// of the outlying copies.</summary>
            public bool Clear(float2 c, float r)
            {
                if (math.cmax(math.abs(c)) < CoreHalf + r) return false;
                if (Bands != null)
                    foreach (var b in Bands)
                        if (c.x > b.x - r && c.x < b.z + r && c.y > b.y - r && c.y < b.w + r) return false;
                if (OutlyingCentres != null)
                    foreach (var oc in OutlyingCentres) if (math.cmax(math.abs(c - oc)) < OutlyingCore + r) return false;
                return true;
            }
        }

        /// <summary>A tree standing around the castle: which document, where (world xz and the ground height under it), turned by how
        /// many quarter turns, and its bodies (contiguous, one island).</summary>
        public struct TreePlacement
        {
            public int Kind;
            public float2 Centre;
            public float Ground;
            public int Turns;
            public int First, Count;
            /// <summary>How far the crown reaches from the trunk (m).</summary>
            public float Radius;
        }

        /// <summary>A clump of foliage standing around the castle: which document, where (world xz and the ground height under it),
        /// turned by how many quarter turns, and its pieces: the parts [PartStart, PartStart + Count) of its cluster's assembly, which
        /// <see cref="FoliageBody"/> maps to bodies (the clumps of a cluster are built interleaved by piece, so that a cluster is drawn
        /// in a handful of ranges rather than one per clump).</summary>
        public struct FoliagePlacement
        {
            public int Kind;
            public float2 Centre;
            public float Ground;
            public int Turns;
            /// <summary>Half the width of the clump's footprint (m).</summary>
            public float Radius;
            /// <summary>The clump stands on a terrace of its own (the ground under it was not level).</summary>
            public bool Terraced;
            public int Cluster, PartStart, Count;
        }

        /// <summary>The clumps of one grid cell, built as one assembly (its parts in the clumps' order) with the bodies laid out by piece.</summary>
        sealed class FoliageCluster
        {
            public BrickAssembly Assembly;
            public AssemblyBodies Bodies;
        }

        /// <summary>Half the width of the square levelled exactly under a tree's trunk (m), and how far a terrace blends into the terrain.</summary>
        const float TrunkSquare = 3f, TerraceBlend = 2f;
        /// <summary>Cells per axis of the grid the foliage is clustered by (over the square it is scattered on).</summary>
        public const int FoliageGrid = 4;

        public Settings Params;
        readonly string m_Owner;
        TextAsset[] m_TreeDocuments = new TextAsset[0];
        BrickAssembly[] m_TreeKinds = new BrickAssembly[0];
        BrickAssembly[,] m_TreeTurned = new BrickAssembly[0, 4];
        readonly List<TreePlacement> m_TreePlacements = new List<TreePlacement>();
        int m_FirstTree, m_TreePieces;
        TextAsset[] m_FoliageDocuments = new TextAsset[0];
        BrickAssembly[] m_FoliageKinds = new BrickAssembly[0];
        readonly List<FoliagePlacement> m_FoliagePlacements = new List<FoliagePlacement>();
        readonly List<FoliageCluster> m_FoliageClusters = new List<FoliageCluster>();
        int m_FirstFoliage, m_FoliagePieces;

        public Vegetation(Settings settings, string owner = "Vegetation")
        {
            Params = settings;
            m_Owner = owner;
        }

        /// <summary>The tree documents of the tree folder (by file name) and their parsed assemblies (null where a document was
        /// rejected); the trees standing around the castle, their pieces contiguous.</summary>
        public TextAsset[] TreeDocuments => m_TreeDocuments;
        public BrickAssembly[] TreeKinds => m_TreeKinds;
        public IReadOnlyList<TreePlacement> TreePlacements => m_TreePlacements;
        public int TreeCount => m_TreePlacements.Count;
        public int TreePieces => m_TreePieces;
        public int FirstTreePiece => m_FirstTree;
        /// <summary>The foliage documents of the foliage folder (by file name) and their parsed assemblies (null where a document was
        /// rejected); the clumps standing around the castle, their pieces contiguous after the trees', cluster by cluster.</summary>
        public TextAsset[] FoliageDocuments => m_FoliageDocuments;
        public BrickAssembly[] FoliageKinds => m_FoliageKinds;
        public IReadOnlyList<FoliagePlacement> FoliagePlacements => m_FoliagePlacements;
        public int FoliageCount => m_FoliagePlacements.Count;
        public int FoliagePieces => m_FoliagePieces;
        public int FirstFoliagePiece => m_FirstFoliage;
        public int FoliageClusterCount => m_FoliageClusters.Count;
        /// <summary>The bodies of a cluster (their parts in the clumps' order, see <see cref="AssemblyBodies.BodyOfPart"/>).</summary>
        public AssemblyBodies FoliageClusterBodies(int cluster) => m_FoliageClusters[cluster].Bodies;
        /// <summary>The body of the i-th piece of a clump.</summary>
        public int FoliageBody(in FoliagePlacement clump, int i) => m_FoliageClusters[clump.Cluster].Bodies.BodyOfPart[clump.PartStart + i];
        /// <summary>Every piece of every tree and clump (the bodies reserved for them are not counted).</summary>
        public int Pieces => m_TreePieces + m_FoliagePieces;

        /// <summary>Bodies to reserve on top of the awake budget for what the settings ask for.</summary>
        public static int ReservedBodies(in Settings s) => (s.Trees > 0 ? s.TreeBodies : 0) + (s.Trees > 0 && s.FoliagePerTree > 0 ? s.FoliageBodies : 0);

        /// <summary>Reads the tree and the foliage folders (the documents by file name) and parses every document once; a rejected
        /// document is logged and skipped.</summary>
        public void LoadDocuments()
        {
            LoadFolder(Params.TreeFolder, "tree", Params.Trees > 0, out m_TreeDocuments, out m_TreeKinds);
            LoadFolder(Params.FoliageFolder, "foliage", Params.Trees > 0 && Params.FoliagePerTree > 0, out m_FoliageDocuments, out m_FoliageKinds);
            m_TreeTurned = new BrickAssembly[m_TreeKinds.Length, 4];
        }

        void LoadFolder(string folder, string what, bool wanted, out TextAsset[] documents, out BrickAssembly[] kinds)
        {
            var docs = new List<TextAsset>(string.IsNullOrEmpty(folder) ? new TextAsset[0] : Resources.LoadAll<TextAsset>(folder));
            docs.Sort((a, b) => string.CompareOrdinal(a.name, b.name));
            documents = docs.ToArray();
            kinds = new BrickAssembly[documents.Length];
            for (int i = 0; i < documents.Length; i++)
            {
                try { kinds[i] = BrickAssembly.Parse(documents[i].text); }
                catch (BrickAssemblyException e) { Debug.LogError($"{m_Owner}: {documents[i].name}.json rejected: {e.Message}"); }
            }
            if (documents.Length == 0 && wanted) Debug.LogWarning($"{m_Owner}: no {what} documents in Resources/{folder}");
        }

        /// <summary>Half the width of every kind's bounding square (m; 0 for a rejected document).</summary>
        static float[] Radii(BrickAssembly[] kinds, float unit)
        {
            var radii = new float[kinds.Length];
            for (int k = 0; k < kinds.Length; k++)
                if (kinds[k] != null) radii[k] = math.max(math.cmax(math.abs(kinds[k].Min.xz)), math.cmax(math.abs(kinds[k].Max.xz))) * unit;
            return radii;
        }

        /// <summary>Scatters the trees, then the foliage, on the ground around the castle and levels the ground under them; returns
        /// whether the terrain was cut (the caller uploads it then).</summary>
        public bool Scatter(int scene, Heightfield field, in Exclusion x)
        {
            bool cut = ScatterTrees(scene, field, x);
            cut |= ScatterFoliage(scene, field, x);
            return cut;
        }

        /// <summary>Scatters up to <see cref="Settings.Trees"/> trees around the castle: kinds, places and quarter turns from the seed, each
        /// standing outside the exclusion (the castle's plateau core, the armies' bands, the outlying copies), clear of the other trees,
        /// as many as fit <see cref="Settings.TreeBodies"/>; then levels the ground under them: a tree stands on the ground of its trunk,
        /// on a terrace levelled into the terrain under its crown. Returns whether the terrain was cut.</summary>
        bool ScatterTrees(int scene, Heightfield field, in Exclusion s)
        {
            m_TreePlacements.Clear();
            int kinds = 0;
            foreach (var k in m_TreeKinds) if (k != null && k.Parts.Count > 0) kinds++;
            if (Params.Trees <= 0 || Params.TreeBodies <= 0 || kinds == 0) return false;
            float unit = PieceCatalog.GridToUnity * Params.Scale;
            uint seed = math.max(Params.Seed, 1u) * 0x9E3779B9u ^ (uint)(scene + 1) * 0x85EBCA6Bu;
            var rng = new Unity.Mathematics.Random(seed != 0u ? seed : 1u);
            var radii = Radii(m_TreeKinds, unit);
            int used = 0;
            for (int attempt = 0; attempt < Params.Trees * 400 && m_TreePlacements.Count < Params.Trees; attempt++)
            {
                int kind = rng.NextInt(m_TreeKinds.Length);
                if (m_TreeKinds[kind] == null || m_TreeKinds[kind].Parts.Count == 0) continue;
                float2 c = rng.NextFloat2(-s.TreeOuter, s.TreeOuter);
                float r = radii[kind] + 1f;
                if (!s.Clear(c, r)) continue;
                bool clear = true;
                foreach (var p in m_TreePlacements) if (math.distance(c, p.Centre) < p.Radius + r) { clear = false; break; }
                if (!clear) continue;
                var assembly = m_TreeKinds[kind];
                if (used + assembly.Parts.Count > Params.TreeBodies) continue;
                used += assembly.Parts.Count;
                m_TreePlacements.Add(new TreePlacement { Kind = kind, Centre = c, Turns = rng.NextInt(4), Radius = radii[kind] });
            }
            if (m_TreePlacements.Count == 0) return false;

            // the ground under every trunk, then a terrace under every crown blending out over 6 m, then the trunks' own squares
            // levelled again exactly (a neighbour's blend may have reached one); the level ground around the castle is left as it is
            bool cut = false;
            for (int i = 0; i < m_TreePlacements.Count; i++)
            {
                var p = m_TreePlacements[i];
                p.Ground = field != null ? field.MeanHeight(p.Centre - TrunkSquare, p.Centre + TrunkSquare) : 0f;
                m_TreePlacements[i] = p;
            }
            if (field != null)
            {
                foreach (var p in m_TreePlacements)
                {
                    float2 lo = p.Centre - p.Radius - 1f, hi = p.Centre + p.Radius + 1f;
                    if (math.abs(field.MeanHeight(lo, hi) - p.Ground) < 1e-3f && math.abs(field.MaxOver(lo, hi) - p.Ground) < 1e-3f) continue;
                    field.Flatten(lo, hi, p.Ground, 6f);
                    cut = true;
                }
                if (cut) foreach (var p in m_TreePlacements) field.Flatten(p.Centre - TrunkSquare, p.Centre + TrunkSquare, p.Ground, TerraceBlend);
            }
            return cut;
        }

        /// <summary>Scatters <see cref="Settings.FoliagePerTree"/> clumps of foliage per tree planted the way the trees are scattered (a
        /// seed of their own, out to the foot of the hills), each clear of the trees' crowns and of the other clumps by its bounding
        /// square, as many as fit <see cref="Settings.FoliageBodies"/>. A clump stands on the ground as it is where that is level within
        /// a cell of its footprint (the samples that shape the surface under it), and on a terrace of its own where it is not: its
        /// footprint and a cell around it levelled at the footprint's mean height, blending out over <see cref="TerraceBlend"/>. A
        /// terrace must reach neither another clump's level cell nor a tree's trunk square, so a clump with one stands two cells and a
        /// blend from every other clump. Returns whether the terrain was cut.</summary>
        bool ScatterFoliage(int scene, Heightfield field, in Exclusion s)
        {
            m_FoliagePlacements.Clear();
            int count = m_TreePlacements.Count * math.max(Params.FoliagePerTree, 0);
            int kinds = 0;
            foreach (var k in m_FoliageKinds) if (k != null && k.Parts.Count > 0) kinds++;
            if (count <= 0 || Params.FoliageBodies <= 0 || kinds == 0) return false;
            float unit = PieceCatalog.GridToUnity * Params.Scale;
            float cell = field != null ? math.cmax(field.Cell) : 0f;
            float apart = 2f * cell + TerraceBlend;                       // between the footprints of two clumps when either has a terrace
            uint seed = math.max(Params.Seed, 1u) * 0xC2B2AE35u ^ (uint)(scene + 1) * 0x27D4EB2Fu;
            var rng = new Unity.Mathematics.Random(seed != 0u ? seed : 1u);
            var radii = Radii(m_FoliageKinds, unit);
            int used = 0;
            for (int attempt = 0; attempt < count * 400 && m_FoliagePlacements.Count < count; attempt++)
            {
                int kind = rng.NextInt(m_FoliageKinds.Length);
                if (m_FoliageKinds[kind] == null || m_FoliageKinds[kind].Parts.Count == 0) continue;
                float2 c = rng.NextFloat2(-s.FoliageOuter, s.FoliageOuter);
                float r = radii[kind];
                if (!s.Clear(c, r + 1f)) continue;
                bool terraced = false;
                if (field != null) { field.SampleRange(c - r - cell, c + r + cell, out float lo, out float hi); terraced = hi - lo > 1e-3f; }
                bool clear = true;
                foreach (var t in m_TreePlacements)
                {
                    float d = math.cmax(math.abs(c - t.Centre));
                    if (d < t.Radius + r + 1f || (terraced && d < TrunkSquare + r + cell + TerraceBlend)) { clear = false; break; }
                }
                if (clear) foreach (var f in m_FoliagePlacements)
                    if (math.cmax(math.abs(c - f.Centre)) < f.Radius + r + (terraced || f.Terraced ? apart : 1f)) { clear = false; break; }
                if (!clear) continue;
                var assembly = m_FoliageKinds[kind];
                if (used + assembly.Parts.Count > Params.FoliageBodies) continue;
                used += assembly.Parts.Count;
                float ground = field != null ? field.MeanHeight(c - r, c + r) : 0f;
                m_FoliagePlacements.Add(new FoliagePlacement { Kind = kind, Centre = c, Turns = rng.NextInt(4), Radius = r, Terraced = terraced, Ground = ground });
            }

            // the terraces: none reaches the ground under another clump or a tree, so every clump's ground is final
            bool cut = false;
            foreach (var f in m_FoliagePlacements)
            {
                if (!f.Terraced) continue;
                field.Flatten(f.Centre - f.Radius - cell, f.Centre + f.Radius + cell, f.Ground, TerraceBlend);
                cut = true;
            }
            return cut;
        }

        /// <summary>Builds what <see cref="Scatter"/> placed: the trees (one island each), then the foliage in clusters; snapped like the
        /// castle, drawn with the piece models (<paramref name="pieceMesh"/> by catalog index), tinted from the documents' colours into
        /// <paramref name="tints"/> (grown as needed; the caller uploads it).</summary>
        public void Build(AvbdGpuWorld world, AvbdGpuRenderer renderer, ref uint[] tints, Func<int, Mesh> pieceMesh, in Exclusion x)
        {
            BuildTrees(world, renderer, ref tints, pieceMesh);
            BuildFoliage(world, renderer, ref tints, pieceMesh, x);
        }

        static void Tint(ref uint[] tints, int body, uint rgb)
        {
            if (tints == null || tints.Length <= body) Array.Resize(ref tints, math.max(body + 1, 1024));
            tints[body] = AvbdGpuRenderer.Tint(new Color32((byte)(rgb >> 16), (byte)(rgb >> 8), (byte)rgb, 255));
        }

        void BuildTrees(AvbdGpuWorld world, AvbdGpuRenderer renderer, ref uint[] tints, Func<int, Mesh> pieceMesh)
        {
            m_FirstTree = world.BodyCount;
            m_TreePieces = 0;
            float unit = PieceCatalog.GridToUnity * Params.Scale;
            for (int i = 0; i < m_TreePlacements.Count; i++)
            {
                var p = m_TreePlacements[i];
                var assembly = m_TreeTurned[p.Kind, p.Turns] ?? (m_TreeTurned[p.Kind, p.Turns] = m_TreeKinds[p.Kind].Turned(p.Turns));
                var spec = new AssemblySpec
                {
                    Scale = Params.Scale, Density = Params.TreeMass / (2f * 3f * 1.2f * unit * unit * unit), Friction = Params.Friction, Margin = AvbdGpuConstants.CollisionMargin,
                    Clearance = AssemblySpec.DefaultClearance, Origin = new float3(p.Centre.x, p.Ground, p.Centre.y),
                };
                var bodies = AssemblyBuilder.Build(world, assembly, spec);
                if (Params.Snap) AssemblyBuilder.AddSnapJoints(world, assembly, bodies, spec, Params.SnapFractureLateral, Params.SnapFractureTension, worldJoints: false);   // a tree stands on the ground by friction
                world.SleepRange(bodies.First, bodies.Count);
                p.First = bodies.First; p.Count = bodies.Count;
                m_TreePlacements[i] = p;
                m_TreePieces += bodies.Count;
                for (int b = 0; b < bodies.Count; b++) Tint(ref tints, bodies.First + b, assembly.Parts[bodies.PartOfBody[b]].Rgb);
                foreach (var g in bodies.Groups)
                {
                    var mesh = pieceMesh(g.Piece);
                    if (mesh != null) renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = mesh, Scale = Params.Scale, Offset = g.MeshOffset, Start = g.Start, Count = g.Count });
                }
            }
        }

        /// <summary>Builds the clumps in clusters, the cells of a <see cref="FoliageGrid"/>-square grid over the ground they are scattered
        /// on: the clumps of a cell go into one assembly whose bodies <see cref="AssemblyBuilder.Build"/> lays out by piece, so that a
        /// cluster is drawn in as many ranges as it has kinds of piece (a hundred and twenty clumps would otherwise be six hundred draws,
        /// each with its shadow proxy) and culled on its own bounds. Every clump sleeps as an island of its own, snapped like the trees.</summary>
        void BuildFoliage(AvbdGpuWorld world, AvbdGpuRenderer renderer, ref uint[] tints, Func<int, Mesh> pieceMesh, in Exclusion s)
        {
            m_FirstFoliage = world.BodyCount;
            m_FoliagePieces = 0;
            m_FoliageClusters.Clear();
            if (m_FoliagePlacements.Count == 0) return;
            float unit = PieceCatalog.GridToUnity * Params.Scale;
            var spec = new AssemblySpec
            {
                Scale = Params.Scale, Density = Params.FoliageMass / (2f * 3f * 1.2f * unit * unit * unit), Friction = Params.Friction, Margin = AvbdGpuConstants.CollisionMargin,
                Clearance = AssemblySpec.DefaultClearance, Origin = float3.zero,   // every clump's place goes into its parts' positions
            };
            float cellSize = 2f * s.FoliageOuter / FoliageGrid;
            var cells = new List<int>[FoliageGrid * FoliageGrid];
            for (int i = 0; i < m_FoliagePlacements.Count; i++)
            {
                int2 cell = math.clamp((int2)math.floor((m_FoliagePlacements[i].Centre + s.FoliageOuter) / cellSize), 0, FoliageGrid - 1);
                int k = cell.y * FoliageGrid + cell.x;
                (cells[k] ??= new List<int>()).Add(i);
            }
            foreach (var clumps in cells)
            {
                if (clumps == null) continue;
                var assembly = new BrickAssembly { Name = $"foliage cluster {m_FoliageClusters.Count}" };
                foreach (int i in clumps)
                {
                    var f = m_FoliagePlacements[i];
                    var turned = m_FoliageKinds[f.Kind].Turned(f.Turns);
                    float3 at = new float3(f.Centre.x, f.Ground, f.Centre.y) / unit;
                    f.Cluster = m_FoliageClusters.Count; f.PartStart = assembly.Parts.Count; f.Count = turned.Parts.Count;
                    m_FoliagePlacements[i] = f;
                    foreach (var p in turned.Parts)
                        assembly.Parts.Add(new AssemblyPart { Id = i + "/" + p.Id, Piece = p.Piece, Position = p.Position + at, Rotation = p.Rotation, Rgb = p.Rgb });
                }
                var bodies = AssemblyBuilder.Build(world, assembly, spec);
                if (Params.Snap) AssemblyBuilder.AddSnapJoints(world, assembly, bodies, spec, Params.SnapFractureLateral, Params.SnapFractureTension, worldJoints: false);   // no two clumps overlap, so no snap crosses between them
                foreach (int i in clumps)   // one island per clump, body by body: a clump's bodies lie between its neighbours'
                {
                    var f = m_FoliagePlacements[i];
                    int island = int.MaxValue;
                    for (int j = 0; j < f.Count; j++) island = math.min(island, bodies.BodyOfPart[f.PartStart + j]);
                    for (int j = 0; j < f.Count; j++) world.SleepRange(bodies.BodyOfPart[f.PartStart + j], 1, island);
                }
                for (int i = 0; i < assembly.Parts.Count; i++) Tint(ref tints, bodies.BodyOfPart[i], assembly.Parts[i].Rgb);
                foreach (var g in bodies.Groups)
                {
                    var mesh = pieceMesh(g.Piece);
                    if (mesh != null) renderer.MeshRanges.Add(new AvbdGpuRenderer.MeshRange { Mesh = mesh, Scale = Params.Scale, Offset = g.MeshOffset, Start = g.Start, Count = g.Count });
                }
                m_FoliageClusters.Add(new FoliageCluster { Assembly = assembly, Bodies = bodies });
                m_FoliagePieces += bodies.Count;
            }
        }

        /// <summary>The HUD line on the trees and the foliage asleep around the castle (empty when there are none).</summary>
        public string AroundText()
        {
            if (m_TreePlacements.Count == 0) return Params.Trees > 0 ? "no trees (none fit, or no documents)\n" : "no trees (F)\n";
            string trees = $"<color=#a8d878>{m_TreePlacements.Count} trees</color> of {m_TreeDocuments.Length} kinds";
            if (m_FoliagePlacements.Count == 0)
                return trees + $" asleep around the castle ({m_TreePieces} pieces of the construction pack, {Params.TreeMass:F2} kg per 2 x 3 brick)" +
                       (Params.FoliagePerTree > 0 && m_FoliageDocuments.Length == 0 ? " - no foliage documents" : "") + "\n";
            int terraced = 0;
            foreach (var f in m_FoliagePlacements) if (f.Terraced) terraced++;
            return trees + $" and <color=#c8e090>{m_FoliagePlacements.Count} clumps of foliage</color> of {m_FoliageDocuments.Length} kinds asleep around the castle " +
                   $"({m_TreePieces + m_FoliagePieces} pieces; {m_FoliageClusters.Count} clusters, {terraced} clumps on terraces)\n";
        }
    }
}
