using System.IO;
using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Phys.AvbdRef;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The brick-assembly loader (Assets/Phys/BRICK_ASSEMBLY.md): the catalog matches the pack, the format's rotation is
    /// Unity's, documents parse and expand as specified, invalid ones are rejected, and the bodies and snap joints land where the
    /// pieces are.</summary>
    public class BrickAssemblyTests
    {
        const string PiecesDir = "Assets/Models/construction_pieces";

        const string Bridge = @"{
  ""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""Small bridge"",
  ""units"": { ""gridToUnity"": 0.1, ""axes"": ""unity"" },
  ""palette"": { ""supports"": ""#687783"", ""deck"": ""#C58B42"" },
  ""parts"": [
    { ""id"": ""left_support"",  ""piece"": ""Brick_2x2"", ""position"": [-3, 0, 0],  ""rotation"": [0, 0, 0],  ""color"": ""supports"" },
    { ""id"": ""right_support"", ""piece"": ""Brick_2x2"", ""position"": [3, 0, 0],   ""rotation"": [0, 0, 0],  ""color"": ""supports"" },
    { ""id"": ""bridge_deck"",   ""piece"": ""Brick_2x8"", ""position"": [0, 1.2, 0], ""rotation"": [0, 90, 0], ""color"": ""deck"" }
  ]
}";

        const string Pillars = @"{
  ""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""Pillars"",
  ""units"": { ""gridToUnity"": 0.1, ""axes"": ""unity"" },
  ""palette"": { ""deck"": ""#C58B42"" },
  ""parts"": [ { ""id"": ""bridge_deck"", ""piece"": ""Brick_2x8"", ""position"": [0, 2.4, 0], ""rotation"": [0, 90, 0], ""color"": ""deck"" } ],
  ""modules"": {
    ""pillar"": { ""parts"": [
      { ""id"": ""bottom"", ""piece"": ""Brick_2x2"", ""position"": [0, 0, 0] },
      { ""id"": ""top"",    ""piece"": ""Brick_2x2"", ""position"": [0, 1.2, 0] } ] },
    ""pair"": { ""parts"": [ { ""id"": ""flag"", ""piece"": ""Tile_1x1"", ""position"": [1, 0, 0] } ],
               ""instances"": [ { ""id"": ""p"", ""module"": ""pillar"", ""position"": [2, 0, 0], ""rotation"": [0, 90, 0] } ] }
  },
  ""instances"": [
    { ""id"": ""pillar_left"",  ""module"": ""pillar"", ""position"": [-3, 0, 0], ""rotation"": [0, 0, 0] },
    { ""id"": ""pillar_right"", ""module"": ""pillar"", ""position"": [3, 0, 0] },
    { ""id"": ""turned"", ""module"": ""pair"", ""position"": [0, 0, 10], ""rotation"": [0, 90, 0] }
  ]
}";

        [Test]
        public void CatalogMatchesThePack()
        {
            Assert.AreEqual(27, PieceCatalog.Pieces.Length);
            Assert.AreEqual(27, Directory.GetFiles(PiecesDir, "*.fbx").Length, "one FBX per catalog piece");
            var manifest = JsonReaderForTests.Parse(File.ReadAllText(Path.Combine(PiecesDir, "mesh_manifest.json")));
            foreach (var piece in PieceCatalog.Pieces)
            {
                Assert.IsTrue(File.Exists(Path.Combine(PiecesDir, piece.Id + ".fbx")), piece.Id + ".fbx");
                Assert.AreEqual(piece.Id.StartsWith("Brick_") || piece.Id.StartsWith("Plate_"), piece.Studs, piece.Id + " studs");
                Assert.AreEqual(!piece.Id.StartsWith("Plain_"), piece.Grips, piece.Id + " grips");
                // the manifest's source axes are width / length / height (with the studs); the chamfer takes under 2.5 mm off the slopes
                float[] dims = manifest[piece.Id];
                Assert.AreEqual(dims[0], piece.Width * PieceCatalog.GridToUnity, 1e-4f, piece.Id + " width");
                Assert.AreEqual(dims[1], piece.Length * PieceCatalog.GridToUnity, 2.5e-3f, piece.Id + " length");
                Assert.AreEqual(dims[2], (piece.Height + (piece.Studs ? PieceCatalog.StudHeight : 0f)) * PieceCatalog.GridToUnity, 2.5e-3f, piece.Id + " height");
            }
        }

        [Test]
        public void RotationIsUnitysEulerConvention()
        {
            AssertVector(new float3(1, 0, 0), math.mul(BrickAssembly.Rotation(new float3(0, 90, 0)), new float3(0, 0, 1)), "+90 about Y maps +Z to +X");
            AssertVector(new float3(0, 0, 1), math.mul(BrickAssembly.Rotation(new float3(90, 0, 0)), new float3(0, 1, 0)), "+90 about X maps +Y to +Z");
            var rng = new Unity.Mathematics.Random(7);
            for (int i = 0; i < 20; i++)
            {
                float3 e = rng.NextFloat3(-180f, 180f), v = rng.NextFloat3(-1f, 1f);
                float3 expected = Quaternion.Euler(e.x, e.y, e.z) * (Vector3)v;   // Z first, then X, then Y
                AssertVector(expected, math.mul(BrickAssembly.Rotation(e), v), $"Euler {e}");
            }
        }

        [Test]
        public void BridgeExampleParses()
        {
            var a = BrickAssembly.Parse(Bridge);
            Assert.AreEqual("Small bridge", a.Name);
            Assert.AreEqual(3, a.Parts.Count);
            int deck = a.IndexOf("bridge_deck");
            Assert.AreEqual("Brick_2x8", PieceCatalog.Pieces[a.Parts[deck].Piece].Id);
            AssertVector(new float3(0, 1.2f, 0), a.Parts[deck].Position, "deck pivot");
            AssertVector(new float3(1, 0, 0), math.mul(a.Parts[deck].Rotation, new float3(0, 0, 1)), "the deck's length runs along X");
            Assert.AreEqual(0xC58B42u, a.Parts[deck].Rgb, "palette colour");
            Assert.AreEqual(0x687783u, a.Parts[a.IndexOf("left_support")].Rgb);
            AssertVector(new float3(-4, 0, -1), a.Min, "bounds min");
            AssertVector(new float3(4, 2.4f, 1), a.Max, "bounds max: the deck spans eight studs along X and rests on the supports");
        }

        [Test]
        public void ModulesExpandWithPathsAndComposedTransforms()
        {
            var a = BrickAssembly.Parse(Pillars);
            Assert.AreEqual(1 + 4 + 3, a.Parts.Count, "the deck, two pillars of two bricks, a pair of a tile and a pillar");
            int top = a.IndexOf("pillar_right/top");
            Assert.GreaterOrEqual(top, 0, "expanded ID path");
            AssertVector(new float3(3, 1.2f, 0), a.Parts[top].Position, "instance position plus local position");
            Assert.AreEqual(BrickAssembly.DefaultRgb, a.Parts[top].Rgb, "default colour");
            // 'turned' rotates the pair by +90 about Y at (0, 0, 10): local +X becomes world -Z, and its pillar is rotated too
            int flag = a.IndexOf("turned/flag"), nested = a.IndexOf("turned/p/bottom");
            AssertVector(new float3(0, 0, 9), a.Parts[flag].Position, "rotated instance");
            AssertVector(new float3(0, 0, 8), a.Parts[nested].Position, "nested instance");
            AssertVector(new float3(0, 0, -1), math.mul(a.Parts[nested].Rotation, new float3(0, 0, 1)), "rotations compose: 90 + 90 degrees about Y");
        }

        [TestCase("not json", "malformed JSON")]
        [TestCase("[1, 2]", "not a JSON object")]
        [TestCase(@"{""format"": ""lego"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": []}", "unsupported format")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 2, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": []}", "unsupported version")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""other"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": []}", "unsupported catalog")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 1, ""axes"": ""unity""}, ""parts"": []}", "gridToUnity")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""blender""}, ""parts"": []}", "axes")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}}", "missing 'parts'")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [{""id"": ""a"", ""piece"": ""Brick_1x1"", ""position"": [0, 0, 0]}, {""id"": ""a"", ""piece"": ""Brick_1x1"", ""position"": [1, 0, 0]}]}", "duplicate ID 'a'")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [{""id"": ""a"", ""piece"": ""Brick_9x9"", ""position"": [0, 0, 0]}]}", "unknown piece 'Brick_9x9'")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [{""id"": ""a"", ""piece"": ""Brick_1x1"", ""position"": [0, 0, 0], ""color"": ""stone""}]}", "not in the palette")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [{""id"": ""a"", ""piece"": ""Brick_1x1"", ""position"": [0, 0, 0], ""color"": ""#12345""}]}", "not #RRGGBB")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [{""id"": ""a"", ""piece"": ""Brick_1x1"", ""position"": [0, 0]}]}", "three numbers")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [{""id"": ""a"", ""piece"": ""Brick_1x1"", ""position"": [0, ""1"", 0]}]}", "three finite numbers")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [], ""instances"": [{""id"": ""i"", ""module"": ""ghost"", ""position"": [0, 0, 0]}]}", "unknown module 'ghost'")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [], ""modules"": {""a"": {""parts"": [], ""instances"": [{""id"": ""b"", ""module"": ""b"", ""position"": [0, 0, 0]}]}, ""b"": {""parts"": [], ""instances"": [{""id"": ""a"", ""module"": ""a"", ""position"": [0, 0, 0]}]}}}", "circular module reference")]
        [TestCase(@"{""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""x"", ""units"": {""gridToUnity"": 0.1, ""axes"": ""unity""}, ""parts"": [{""id"": ""i/p"", ""piece"": ""Brick_1x1"", ""position"": [0, 0, 0]}], ""modules"": {""m"": {""parts"": [{""id"": ""p"", ""piece"": ""Brick_1x1"", ""position"": [0, 0, 0]}]}}, ""instances"": [{""id"": ""i"", ""module"": ""m"", ""position"": [5, 0, 0]}]}", "not unique")]
        public void InvalidDocumentsAreRejected(string json, string message)
        {
            var e = Assert.Throws<BrickAssemblyException>(() => BrickAssembly.Parse(json));
            StringAssert.Contains(message, e.Message);
        }

        [Test]
        public void BoxesFollowThePiecesAndTheirDownAxis()
        {
            var spec = new AssemblySpec { Scale = 5f, Density = 100f, Friction = 0.6f, Margin = 0.01f, Clearance = 0.002f, Origin = new float3(1f, 0f, 2f) };
            float unit = spec.Unit, gap = 2f * spec.Clearance * spec.Scale;
            var upright = new AssemblyPart { Piece = PieceCatalog.IndexOf("Brick_2x8"), Position = new float3(0, 1.2f, 0), Rotation = BrickAssembly.Rotation(new float3(0, 90, 0)) };
            AssemblyBuilder.Box(upright, spec, out float3 size, out float3 position, out float3 offset);
            AssertVector(new float3(2f * unit - gap, 1.2f * unit + spec.Margin, 8f * unit - gap), size, "the box is the body plus one margin in height, less the clearance across");
            AssertVector(spec.Origin + new float3(0f, 1.2f * unit + (1.2f * unit - spec.Margin) * 0.5f, 0f), position, "the box hangs one margin below the pivot");
            AssertVector(new float3(0f, -(1.2f * unit - spec.Margin) * 0.5f, 0f), offset, "the mesh pivot sits one margin above the box bottom");

            // a cylinder lying along +Z (+90 about X): its local +Z axis points down, so that side gets the margin
            var lying = new AssemblyPart { Piece = PieceCatalog.IndexOf("Plain_Cylinder"), Position = new float3(0, 3, 0), Rotation = BrickAssembly.Rotation(new float3(90, 0, 0)) };
            AssertVector(new float3(0, 0, 1), AssemblyBuilder.DownAxis(lying.Rotation), "down axis");
            AssemblyBuilder.Box(lying, spec, out size, out position, out offset);
            AssertVector(new float3(1.5f * unit - gap, 1.5f * unit - gap, 1.5f * unit + spec.Margin), size, "the margin goes on the local axis that faces down, the clearance on the two others");
            AssertVector(spec.Origin + new float3(0f, 3f * unit - spec.Margin * 0.5f, 0.75f * unit), position, "the box centre: half a cylinder along +Z, half a margin down");
            AssertVector(new float3(0f, -0.75f * unit, -spec.Margin * 0.5f), offset, "mesh offset in body space");
        }

        [Test]
        public void BuildGroupsBodiesByMeshAndSnapsTheBridge()
        {
            var a = BrickAssembly.Parse(Bridge);
            var spec = new AssemblySpec { Scale = 5f, Density = 1.1f, Friction = 0.6f, Margin = 0.01f, Origin = float3.zero };   // about 1 kg per 2 x 3 brick, the demo's regime
            var world = new RefSceneBuilder(new Solver());
            world.AddBody(new float3(100, 1, 100), 0f, 0.6f, new float3(0, -0.5f, 0));
            var bodies = AssemblyBuilder.Build(world, a, spec);
            Assert.AreEqual(1, bodies.First);
            Assert.AreEqual(3, bodies.Count);
            Assert.AreEqual(2, bodies.Groups.Count, "one draw range per piece and orientation");
            Assert.AreEqual(3, bodies.Groups[0].Count + bodies.Groups[1].Count);
            for (int i = 0; i < 3; i++) Assert.AreEqual(i, bodies.PartOfBody[bodies.BodyOfPart[i] - bodies.First], "the maps between parts and bodies invert each other");
            int deck = bodies.BodyOfPart[a.IndexOf("bridge_deck")];
            AssertVector(new float3(8f * spec.Unit, 1.2f * spec.Unit + spec.Margin, 2f * spec.Unit), math.abs(math.mul(world.Rotation(deck), world.Bodies[deck].size)), "the deck's box, turned along X");
            Assert.AreEqual(1.2f * spec.Unit + (1.2f * spec.Unit - spec.Margin) * 0.5f, world.Position(deck).y, 1e-5f, "the deck's box centre");

            int joints = AssemblyBuilder.AddSnapJoints(world, a, bodies, spec, 300f, 50f);
            Assert.AreEqual(4 + 4 + 4 + 4, joints, "four world joints per support on the ground, four per support under the deck");
            int worldJoints = 0, deckJoints = 0;
            foreach (var j in world.Joints)
            {
                if (j.bodyA == null) worldJoints++;
                else if (j.bodyB == world.Bodies[deck])
                {
                    deckJoints++;
                    float3 anchorA = j.bodyA.positionLin + math.mul(world.Rotation(world.Bodies.IndexOf(j.bodyA)), j.rA);
                    float3 anchorB = j.bodyB.positionLin + math.mul(world.Rotation(deck), j.rB);
                    AssertVector(anchorA, anchorB, "both anchors meet at the interface plane");
                    Assert.AreEqual(1.2f * spec.Unit, anchorA.y, 1e-5f, "on top of the support");
                    Assert.AreEqual(2, j.snapAxis, "the snap axis is the deck's up");
                    Assert.AreEqual(300f / 4f, j.fractureLateral, 1e-6f); Assert.AreEqual(50f / 4f, j.fractureTension, 1e-6f);
                    Assert.AreEqual(spec.SnapBreakDistance, j.breakDistance, 1e-6f);
                }
            }
            Assert.AreEqual(8, worldJoints); Assert.AreEqual(8, deckJoints);

            // the snapped bridge stands on the reference solver
            world.Solver.iterations = 10;
            float3 start = world.Position(deck);
            for (int step = 0; step < 120; step++) world.Solver.Step();
            Assert.Less(math.distance(start, world.Position(deck)), 0.02f, "the deck stays put");
            foreach (var j in world.Joints) Assert.IsFalse(j.broken, "no snap broke under the bridge's own weight");
        }

        [Test]
        public void EmeraldCrownCitadelDesignExpands()
        {
            // the agent's design as delivered (Tools/castles): modules, nested instances, instances turned by 180 and -90 degrees
            var a = BrickAssembly.Parse(File.ReadAllText("Tools/castles/emerald_crown_citadel.design.json"));
            Assert.AreEqual("Emerald Crown Citadel", a.Name);
            Assert.AreEqual(1983, a.Parts.Count, "118 module instances, some nested, expand to 1 983 pieces");
            Assert.AreEqual(0f, a.Min.y, 1e-4f, "stands on the ground");
            Assert.Greater(a.Max.y, 30f); Assert.Greater(a.Extent.x, 69f); Assert.Greater(a.Extent.z, 76f);
            var counts = a.PieceCounts();
            Assert.AreEqual("Brick_2x8", PieceCatalog.Pieces[counts[0].piece].Id); Assert.AreEqual(560, counts[0].count);
            // instances turned by 180 and -90 degrees, and a tile turned on its side inside a turned instance (an independent expansion)
            var wall = a.Parts[a.IndexOf("wall_side_-24_-12/p0001")];
            AssertVector(new float3(-24, 2.8f, -12), wall.Position, "part of an instance turned 180 degrees");
            AssertVector(new float3(0, 0, -1), math.mul(wall.Rotation, new float3(0, 0, 1)), "its length axis points -Z");
            var rear = a.Parts[a.IndexOf("wall_rear_-16/p0010")];
            AssertVector(new float3(-13, 11.6f, 20.5f), rear.Position, "part of an instance turned -90 degrees");
            AssertVector(new float3(-1, 0, 0), math.mul(rear.Rotation, new float3(0, 0, 1)), "its length axis points -X");
            var tile = a.Parts[a.IndexOf("rear_spire_-24/tower_shell_000/p0076")];
            AssertVector(new float3(-24, 6.4f, 24.4f), tile.Position, "nested instance");
            AssertVector(new float3(0, -1, 0), math.mul(tile.Rotation, new float3(0, 0, 1)), "the tile stands on its edge");
            AssertVector(new float3(0, 0, -1), math.mul(tile.Rotation, new float3(0, 1, 0)), "its top faces -Z");
            var d = AssemblyBuilder.Diagnose(a);
            Assert.AreEqual(0, d.Intersections, "no two pieces intersect");
            Assert.Greater(d.Floating, 40, "the design has pieces resting on nothing: " + d.Sample);
        }

        [Test]
        public void EmeraldCrownCitadelIsBonded()
        {
            // the design re-bonded by Tools/rebond_castle.py: the same cells in the same colours, tiled for bond, with hidden supports
            var a = BrickAssembly.Parse(File.ReadAllText("Assets/Resources/Castles/emerald_crown_citadel.json"));
            var design = BrickAssembly.Parse(File.ReadAllText("Tools/castles/emerald_crown_citadel.design.json"));
            Assert.AreEqual(design.Name, a.Name);
            AssertVector(design.Min, a.Min, "the same bounds"); AssertVector(design.Max, a.Max, "the same bounds");
            Assert.Greater(a.Parts.Count, 2000); Assert.Less(a.Parts.Count, 2300);
            var d = AssemblyBuilder.Diagnose(a);
            Assert.AreEqual(0, d.Intersections);
            Assert.AreEqual(7, d.Floating, "only the three portcullis teeth and the trees' outer foliage bricks rest on nothing: " + d.Sample);
            Assert.LessOrEqual(d.PoorlySupported, 8, d.Sample);
            var spec = new AssemblySpec { Scale = 5f, Density = 1.1f, Friction = 0.6f, Margin = 0.01f, Clearance = AssemblySpec.DefaultClearance, Origin = float3.zero };
            var world = new RefSceneBuilder(new Solver());
            var bodies = AssemblyBuilder.Build(world, a, spec);
            int joints = AssemblyBuilder.AddSnapJoints(world, a, bodies, spec, 300f, 50f);
            Assert.Greater(joints, 8000);
            Debug.Log($"Emerald Crown Citadel (bonded): {a.Parts.Count} pieces in {bodies.Groups.Count} mesh groups, {joints} snap joints; " +
                $"{d.Floating} floating, {d.PoorlySupported} poorly supported ({d.Sample})");
        }

        [Test]
        public void OutpostRoundTripMatchesTheCastlePlanner()
        {
            var plan = CastlePlan.Presets[0];
            var layout = BrickCastle.Generate(plan);
            var a = BrickAssembly.Parse(BrickAssembly.WriteLayout(layout, plan.Name));
            Assert.AreEqual(plan.Name, a.Name);
            Assert.AreEqual(layout.Bricks.Count, a.Parts.Count);
            for (int i = 0; i < layout.Bricks.Count; i++)
            {
                var b = layout.Bricks[i];
                var p = a.Parts[i];
                Assert.AreEqual("Brick_2x3", PieceCatalog.Pieces[p.Piece].Id);
                AssertVector(new float3(b.X + b.W * 0.5f, b.Layer * 1.2f, b.Z + b.D * 0.5f), p.Position, $"brick {i} pivot");
                AssertVector(b.Rotated ? new float3(1, 0, 0) : new float3(0, 0, 1), math.mul(p.Rotation, new float3(0, 0, 1)), $"brick {i} orientation");
            }
            Assert.IsTrue(AssemblyBuilder.Diagnose(a).Clean, "every brick of the planned castle is supported and nothing intersects");

            // the same snap connections as the castle demo makes from the stud-grid layout
            var spec = new AssemblySpec { Scale = 5f, Density = 1.1f, Friction = 0.6f, Margin = 0.01f, Origin = float3.zero };
            var world = new RefSceneBuilder(new Solver());
            var bodies = AssemblyBuilder.Build(world, a, spec);
            int joints = AssemblyBuilder.AddSnapJoints(world, a, bodies, spec, 300f, 50f);
            var castle = new RefSceneBuilder(new Solver());
            int first = BrickCastle.Build(castle, layout, new BrickSpec { Scale = 5f, Density = 1.1f, Friction = 0.6f, Margin = 0.01f });
            int castleJoints = BrickCastle.AddSnapJoints(castle, layout, first, new BrickSpec { Scale = 5f, Density = 1.1f, Friction = 0.6f, Margin = 0.01f }, 300f, 50f);
            Assert.AreEqual(castleJoints, joints, "four joints per brick-on-brick overlap and per ground brick, like the castle demo");
            Debug.Log($"Outpost: {a.Parts.Count} bricks, {joints} snap joints both ways");
        }

        static void AssertVector(float3 expected, float3 actual, string what, float tolerance = 1e-4f)
        {
            Assert.Less(math.distance(expected, actual), tolerance, $"{what}: expected {expected}, got {actual}");
        }
    }

    /// <summary>Reads the manifest's list of {name, dimensions_xyz_m} entries (JsonUtility does not read top-level arrays).</summary>
    static class JsonReaderForTests
    {
        [System.Serializable] class Entry { public string name; public float[] dimensions_xyz_m; }
        [System.Serializable] class Manifest { public Entry[] items; }

        public static System.Collections.Generic.Dictionary<string, float[]> Parse(string json)
        {
            var result = new System.Collections.Generic.Dictionary<string, float[]>();
            foreach (var e in JsonUtility.FromJson<Manifest>("{\"items\":" + json + "}").items) result[e.name] = e.dimensions_xyz_m;
            return result;
        }
    }
}
