using System.IO;
using NUnit.Framework;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>The occupancy map of a brick assembly (BRICK_ASSEMBLY.md, "Occupancy map"): derived from the parts, parsed from
    /// and written into documents, and the posts it yields for a garrison.</summary>
    public class AssemblyOccupancyTests
    {
        const string Bridge = @"{
  ""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""Small bridge"",
  ""units"": { ""gridToUnity"": 0.1, ""axes"": ""unity"" },
  ""parts"": [
    { ""id"": ""left_support"",  ""piece"": ""Brick_2x2"", ""position"": [-3, 0, 0] },
    { ""id"": ""right_support"", ""piece"": ""Brick_2x2"", ""position"": [3, 0, 0] },
    { ""id"": ""bridge_deck"",   ""piece"": ""Brick_2x8"", ""position"": [0, 1.2, 0], ""rotation"": [0, 90, 0] }
  ]
}";

        static string WithOccupancy(string body) => @"{
  ""format"": ""brick-assembly"", ""version"": 1, ""catalog"": ""generic-construction-27-v1"", ""name"": ""Occupied"",
  ""units"": { ""gridToUnity"": 0.1, ""axes"": ""unity"" },
  ""parts"": [ { ""id"": ""a"", ""piece"": ""Brick_1x1"", ""position"": [0.5, 0, 0.5] } ],
  ""occupancy"": " + body + "\n}";

        [Test]
        public void DerivedBridgeCoversTheSupportsAndTheTurnedDeck()
        {
            var a = BrickAssembly.Parse(Bridge);
            Assert.IsNull(a.Occupancy, "the bridge carries no section");
            var o = a.OccupancyOrDerived();
            Assert.AreSame(o, a.Occupancy, "derived once, then kept");
            Assert.AreEqual(new int2(-4, -1), o.Origin);
            Assert.AreEqual(new int2(8, 2), o.Size);
            // the deck spans x = -4 .. 4 at 2.4 high; the supports underneath end at 1.2 but the deck tops them
            for (int x = -4; x < 4; x++) Assert.AreEqual(2.4f, o.TopAt(x, 0), 1e-4f, $"cell {x}, 0");
            Assert.AreEqual(0f, o.TopAt(4, 0), "beyond the deck");
            Assert.AreEqual(0f, o.TopAt(0, 5), "outside the map");
            Assert.IsTrue(o.Solid(new float3(-3.5f, 0.6f, 0.2f)), "inside a support");
            Assert.IsTrue(o.Solid(new float3(0.2f, 2.0f, 0.2f)), "under the deck's top reads as solid (tops only)");
            Assert.IsFalse(o.Solid(new float3(0.2f, 2.5f, 0.2f)), "above the deck");
            Assert.IsFalse(o.Solid(new float3(6f, 0.5f, 0f)), "off the map");
        }

        [Test]
        public void SectionParsesAndExplicitPostsWin()
        {
            var a = BrickAssembly.Parse(WithOccupancy(@"{ ""origin"": [0, 0], ""size"": [3, 2], ""rows"": [[1.2, 0, 0], [0, 0, 6]], ""posts"": [[1.5, 6, 1.5, 90], [0.5, 0, 0.5]] }"));
            var o = a.Occupancy;
            Assert.IsNotNull(o);
            Assert.AreEqual(new int2(3, 2), o.Size);
            Assert.AreEqual(1.2f, o.TopAt(0, 0), 1e-6f);
            Assert.AreEqual(6f, o.TopAt(2, 1), 1e-6f);
            Assert.IsTrue(o.ExplicitPosts);
            Assert.AreEqual(2, o.Posts.Count);
            Assert.AreEqual(math.radians(90f), o.Posts[0].Yaw, 1e-6f);
            Assert.IsTrue(o.Posts[0].OnWall);
            Assert.IsFalse(o.Posts[1].OnWall);
            Assert.AreEqual(0f, o.Posts[1].Yaw);
            int before = o.Posts.Count;
            o.DerivePosts(AssemblyOccupancy.PostRules.Default);
            Assert.AreEqual(before, o.Posts.Count, "explicit posts are kept");
            Assert.AreSame(o, a.OccupancyOrDerived(), "the document's map is used, not a derived one");
        }

        [TestCase(@"{ ""origin"": [0, 0], ""size"": [3, 2], ""rows"": [[1, 0, 0]] }", "rows")]
        [TestCase(@"{ ""origin"": [0, 0], ""size"": [3, 2], ""rows"": [[1, 0], [0, 0, 0]] }", "row 0")]
        [TestCase(@"{ ""origin"": [0, 0], ""size"": [3, 2], ""rows"": [[1, 0, ""x""], [0, 0, 0]] }", "row 0 cell 2")]
        [TestCase(@"{ ""origin"": [0, 0], ""size"": [3, 2], ""rows"": [[1, 0, -1], [0, 0, 0]] }", "row 0 cell 2")]
        [TestCase(@"{ ""origin"": [0.5, 0], ""size"": [3, 2], ""rows"": [[1, 0, 0], [0, 0, 0]] }", "origin")]
        [TestCase(@"{ ""origin"": [0, 0], ""size"": [0, 2], ""rows"": [] }", "size")]
        [TestCase(@"{ ""origin"": [0, 0], ""size"": [1, 1], ""rows"": [[1]], ""posts"": [[1, 2]] }", "post 0")]
        public void MalformedSectionsAreRejected(string section, string message)
        {
            var e = Assert.Throws<BrickAssemblyException>(() => BrickAssembly.Parse(WithOccupancy(section)));
            StringAssert.Contains(message, e.Message);
        }

        [Test]
        public void WrittenSectionEqualsTheDerivedMapAndRoundTrips()
        {
            var plan = CastlePlan.Presets[0];
            var layout = BrickCastle.Generate(plan);
            string json = BrickAssembly.WriteLayout(layout, plan.Name);
            var a = BrickAssembly.Parse(json);
            Assert.IsNotNull(a.Occupancy, "the writer emits the section");
            var derived = AssemblyOccupancy.FromParts(a);
            Assert.IsTrue(a.Occupancy.SameTops(derived), "the layout's map equals the one derived from the written parts");
            Assert.IsTrue(a.Occupancy.ExplicitPosts);
            derived.DerivePosts(AssemblyOccupancy.PostRules.Default);
            Assert.AreEqual(derived.Posts.Count, a.Occupancy.Posts.Count, "the written posts are the derived ones");
            for (int i = 0; i < derived.Posts.Count; i++)
            {
                Assert.Less(math.distance(derived.Posts[i].Position, a.Occupancy.Posts[i].Position), 1e-3f, $"post {i} position");
                Assert.AreEqual(derived.Posts[i].Yaw, a.Occupancy.Posts[i].Yaw, 1e-4f, $"post {i} yaw");
            }
            // splicing the section into a document without one, and again into one with it, gives the same map
            string bare = json.Substring(0, json.IndexOf(",\n  \"occupancy\"")) + "\n}\n";
            var withoutSection = BrickAssembly.Parse(bare);
            Assert.IsNull(withoutSection.Occupancy);
            string spliced = AssemblyOccupancy.Splice(bare, derived);
            var again = BrickAssembly.Parse(spliced);
            Assert.IsTrue(again.Occupancy.SameTops(derived));
            Assert.AreEqual(derived.Posts.Count, again.Occupancy.Posts.Count);
            string twice = AssemblyOccupancy.Splice(spliced, derived);
            Assert.AreEqual(1, CountOf(twice, "\"occupancy\""), "a second splice replaces the section");
            Assert.AreEqual(again.Parts.Count, BrickAssembly.Parse(twice).Parts.Count);
        }

        [Test]
        public void OutpostPostsStandOnTheOuterWallTops()
        {
            var layout = BrickCastle.Generate(CastlePlan.Presets[0]);
            var o = BrickAssembly.LayoutOccupancy(layout);
            int walls = 0, ground = 0;
            float2 centre = (float2)o.Origin + (float2)o.Size * 0.5f;
            var cells = new System.Collections.Generic.List<int2>();
            foreach (var p in o.Posts)
            {
                if (!p.OnWall) { ground++; Assert.LessOrEqual(p.Position.y, 0.5f, "a ground post stands on the ground"); continue; }
                walls++;
                Assert.GreaterOrEqual(p.Position.y, 3.6f, "a wall post stands at least three courses up");
                Assert.AreEqual(o.TopAt(p.Position.xz), p.Position.y, 0.5f, "on the top of its footprint");
                float3 forward = p.Forward;
                Assert.Greater(math.dot(forward.xz, p.Position.xz - centre), 0f, "facing outward");
                var cell = (int2)math.floor(p.Position.xz);
                foreach (var other in cells) Assert.GreaterOrEqual(math.cmax(math.abs(other - cell)), 3, "posts keep their spacing");
                cells.Add(cell);
            }
            Assert.GreaterOrEqual(walls, 8, "a ring of wall posts");
            Assert.Greater(ground, 0, "courtyard posts follow");
            Assert.IsTrue(o.Posts[0].OnWall, "wall posts come first");
        }

        [Test]
        public void CitadelHasWallPostsAndPavementGroundPosts()
        {
            var a = BrickAssembly.Parse(File.ReadAllText("Assets/Resources/Castles/emerald_crown_citadel.json"));
            var o = AssemblyOccupancy.FromParts(a);
            o.DerivePosts(AssemblyOccupancy.PostRules.Default);
            int walls = 0, ground = 0;
            foreach (var p in o.Posts)
            {
                if (p.OnWall) { walls++; Assert.GreaterOrEqual(p.Position.y, 3.6f); }
                else { ground++; Assert.Less(p.Position.y, 3.6f); Assert.GreaterOrEqual(p.Position.y, 0f); Assert.AreEqual(o.TopAt(p.Position.xz), p.Position.y, 1e-3f, "on its floor"); }
            }
            Assert.GreaterOrEqual(walls, 8, $"wall posts ({o.Posts.Count} posts)");
            Assert.Greater(ground, 0, "ground posts on the raised courtyard floor inside the walls");
        }

        static int CountOf(string text, string needle)
        {
            int n = 0;
            for (int i = text.IndexOf(needle, System.StringComparison.Ordinal); i >= 0; i = text.IndexOf(needle, i + needle.Length, System.StringComparison.Ordinal)) n++;
            return n;
        }
    }
}
