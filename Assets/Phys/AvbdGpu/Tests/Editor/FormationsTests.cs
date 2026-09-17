using System.Collections.Generic;
using NUnit.Framework;
using Phys.AvbdGpu.Siege;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Tests
{
    /// <summary>Where the armies form up: ranks by range on the attacked side, posts spread over the garrison.</summary>
    public class FormationsTests
    {
        [Test]
        public void ShortRangedTypesStandInFrontInCentredRows()
        {
            var ranks = new List<Formations.Rank>
            {
                new Formations.Rank { Type = 0, Count = 2, Width = 3.6f, Depth = 6.6f, Range = 60f },   // trebuchets
                new Formations.Rank { Type = 1, Count = 5, Width = 1.7f, Depth = 0.6f, Range = 25f },   // archers (1.4 footprints < the column spacing)
            };
            var placed = Formations.Ranks(ranks, new float3(0, 0, -1), faceDistance: 20f, formationDistance: 30f, columnSpacing: 2.5f, rankSpacing: 3f, maxColumns: 3);
            Assert.AreEqual(7, placed.Count);
            // archers first (range 25), in two rows of three and two, each row centred on the face
            for (int i = 0; i < 5; i++) Assert.AreEqual(1, placed[i].type);
            for (int i = 5; i < 7; i++) Assert.AreEqual(0, placed[i].type);
            Assert.AreEqual(-50f, placed[0].position.z, 1e-4f, "the first row stands 30 m beyond the face at 20 m");
            Assert.AreEqual(2.5f, placed[0].position.x, 1e-4f); Assert.AreEqual(0f, placed[1].position.x, 1e-4f); Assert.AreEqual(-2.5f, placed[2].position.x, 1e-4f);   // the row runs along -x seen from the castle
            Assert.AreEqual(-53f, placed[3].position.z, 1e-4f, "the second row a rank spacing further out");
            Assert.AreEqual(1.25f, placed[3].position.x, 1e-4f, "two units centred");
            Assert.AreEqual(-56f, placed[5].position.z, 1e-4f, "the trebuchets behind the archers");
            Assert.AreEqual(math.max(2.5f, 3.6f * 1.4f) * 0.5f, math.abs(placed[5].position.x), 1e-4f, "wide machines spaced by their footprint");
            foreach (var p in placed) Assert.AreEqual(0f, p.yaw, 1e-4f, "facing +z, toward the castle");
            Assert.AreEqual(3f + 3f + 6.6f * 1.4f, Formations.Depth(ranks, 3f, 3), 1e-4f);
        }

        [Test]
        public void OtherSidesTurnTheFormation()
        {
            var ranks = new List<Formations.Rank> { new Formations.Rank { Type = 0, Count = 3, Width = 1.7f, Depth = 0.6f, Range = 25f } };
            var east = Formations.Ranks(ranks, new float3(1, 0, 0), 10f, 20f, 2.5f, 3f, 12);
            Assert.AreEqual(30f, east[0].position.x, 1e-4f);
            Assert.AreEqual(math.atan2(-1f, 0f), east[0].yaw, 1e-4f, "facing -x");
            Assert.AreEqual(2.5f, math.abs(east[0].position.z), 1e-4f, "the row runs along z");
        }

        [Test]
        public void PostsSpreadEvenlyAndWrap()
        {
            CollectionAssert.AreEqual(new[] { 0, 2, 5, 7 }, Formations.AssignPosts(10, 4), "four units over ten posts, spread out");
            CollectionAssert.AreEqual(new[] { 0, 1, 2 }, Formations.AssignPosts(3, 3));
            CollectionAssert.AreEqual(new[] { 0, 1, 2, 0, 1 }, Formations.AssignPosts(3, 5), "more units than posts wrap around");
            CollectionAssert.AreEqual(new[] { 0, 0, 0 }, Formations.AssignPosts(0, 3), "no posts: everyone on post 0");
        }
    }
}
