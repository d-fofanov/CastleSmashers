using System.Collections.Generic;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Siege
{
    /// <summary>Where armies stand at the start: the attackers in ranks facing one side of the castle, the defenders on the posts of
    /// the occupancy map.</summary>
    public static class Formations
    {
        /// <summary>A roster entry with its archetype's footprint (solver metres) and range.</summary>
        public struct Rank
        {
            public int Type, Count;
            public float Width, Depth, Range;
        }

        /// <summary>Ranks facing the castle from the attacked side: the entries sorted by range (the short-ranged in front), each laid
        /// out in rows of at most <paramref name="maxColumns"/> centred on the face, the columns <paramref name="columnSpacing"/> (or 1.4
        /// footprints) apart, the rows <paramref name="rankSpacing"/> (or 1.4 footprints) apart, the first row
        /// <paramref name="formationDistance"/> beyond the face at <paramref name="faceDistance"/> from the centre along
        /// <paramref name="outward"/>. Positions are ground points (y = 0) about the castle's centre; the yaw faces the castle.</summary>
        public static List<(int type, float3 position, float yaw)> Ranks(IReadOnlyList<Rank> ranks, float3 outward, float faceDistance, float formationDistance,
            float columnSpacing, float rankSpacing, int maxColumns)
        {
            var result = new List<(int, float3, float)>();
            var sorted = new List<Rank>(ranks);
            sorted.Sort((a, b) => a.Range != b.Range ? a.Range.CompareTo(b.Range) : a.Type.CompareTo(b.Type));
            outward = math.normalizesafe(new float3(outward.x, 0f, outward.z), new float3(0, 0, -1));
            float3 along = new float3(outward.z, 0f, -outward.x);
            float yaw = Battle.YawOf(-outward);
            float depth = 0f;
            foreach (var r in sorted)
            {
                if (r.Count <= 0) continue;
                int cols = math.max(1, math.min(r.Count, maxColumns));
                float dx = math.max(columnSpacing, r.Width * 1.4f), dz = math.max(rankSpacing, r.Depth * 1.4f);
                int placed = 0;
                while (placed < r.Count)
                {
                    int n = math.min(cols, r.Count - placed);
                    for (int c = 0; c < n; c++)
                    {
                        float3 p = outward * (faceDistance + formationDistance + depth) + along * ((c - (n - 1) * 0.5f) * dx);
                        result.Add((r.Type, p, yaw));
                    }
                    placed += n;
                    depth += dz;
                }
            }
            return result;
        }

        /// <summary>The depth of the formation laid out by <see cref="Ranks"/> (the same rows).</summary>
        public static float Depth(IReadOnlyList<Rank> ranks, float rankSpacing, int maxColumns)
        {
            float depth = 0f;
            foreach (var r in ranks)
            {
                if (r.Count <= 0) continue;
                int cols = math.max(1, math.min(r.Count, maxColumns));
                depth += ((r.Count + cols - 1) / cols) * math.max(rankSpacing, r.Depth * 1.4f);
            }
            return depth;
        }

        /// <summary>The post of every unit: the posts spread evenly over the first units (every post used once when there are enough),
        /// wrapping around when the units outnumber them.</summary>
        public static int[] AssignPosts(int postCount, int units)
        {
            var result = new int[math.max(units, 0)];
            if (postCount <= 0) return result;
            int take = math.min(units, postCount);
            for (int i = 0; i < units; i++) result[i] = i < take ? (int)((long)i * postCount / take) : i % postCount;
            return result;
        }
    }
}
