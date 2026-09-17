// The occupancy map of a brick assembly (Assets/Phys/BRICK_ASSEMBLY.md, "Occupancy map"): per stud cell the top of the
// highest piece standing over it, and the posts where figures stand (a garrison's places on the walls). A document may
// carry the section; a loader without it derives the map from the parts and the posts from the map.

using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Scenes
{
    /// <summary>A place where a figure stands: the surface point in grid units (y = the top it stands on) and the heading
    /// (radians about +y, 0 = facing +z).</summary>
    public struct AssemblyPost
    {
        public float3 Position;
        public float Yaw;
        /// <summary>On a wall or tower top (false: on the ground inside the walls).</summary>
        public bool OnWall;

        public float3 Forward => new float3(math.sin(Yaw), 0f, math.cos(Yaw));
    }

    /// <summary>A grid over the assembly's footprint, one cell per stud: the top of the highest body over the cell in grid units
    /// (0 = nothing stands there), and the posts.</summary>
    public sealed class AssemblyOccupancy
    {
        /// <summary>Grid cell (in the document's grid units) whose min corner is cell (0, 0) of the map.</summary>
        public readonly int2 Origin;
        /// <summary>Cells along x and z.</summary>
        public readonly int2 Size;
        /// <summary>[z * Size.x + x]: the top of the highest body over the cell, grid units; 0 = free.</summary>
        public readonly float[] Top;
        /// <summary>The posts: from the document (<see cref="ExplicitPosts"/>) or <see cref="DerivePosts"/>; wall posts first.</summary>
        public readonly List<AssemblyPost> Posts = new List<AssemblyPost>();
        public bool ExplicitPosts;

        /// <summary>How posts are derived from the map (grid units and cells).</summary>
        public struct PostRules
        {
            /// <summary>A wall post needs at least this much height under it (three courses).</summary>
            public float MinWallHeight;
            /// <summary>Cells of a post's footprint may differ this much from its top (a plate step).</summary>
            public float LevelTolerance;
            /// <summary>Minimum Chebyshev distance in cells between two posts of the same kind.</summary>
            public int Spacing;
            /// <summary>A wall post's outward walk must reach a drop within this many cells (the wall's thickness incl. a parapet).</summary>
            public int MaxWallThickness;

            public static PostRules Default => new PostRules { MinWallHeight = 3.6f, LevelTolerance = 0.45f, Spacing = 4, MaxWallThickness = 8 };
        }

        public AssemblyOccupancy(int2 origin, int2 size, float[] top = null)
        {
            Origin = origin;
            Size = math.max(size, 1);
            Top = top ?? new float[Size.x * Size.y];
        }

        public int Cells => Size.x * Size.y;

        /// <summary>Cells with something standing on them.</summary>
        public int SolidCells
        {
            get { int n = 0; foreach (float t in Top) if (t > 0f) n++; return n; }
        }

        /// <summary>The top over the grid cell (absolute grid coordinates), 0 outside the map.</summary>
        public float TopAt(int x, int z)
        {
            x -= Origin.x; z -= Origin.y;
            return x < 0 || z < 0 || x >= Size.x || z >= Size.y ? 0f : Top[z * Size.x + x];
        }

        public float TopAt(float2 xz) => TopAt((int)math.floor(xz.x), (int)math.floor(xz.y));

        /// <summary>Whether a point in grid units lies inside a body's column (below the cell's top). Openings under lintels read
        /// as solid: the map holds tops only.</summary>
        public bool Solid(float3 gridPoint) => gridPoint.y >= 0f && gridPoint.y < TopAt(gridPoint.xz) - 1e-3f;

        /// <summary>Whether two maps cover the same cells with the same tops (within <paramref name="tolerance"/>).</summary>
        public bool SameTops(AssemblyOccupancy other, float tolerance = 1e-3f)
        {
            if (other == null || math.any(other.Origin != Origin) || math.any(other.Size != Size)) return false;
            for (int i = 0; i < Top.Length; i++) if (math.abs(Top[i] - other.Top[i]) > tolerance) return false;
            return true;
        }

        // ------------------------------------------------------------------------------------------------ derivation

        /// <summary>The map of the parts' bodies: every part covers the cells whose centres lie within its bounding box on the
        /// ground plane (a 2 x 3 brick at x = 1 covers cells 0 and 1), the top is the box's top. Rotated and lying parts take
        /// their bounding box.</summary>
        public static AssemblyOccupancy FromParts(BrickAssembly a)
        {
            if (a.Parts.Count == 0) return new AssemblyOccupancy(int2.zero, new int2(1, 1));
            // a turned piece's bounds carry rounding dust (cos 90 degrees is not zero): keep it off the cell borders
            int2 origin = (int2)math.floor(a.Min.xz + 1e-4f), size = math.max((int2)math.ceil(a.Max.xz - 1e-4f) - origin, 1);
            var o = new AssemblyOccupancy(origin, size);
            foreach (var p in a.Parts)
            {
                PartBox(p, out float3 min, out float3 max);
                int x0 = (int)math.floor(min.x + 0.5f), x1 = (int)math.ceil(max.x - 0.5f);   // cells with centre in [min, max)
                int z0 = (int)math.floor(min.z + 0.5f), z1 = (int)math.ceil(max.z - 0.5f);
                x0 = math.max(x0, origin.x); z0 = math.max(z0, origin.y);
                x1 = math.min(x1, origin.x + size.x); z1 = math.min(z1, origin.y + size.y);
                float top = math.max(max.y, 0f);
                for (int z = z0; z < z1; z++)
                    for (int x = x0; x < x1; x++)
                    {
                        int i = (z - origin.y) * size.x + (x - origin.x);
                        if (top > o.Top[i]) o.Top[i] = top;
                    }
            }
            return o;
        }

        /// <summary>The body box of a part in grid units (the same box <see cref="BrickAssembly"/> bounds).</summary>
        public static void PartBox(in AssemblyPart p, out float3 min, out float3 max)
        {
            float3 half = PieceCatalog.Pieces[p.Piece].Size * 0.5f;
            float3 centre = p.Position + math.mul(p.Rotation, new float3(0f, half.y, 0f));
            float3 extent = math.abs(math.mul(p.Rotation, new float3(half.x, 0f, 0f))) + math.abs(math.mul(p.Rotation, new float3(0f, half.y, 0f))) + math.abs(math.mul(p.Rotation, new float3(0f, 0f, half.z)));
            min = centre - extent; max = centre + extent;
        }

        // ------------------------------------------------------------------------------------------------ posts

        struct Candidate { public int X, Z; public float3 Position; public float Yaw; public float Rank; public float Angle; }

        /// <summary>Derives the posts unless the document set them: first the wall posts - a cell on top of a wall or tower (at
        /// least <see cref="PostRules.MinWallHeight"/> high) level with its two neighbours along the wall (and, when that is
        /// level too, the cell inward: the footprint's centre moves half a cell in), whose outward walk (away from the map's
        /// centre along the dominant axis) drops within the wall's thickness and meets nothing as high again before the border;
        /// posts nearest to the outer edge come first, then those with equally high walls beyond them (an inner ward), spread
        /// around the centre by angle at least <see cref="PostRules.Spacing"/> cells apart, facing outward - then the ground
        /// posts: cells below wall height whose 3 x 3 neighbourhood is level (a paved or raised courtyard floor), enclosed in all
        /// four axis directions by cells a wall's height above them, nearest to a wall first, facing it. Returns <see cref="Posts"/>.</summary>
        public List<AssemblyPost> DerivePosts(PostRules rules)
        {
            if (ExplicitPosts) return Posts;
            Posts.Clear();
            float2 centre = (float2)Origin + (float2)Size * 0.5f;
            var walls = new List<Candidate>();
            var ground = new List<Candidate>();
            for (int z = 0; z < Size.y; z++)
                for (int x = 0; x < Size.x; x++)
                {
                    float h = Top[z * Size.x + x];
                    if (h >= rules.MinWallHeight) WallCandidate(x, z, h, centre, rules, walls);
                    else GroundCandidate(x, z, h, centre, rules, ground);
                }
            walls.Sort((a, b) => a.Rank != b.Rank ? a.Rank.CompareTo(b.Rank) : a.Angle != b.Angle ? a.Angle.CompareTo(b.Angle) : a.Z != b.Z ? a.Z.CompareTo(b.Z) : a.X.CompareTo(b.X));
            ground.Sort((a, b) => a.Rank != b.Rank ? a.Rank.CompareTo(b.Rank) : a.Angle != b.Angle ? a.Angle.CompareTo(b.Angle) : a.Z != b.Z ? a.Z.CompareTo(b.Z) : a.X.CompareTo(b.X));
            Accept(walls, rules.Spacing, true);
            Accept(ground, rules.Spacing, false);
            return Posts;
        }

        void Accept(List<Candidate> candidates, int spacing, bool onWall)
        {
            var taken = new List<int2>();
            foreach (var c in candidates)
            {
                bool clear = true;
                foreach (var t in taken)
                    if (math.max(math.abs(t.x - c.X), math.abs(t.y - c.Z)) < spacing) { clear = false; break; }
                if (!clear) continue;
                taken.Add(new int2(c.X, c.Z));
                Posts.Add(new AssemblyPost { Position = c.Position, Yaw = c.Yaw, OnWall = onWall });
            }
        }

        float TopLocal(int x, int z) => x < 0 || z < 0 || x >= Size.x || z >= Size.y ? 0f : Top[z * Size.x + x];

        static void Axes(int x, int z, float2 centre, int2 origin, out int2 outward, out int2 along)
        {
            float2 v = new float2(origin.x + x + 0.5f, origin.y + z + 0.5f) - centre;
            outward = math.abs(v.x) >= math.abs(v.y) ? new int2(v.x >= 0f ? 1 : -1, 0) : new int2(0, v.y >= 0f ? 1 : -1);
            along = new int2(outward.y, outward.x);
        }

        void WallCandidate(int x, int z, float h, float2 centre, in PostRules rules, List<Candidate> result)
        {
            Axes(x, z, centre, Origin, out int2 outward, out int2 along);
            int2 inward = -outward;
            // the footprint: this cell and its two neighbours along the wall, level with each other (a merlon next to it disqualifies)
            float top = h;
            for (int i = -1; i <= 1; i += 2)
            {
                int cx = x + along.x * i, cz = z + along.y * i;
                if (cx < 0 || cz < 0 || cx >= Size.x || cz >= Size.y) return;
                float t = TopLocal(cx, cz);
                if (math.abs(t - h) > rules.LevelTolerance) return;
                top = math.max(top, t);
            }
            // a walk two cells deep lets the figure stand back from the edge
            bool deep = true;
            for (int i = -1; i <= 1; i++)
            {
                int cx = x + along.x * i + inward.x, cz = z + along.y * i + inward.y;
                if (cx < 0 || cz < 0 || cx >= Size.x || cz >= Size.y || math.abs(TopLocal(cx, cz) - h) > rules.LevelTolerance) { deep = false; break; }
            }
            // the outward walk: a drop within the wall's thickness, then nothing as high again (or as few such cells as possible)
            int drop = -1, blocked = 0;
            for (int k = 1; ; k++)
            {
                int cx = x + outward.x * k, cz = z + outward.y * k;
                if (cx < 0 || cz < 0 || cx >= Size.x || cz >= Size.y) break;
                float t = TopLocal(cx, cz);
                if (drop < 0) { if (t < top - rules.LevelTolerance) drop = k; else if (k > rules.MaxWallThickness) return; }
                else if (t >= top - rules.LevelTolerance) blocked++;
            }
            if (drop < 0) return;   // the wall runs to the border: not a wall top
            float2 cell = new float2(Origin.x + x + 0.5f, Origin.y + z + 0.5f) + (deep ? (float2)inward * 0.5f : float2.zero);   // the footprint's centre
            float2 rel = cell - centre;
            result.Add(new Candidate
            {
                X = x, Z = z, Position = new float3(cell.x, top, cell.y), Yaw = math.atan2(outward.x, outward.y),
                Rank = blocked * 1000f + drop, Angle = math.atan2(rel.x, rel.y),
            });
        }

        void GroundCandidate(int x, int z, float h, float2 centre, in PostRules rules, List<Candidate> result)
        {
            // a level floor: the ground, a pavement or a raised courtyard, as long as it is below wall height
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                {
                    int cx = x + i, cz = z + j;
                    if (cx < 0 || cz < 0 || cx >= Size.x || cz >= Size.y) return;   // the border is not enclosed
                    if (math.abs(TopLocal(cx, cz) - h) > rules.LevelTolerance) return;
                }
            // enclosed: a wall's height above the floor in every axis direction before the border; face the nearest one
            float wall = h + rules.MinWallHeight;
            int nearest = int.MaxValue; int2 toWall = default;
            int2[] dirs = { new int2(1, 0), new int2(-1, 0), new int2(0, 1), new int2(0, -1) };
            foreach (var d in dirs)
            {
                int k = 1;
                for (; ; k++)
                {
                    int cx = x + d.x * k, cz = z + d.y * k;
                    if (cx < 0 || cz < 0 || cx >= Size.x || cz >= Size.y) return;   // open to the border
                    if (TopLocal(cx, cz) >= wall) break;
                }
                if (k < nearest) { nearest = k; toWall = d; }
            }
            float2 cell = new float2(Origin.x + x + 0.5f, Origin.y + z + 0.5f);
            float2 rel = cell - centre;
            result.Add(new Candidate
            {
                X = x, Z = z, Position = new float3(cell.x, h, cell.y), Yaw = math.atan2(toWall.x, toWall.y),
                Rank = nearest, Angle = math.atan2(rel.x, rel.y),
            });
        }

        // ------------------------------------------------------------------------------------------------ JSON

        /// <summary>Parses the document's "occupancy" object; throws <see cref="BrickAssemblyException"/> for anything malformed.</summary>
        internal static AssemblyOccupancy Parse(Dictionary<string, object> obj, string where)
        {
            int2 origin = GetInt2(obj, "origin", where), size = GetInt2(obj, "size", where);
            if (size.x < 1 || size.y < 1 || (long)size.x * size.y > 16_000_000) throw new BrickAssemblyException($"{where}: 'size' must be two positive integers");
            var rows = BrickAssembly.GetArray(obj, "rows", where, true);
            if (rows.Count != size.y) throw new BrickAssemblyException($"{where}: 'rows' has {rows.Count} rows, 'size' says {size.y}");
            var o = new AssemblyOccupancy(origin, size);
            for (int z = 0; z < size.y; z++)
            {
                if (!(rows[z] is List<object> row) || row.Count != size.x) throw new BrickAssemblyException($"{where}: row {z} is not an array of {size.x} numbers");
                for (int x = 0; x < size.x; x++)
                {
                    if (!(row[x] is double d) || double.IsNaN(d) || double.IsInfinity(d) || d < 0) throw new BrickAssemblyException($"{where}: row {z} cell {x} is not a finite non-negative number");
                    o.Top[z * size.x + x] = (float)d;
                }
            }
            var posts = BrickAssembly.GetArray(obj, "posts", where, false);
            if (posts != null)
            {
                o.ExplicitPosts = true;
                for (int i = 0; i < posts.Count; i++)
                {
                    if (!(posts[i] is List<object> p) || p.Count < 3 || p.Count > 4) throw new BrickAssemblyException($"{where}: post {i} is not [x, y, z] or [x, y, z, yaw]");
                    var v = new float[4];
                    for (int k = 0; k < p.Count; k++)
                    {
                        if (!(p[k] is double d) || double.IsNaN(d) || double.IsInfinity(d)) throw new BrickAssemblyException($"{where}: post {i} is not finite numbers");
                        v[k] = (float)d;
                    }
                    var position = new float3(v[0], v[1], v[2]);
                    o.Posts.Add(new AssemblyPost { Position = position, Yaw = math.radians(v[3]), OnWall = position.y > 0.5f });
                }
            }
            return o;
        }

        static int2 GetInt2(Dictionary<string, object> obj, string key, string where)
        {
            var list = BrickAssembly.GetArray(obj, key, where, true);
            if (list.Count != 2) throw new BrickAssemblyException($"{where}: '{key}' is not two integers");
            var v = new int2();
            for (int i = 0; i < 2; i++)
            {
                if (!(list[i] is double d) || d != math.floor(d) || math.abs(d) > 1e6) throw new BrickAssemblyException($"{where}: '{key}' is not two integers");
                v[i] = (int)d;
            }
            return v;
        }

        /// <summary>Writes the "occupancy" object (without the key) with its posts.</summary>
        public void Write(StringBuilder sb, string indent)
        {
            string inner = indent + "  ";
            sb.Append("{\n").Append(inner).Append("\"origin\": [").Append(Origin.x).Append(", ").Append(Origin.y).Append("],\n");
            sb.Append(inner).Append("\"size\": [").Append(Size.x).Append(", ").Append(Size.y).Append("],\n");
            sb.Append(inner).Append("\"rows\": [\n");
            for (int z = 0; z < Size.y; z++)
            {
                sb.Append(inner).Append("  [");
                for (int x = 0; x < Size.x; x++) sb.Append(x > 0 ? ", " : "").Append(Number(Top[z * Size.x + x]));
                sb.Append(z + 1 < Size.y ? "],\n" : "]\n");
            }
            sb.Append(inner).Append(Posts.Count > 0 ? "]," : "]").Append('\n');
            if (Posts.Count > 0)
            {
                sb.Append(inner).Append("\"posts\": [\n");
                for (int i = 0; i < Posts.Count; i++)
                {
                    var p = Posts[i];
                    sb.Append(inner).Append("  [").Append(Number(p.Position.x)).Append(", ").Append(Number(p.Position.y)).Append(", ").Append(Number(p.Position.z))
                      .Append(", ").Append(Number(math.degrees(p.Yaw))).Append(i + 1 < Posts.Count ? "],\n" : "]\n");
                }
                sb.Append(inner).Append("]\n");
            }
            sb.Append(indent).Append('}');
        }

        static string Number(float v) => v.ToString(math.abs(v - math.round(v)) < 1e-6f ? "0" : "0.###", CultureInfo.InvariantCulture);

        /// <summary>The document text with this map as its "occupancy" section (an existing section replaced, otherwise appended as
        /// the last root field); the rest of the text is kept as written.</summary>
        public static string Splice(string json, AssemblyOccupancy o)
        {
            var text = new StringBuilder(json.TrimEnd());
            int key = FindKey(text, "occupancy");
            if (key >= 0)
            {
                int open = text.ToString().IndexOf('{', key);
                int close = MatchBrace(text, open);
                int start = key, end = close + 1;
                // take the separating comma with it: the one before the section, or the one after when it was not the last field
                int before = start - 1;
                while (before >= 0 && char.IsWhiteSpace(text[before])) before--;
                int after = end;
                while (after < text.Length && char.IsWhiteSpace(text[after])) after++;
                if (after < text.Length && text[after] == ',') end = after + 1;
                else if (before >= 0 && text[before] == ',') start = before;
                text.Remove(start, end - start);
            }
            int last = text.Length - 1;
            while (last >= 0 && text[last] != '}') last--;
            if (last < 0) throw new BrickAssemblyException("the document is not a JSON object");
            int prev = last - 1;
            while (prev >= 0 && char.IsWhiteSpace(text[prev])) prev--;
            var section = new StringBuilder();
            if (prev >= 0 && text[prev] != '{' && text[prev] != ',') section.Append(',');
            section.Append("\n  \"occupancy\": ");
            o.Write(section, "  ");
            section.Append('\n');
            text.Remove(prev + 1, last - prev - 1);
            text.Insert(prev + 1, section.ToString());
            text.Append('\n');
            return text.ToString();
        }

        /// <summary>Index of the quoted key outside any string value, or -1.</summary>
        static int FindKey(StringBuilder text, string key)
        {
            string quoted = "\"" + key + "\"";
            bool inString = false;
            for (int i = 0; i < text.Length; i++)
            {
                char c = text[i];
                if (inString) { if (c == '\\') i++; else if (c == '"') inString = false; continue; }
                if (c == '"')
                {
                    if (string.CompareOrdinal(text.ToString(i, math.min(quoted.Length, text.Length - i)), quoted) == 0)
                    {
                        int j = i + quoted.Length;
                        while (j < text.Length && char.IsWhiteSpace(text[j])) j++;
                        if (j < text.Length && text[j] == ':') return i;
                    }
                    inString = true;
                }
            }
            return -1;
        }

        static int MatchBrace(StringBuilder text, int open)
        {
            int depth = 0; bool inString = false;
            for (int i = open; i < text.Length; i++)
            {
                char c = text[i];
                if (inString) { if (c == '\\') i++; else if (c == '"') inString = false; continue; }
                if (c == '"') inString = true;
                else if (c == '{' || c == '[') depth++;
                else if (c == '}' || c == ']') { depth--; if (depth == 0) return i; }
            }
            throw new BrickAssemblyException("unbalanced braces in the occupancy section");
        }
    }
}
