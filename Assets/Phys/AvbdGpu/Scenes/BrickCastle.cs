// A castle built from the 2 x 3 construction brick (Assets/Models/ConstructorBlock2x3): the layout is planned on the stud grid
// (integer cells, courses) and then turned into solver boxes. The brick model's studs nest in the hollow underside of the brick
// above, so the collision box is the brick body without the studs; consecutive courses sit exactly one body height apart.

using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace Phys.AvbdGpu.Scenes
{
    /// <summary>Dimensions of the construction brick model (metres). The FBX is 23.8 generator units = 0.3 m long, studs 8 units apart.</summary>
    public static class Brick
    {
        public const float Pitch = 0.3f * 8f / 23.8f;                  // stud pitch 0.1008
        public const float Width = 0.199159664f;                        // 2 studs (model x)
        public const float Length = 0.3f;                               // 3 studs (model z)
        public const float BodyHeight = 0.3f * 9.6f / 23.8f;            // 0.1210, without the studs
        public const float StudHeight = 0.142436975f - BodyHeight;      // 0.0214
    }

    public enum BrickTone : byte { Stone = 0, Slate = 1, Tan = 2, Red = 3, Wood = 4 }

    /// <summary>A brick on the stud grid: its min-corner cell (X, Z), course (Layer) and orientation (2 x 3 or 3 x 2 cells).</summary>
    public struct BrickPlacement
    {
        public int X, Z, Layer;
        public bool Rotated;
        public BrickTone Tone;
        public int W => Rotated ? 3 : 2;
        public int D => Rotated ? 2 : 3;
    }

    /// <summary>Bricks on the stud grid with an occupancy map (overlaps are rejected, support can be checked).</summary>
    public sealed class BrickLayout
    {
        public readonly List<BrickPlacement> Bricks = new List<BrickPlacement>();
        readonly Dictionary<long, int> m_Cells = new Dictionary<long, int>();
        public int Layers { get; private set; }

        static long Key(int x, int z, int layer) => ((long)layer << 42) | ((long)(x + (1 << 20)) << 21) | (long)(z + (1 << 20));

        public int Add(int x, int z, int layer, bool rotated, BrickTone tone)
        {
            var b = new BrickPlacement { X = x, Z = z, Layer = layer, Rotated = rotated, Tone = tone };
            for (int i = 0; i < b.W; i++)
                for (int j = 0; j < b.D; j++)
                    if (m_Cells.ContainsKey(Key(x + i, z + j, layer)))
                        throw new InvalidOperationException($"BrickLayout: cell ({x + i}, {z + j}) of course {layer} is already occupied");
            int index = Bricks.Count;
            Bricks.Add(b);
            for (int i = 0; i < b.W; i++)
                for (int j = 0; j < b.D; j++)
                    m_Cells[Key(x + i, z + j, layer)] = index;
            Layers = math.max(Layers, layer + 1);
            return index;
        }

        /// <summary>Brick index occupying the cell, or -1.</summary>
        public int BrickAt(int x, int z, int layer) => m_Cells.TryGetValue(Key(x, z, layer), out int i) ? i : -1;

        /// <summary>Fraction of the brick's cells resting on the ground (course 0) or on a brick of the course below.</summary>
        public float Support(int index)
        {
            var b = Bricks[index];
            if (b.Layer == 0) return 1f;
            int supported = 0;
            for (int i = 0; i < b.W; i++)
                for (int j = 0; j < b.D; j++)
                    if (BrickAt(b.X + i, b.Z + j, b.Layer - 1) >= 0) supported++;
            return supported / (float)(b.W * b.D);
        }

        /// <summary>Bricks of the course below overlapping brick <paramref name="index"/>, with the overlapping cells.</summary>
        public List<(int brick, List<(int x, int z)> cells)> Supporters(int index)
        {
            var result = new List<(int, List<(int, int)>)>();
            var b = Bricks[index];
            if (b.Layer == 0) return result;
            for (int i = 0; i < b.W; i++)
                for (int j = 0; j < b.D; j++)
                {
                    int below = BrickAt(b.X + i, b.Z + j, b.Layer - 1);
                    if (below < 0) continue;
                    int k = result.FindIndex(r => r.Item1 == below);
                    if (k < 0) { result.Add((below, new List<(int, int)>())); k = result.Count - 1; }
                    result[k].Item2.Add((b.X + i, b.Z + j));
                }
            return result;
        }
    }

    /// <summary>Castle dimensions in studs and courses. Sides must be 38 + 12 k so that the walls tile in English bond.</summary>
    public struct CastlePlan
    {
        public string Name;
        public int Side;
        public int WallCourses, TowerCourses, GateCourses, TurretCourses, KeepCourses, KeepSize;

        public static readonly CastlePlan[] Presets =
        {
            new CastlePlan { Name = "Small castle", Side = 38, WallCourses = 8, TowerCourses = 14, GateCourses = 12, TurretCourses = 5, KeepCourses = 18, KeepSize = 14 },
            new CastlePlan { Name = "Castle", Side = 50, WallCourses = 10, TowerCourses = 18, GateCourses = 14, TurretCourses = 6, KeepCourses = 24, KeepSize = 14 },
            new CastlePlan { Name = "Large castle", Side = 74, WallCourses = 14, TowerCourses = 26, GateCourses = 20, TurretCourses = 8, KeepCourses = 34, KeepSize = 20 },
        };
    }

    /// <summary>How the layout maps to solver bodies: model scale, material, and where the castle stands.</summary>
    public struct BrickSpec
    {
        /// <summary>Solver metres per model metre (1 = the true 0.2 x 0.12 x 0.3 m brick).</summary>
        public float Scale;
        public float Density, Friction;
        /// <summary>The solver's collision margin: resting contacts settle this deep, so the box is that much taller than the brick
        /// body and the courses are laid exactly one body height apart.</summary>
        public float Margin;
        /// <summary>World position of stud-grid cell (0, 0) at ground level.</summary>
        public float3 Origin;

        /// <summary>Both solvers' COLLISION_MARGIN (AvbdGpuConstants.CollisionMargin; the Scenes assembly cannot reference it).</summary>
        public const float CollisionMargin = 0.01f;

        public static BrickSpec Default => new BrickSpec { Scale = 1f, Density = 1050f, Friction = 0.6f, Margin = CollisionMargin, Origin = float3.zero };

        public float3 BoxSize => new float3(Brick.Width * Scale, Brick.BodyHeight * Scale + Margin, Brick.Length * Scale);
        /// <summary>The model pivot (bottom-face centre) relative to the collision box centre.</summary>
        public float3 MeshOffset => new float3(0f, -(Brick.BodyHeight * Scale - Margin) * 0.5f, 0f);
        public float BrickMass => Brick.Width * Brick.BodyHeight * Brick.Length * Scale * Scale * Scale * Density;

        public float3 Center(BrickPlacement b) => Origin + new float3(
            (b.X + b.W * 0.5f) * Brick.Pitch * Scale,
            b.Layer * Brick.BodyHeight * Scale + (Brick.BodyHeight * Scale - Margin) * 0.5f,
            (b.Z + b.D * 0.5f) * Brick.Pitch * Scale);

        public quaternion Rotation(BrickPlacement b) => b.Rotated ? quaternion.RotateY(math.PI * 0.5f) : quaternion.identity;

        /// <summary>World position of a stud-grid point at the top of course <paramref name="layer"/> - 1 (the interface plane).</summary>
        public float3 GridPoint(float x, float z, int layer) => Origin + new float3(x * Brick.Pitch * Scale, layer * Brick.BodyHeight * Scale, z * Brick.Pitch * Scale);
    }

    /// <summary>Plans the castle on the stud grid and builds it into an <see cref="ISceneBuilder"/>.</summary>
    public static class BrickCastle
    {
        public const int TowerSize = 10;        // corner towers: 10 x 10 rings with 3-stud walls
        public const int TowerOut = 2;          // studs the towers stand proud of the walls' outer face
        public const int WallDepth = 5;         // English bond: a stretcher row and a header row per course
        public const int GateHouseWidth = 18;   // 6-deep block, 3 stretcher rows / 2 header rows per course
        public const int GateHouseDepth = 6;
        public const int GateWidth = 6;         // closed by corbelling one stud per course from both sides
        public const int GateCourses = 7;
        public const int TurretSize = 5;        // pinwheel of four bricks around a one-stud hole

        enum Axis { X, Z }

        // ------------------------------------------------------------------------------------------------ the plan

        public static BrickLayout Generate(CastlePlan plan)
        {
            int S = plan.Side;
            if ((S - 38) % 12 != 0 || S < 38) throw new ArgumentException($"castle side must be 38 + 12 k studs, not {S}");
            var L = new BrickLayout();
            int T = TowerSize, face = TowerOut;
            int wall0 = T, wall1 = S - T;

            // corner towers
            foreach (var (cx, cz) in new[] { (0, 0), (S - T, 0), (0, S - T), (S - T, S - T) })
            {
                Ring(L, cx, cz, T, 0, plan.TowerCourses, BrickTone.Stone);
                RingMerlons(L, cx, cz, T, plan.TowerCourses, BrickTone.Slate);
            }

            // curtain walls: west, east, back; the front wall has the gatehouse in the middle
            EnglishWall(L, Axis.Z, wall0, wall1, face, +1, plan.WallCourses, BrickTone.Stone);
            EnglishWall(L, Axis.Z, wall0, wall1, S - face - 1, -1, plan.WallCourses, BrickTone.Stone);
            EnglishWall(L, Axis.X, wall0, wall1, S - face - 1, -1, plan.WallCourses, BrickTone.Stone);
            int gh0 = (S - GateHouseWidth) / 2, gh1 = gh0 + GateHouseWidth;
            EnglishWall(L, Axis.X, wall0, gh0, face, +1, plan.WallCourses, BrickTone.Stone);
            EnglishWall(L, Axis.X, gh1, wall1, face, +1, plan.WallCourses, BrickTone.Stone);
            GateHouse(L, gh0, face, plan.GateCourses, plan.TurretCourses, BrickTone.Tan);

            // keep, centred, towards the back
            int K = plan.KeepSize;
            int kx = (S - K) / 2, kz = S - face - WallDepth - 3 - K;
            Ring(L, kx, kz, K, 0, plan.KeepCourses - 2, BrickTone.Stone);
            Ring(L, kx, kz, K, plan.KeepCourses - 2, plan.KeepCourses, BrickTone.Red);
            RingMerlons(L, kx, kz, K, plan.KeepCourses, BrickTone.Red);

            // stairs up the inside of the west wall
            Stairs(L, face + WallDepth, wall0 + 2, plan.WallCourses, BrickTone.Wood);
            return L;
        }

        /// <summary>Centre of the castle at ground level, in stud units.</summary>
        public static float2 Center(CastlePlan plan) => new float2(plan.Side * 0.5f, plan.Side * 0.5f);

        // ------------------------------------------------------------------------------------------------ pieces

        static void Place(BrickLayout L, int layer, Axis axis, int along, int across, int len, BrickTone tone)
        {
            if (axis == Axis.X) L.Add(along, across, layer, len == 3, tone);
            else L.Add(across, along, layer, len == 2, tone);
        }

        /// <summary>Bricks of length <paramref name="len"/> along [a0, a1) at <paramref name="across"/>, as many as fit, from the start or the end.</summary>
        static void Run(BrickLayout L, int layer, Axis axis, int a0, int a1, int across, int len, BrickTone tone, bool fromEnd = false)
        {
            if (fromEnd) for (int a = a1 - len; a >= a0; a -= len) Place(L, layer, axis, a, across, len, tone);
            else for (int a = a0; a + len <= a1; a += len) Place(L, layer, axis, a, across, len, tone);
        }

        /// <summary>Min across-cell of a row that starts <paramref name="offset"/> cells behind the outer face and is <paramref name="depth"/> deep.</summary>
        static int RowMin(int outerAt, int dir, int offset, int depth) => dir > 0 ? outerAt + offset : outerAt - offset - depth + 1;

        /// <summary>Five-stud English bond: even courses have stretchers outside and headers inside, odd courses the reverse.
        /// [a0, a1) must be a multiple of six for the course to tile flush.</summary>
        static void EnglishWall(BrickLayout L, Axis axis, int a0, int a1, int outerAt, int dir, int courses, BrickTone tone)
        {
            for (int layer = 0; layer < courses; layer++)
            {
                bool stretchOut = layer % 2 == 0;
                Run(L, layer, axis, a0, a1, RowMin(outerAt, dir, 0, stretchOut ? 2 : 3), stretchOut ? 3 : 2, tone);
                Run(L, layer, axis, a0, a1, RowMin(outerAt, dir, stretchOut ? 2 : 3, stretchOut ? 3 : 2), stretchOut ? 2 : 3, tone);
            }
            Merlons(L, courses, axis, a0, a1, RowMin(outerAt, dir, 0, 3), 2, 2, BrickTone.Slate);
        }

        /// <summary>Header merlons (len along the wall, 3 deep) with gaps along [a0, a1).</summary>
        static void Merlons(BrickLayout L, int layer, Axis axis, int a0, int a1, int across, int len, int gap, BrickTone tone)
        {
            for (int a = a0; a + len <= a1; a += len + gap) Place(L, layer, axis, a, across, len, tone);
        }

        /// <summary>Hollow tower of size n x n (n even, >= 8) with three-stud walls of headers (two studs along the wall, three
        /// deep); the two course patterns are rotated copies of each other, so the joints of consecutive courses never line up.</summary>
        static void Ring(BrickLayout L, int x0, int z0, int n, int layer0, int layer1, BrickTone tone)
        {
            for (int layer = layer0; layer < layer1; layer++)
            {
                if (layer % 2 == 0)
                {
                    Run(L, layer, Axis.X, x0, x0 + n, z0, 2, tone);                     // full front and back strips
                    Run(L, layer, Axis.X, x0, x0 + n, z0 + n - 3, 2, tone);
                    Run(L, layer, Axis.Z, z0 + 3, z0 + n - 3, x0, 2, tone);             // sides between them
                    Run(L, layer, Axis.Z, z0 + 3, z0 + n - 3, x0 + n - 3, 2, tone);
                }
                else
                {
                    Run(L, layer, Axis.Z, z0, z0 + n, x0, 2, tone);                     // full sides
                    Run(L, layer, Axis.Z, z0, z0 + n, x0 + n - 3, 2, tone);
                    Run(L, layer, Axis.X, x0 + 3, x0 + n - 3, z0, 2, tone);             // front and back between them
                    Run(L, layer, Axis.X, x0 + 3, x0 + n - 3, z0 + n - 3, 2, tone);
                }
            }
        }

        static void RingMerlons(BrickLayout L, int x0, int z0, int n, int layer, BrickTone tone)
        {
            Merlons(L, layer, Axis.X, x0, x0 + n, z0, 2, 2, tone);
            Merlons(L, layer, Axis.X, x0, x0 + n, z0 + n - 3, 2, 2, tone);
            Merlons(L, layer, Axis.Z, z0 + 4, z0 + n - 4, x0, 2, 2, tone);
            Merlons(L, layer, Axis.Z, z0 + 4, z0 + n - 4, x0 + n - 3, 2, 2, tone);
        }

        /// <summary>Solid 5 x 5 turret: a pinwheel of four bricks around a one-stud hole, mirrored on alternate courses.</summary>
        static void Turret(BrickLayout L, int x0, int z0, int layer0, int layer1, BrickTone tone)
        {
            for (int layer = layer0; layer < layer1; layer++)
            {
                if (layer % 2 == 0)
                {
                    L.Add(x0, z0, layer, true, tone); L.Add(x0 + 3, z0, layer, false, tone);
                    L.Add(x0 + 2, z0 + 3, layer, true, tone); L.Add(x0, z0 + 2, layer, false, tone);
                }
                else
                {
                    L.Add(x0, z0, layer, false, tone); L.Add(x0 + 2, z0, layer, true, tone);
                    L.Add(x0 + 3, z0 + 2, layer, false, tone); L.Add(x0, z0 + 3, layer, true, tone);
                }
            }
            L.Add(x0, z0, layer1, false, BrickTone.Slate);
            L.Add(x0 + 3, z0 + 2, layer1, false, BrickTone.Slate);
        }

        /// <summary>Six-deep gatehouse on the front wall: stretcher courses (three rows) alternate with header courses (two rows).
        /// The passage is closed by three corbel courses of stretchers, then turrets and merlons on top.</summary>
        static void GateHouse(BrickLayout L, int x0, int z0, int courses, int turretCourses, BrickTone tone)
        {
            int x1 = x0 + GateHouseWidth;
            int g0 = x0 + (GateHouseWidth - GateWidth) / 2, g1 = g0 + GateWidth;
            int closed = GateCourses + GateWidth / 2;
            for (int layer = 0; layer < courses; layer++)
            {
                bool headers = layer < GateCourses ? layer % 2 == 1 : layer >= closed && (layer - closed) % 2 == 0;
                if (headers)
                {
                    foreach (int row in new[] { 0, 3 })
                    {
                        if (layer < GateCourses) { Run(L, layer, Axis.X, x0, g0, z0 + row, 2, tone); Run(L, layer, Axis.X, g1, x1, z0 + row, 2, tone); }
                        else Run(L, layer, Axis.X, x0, x1, z0 + row, 2, tone);
                    }
                }
                else if (layer < GateCourses)
                {
                    foreach (int row in new[] { 0, 2, 4 }) { Run(L, layer, Axis.X, x0, g0, z0 + row, 3, tone); Run(L, layer, Axis.X, g1, x1, z0 + row, 3, tone); }
                }
                else if (layer < closed)
                {
                    // corbel course k: the stretchers next to the passage overhang it by k studs (two thirds supported); the
                    // k mod 3 studs left over at the far ends become one header (2 studs) or two headers (4 = 1 mod 3) so every course is full
                    int k = layer - GateCourses + 1;
                    int lead = k % 3 == 1 ? 4 : k % 3 == 2 ? 2 : 0;
                    foreach (int row in new[] { 0, 2, 4 }) { Run(L, layer, Axis.X, x0 + lead, g0 + k, z0 + row, 3, tone, fromEnd: true); Run(L, layer, Axis.X, g1 - k, x1 - lead, z0 + row, 3, tone); }
                    foreach (int row in new[] { 0, 3 }) { Run(L, layer, Axis.X, x0, x0 + lead, z0 + row, 2, tone); Run(L, layer, Axis.X, x1 - lead, x1, z0 + row, 2, tone); }
                }
                else
                {
                    foreach (int row in new[] { 0, 2, 4 }) Run(L, layer, Axis.X, x0, x1, z0 + row, 3, tone);
                }
            }
            Turret(L, x0, z0, courses, courses + turretCourses, tone);
            Turret(L, x1 - TurretSize, z0, courses, courses + turretCourses, tone);
            Merlons(L, courses, Axis.X, x0 + TurretSize + 1, x1 - TurretSize - 1, z0, 2, 2, BrickTone.Slate);
        }

        /// <summary>Staircase of stacked bricks along a wall running in z: step k is a column of k + 1 bricks.</summary>
        static void Stairs(BrickLayout L, int x0, int z0, int steps, BrickTone tone)
        {
            for (int k = 0; k < steps; k++)
                for (int layer = 0; layer <= k; layer++)
                    L.Add(x0, z0 + 2 * k, layer, true, tone);
        }

        // ------------------------------------------------------------------------------------------------ bodies

        /// <summary>Adds every brick of the layout as a box; returns the index of the first brick body (bricks are consecutive).</summary>
        public static int Build(ISceneBuilder s, BrickLayout layout, BrickSpec spec)
        {
            int first = -1;
            float3 size = spec.BoxSize;
            foreach (var b in layout.Bricks)
            {
                int body = s.AddBody(size, spec.Density, spec.Friction, spec.Center(b), spec.Rotation(b), float3.zero);
                if (first < 0) first = body;
            }
            return first;
        }

        /// <summary>Snaps the bricks together: hard ball-socket joints (breaking at <paramref name="fracture"/> newtons each) between
        /// every brick and the bricks it rests on, and between the ground course and the world. Jointed bodies do not collide, so
        /// four joints at the inset corners of every overlap hold the pair rigid (the solver's angular lock assumes equal
        /// orientations, which the two brick orientations do not have).</summary>
        public static int AddSnapJoints(ISceneBuilder s, BrickLayout layout, int firstBody, BrickSpec spec, float fracture)
        {
            int joints = 0;
            for (int i = 0; i < layout.Bricks.Count; i++)
            {
                var b = layout.Bricks[i];
                int bodyB = firstBody + i;
                float3 posB = spec.Center(b);
                quaternion invB = math.conjugate(spec.Rotation(b));
                if (b.Layer == 0)
                {
                    foreach (var p in InsetCorners(b.X, b.X + b.W, b.Z, b.Z + b.D))
                    {
                        float3 anchor = spec.GridPoint(p.x, p.y, 0);
                        s.AddJoint(-1, bodyB, anchor, math.mul(invB, anchor - posB), float.PositiveInfinity, 0f, fracture);
                        joints++;
                    }
                    continue;
                }
                foreach (var (below, cells) in layout.Supporters(i))
                {
                    var a = layout.Bricks[below];
                    int bodyA = firstBody + below;
                    float3 posA = spec.Center(a);
                    quaternion invA = math.conjugate(spec.Rotation(a));
                    int xMin = int.MaxValue, xMax = int.MinValue, zMin = int.MaxValue, zMax = int.MinValue;
                    foreach (var (x, z) in cells) { xMin = math.min(xMin, x); xMax = math.max(xMax, x + 1); zMin = math.min(zMin, z); zMax = math.max(zMax, z + 1); }
                    foreach (var p in InsetCorners(xMin, xMax, zMin, zMax))
                    {
                        float3 anchor = spec.GridPoint(p.x, p.y, b.Layer);
                        s.AddJoint(bodyA, bodyB, math.mul(invA, anchor - posA), math.mul(invB, anchor - posB), float.PositiveInfinity, 0f, fracture);
                        joints++;
                    }
                }
            }
            return joints;
        }

        /// <summary>The four corners of the rectangle [x0, x1) x [z0, z1) moved a quarter of the way to its centre.</summary>
        static float2[] InsetCorners(int x0, int x1, int z0, int z1)
        {
            float dx = (x1 - x0) * 0.25f, dz = (z1 - z0) * 0.25f;
            return new[] { new float2(x0 + dx, z0 + dz), new float2(x1 - dx, z0 + dz), new float2(x0 + dx, z1 - dz), new float2(x1 - dx, z1 - dz) };
        }
    }
}
