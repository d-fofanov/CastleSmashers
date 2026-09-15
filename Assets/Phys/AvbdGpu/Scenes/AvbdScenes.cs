// Scenes ported from scenes.h of avbd-demo3d (https://github.com/savant117/avbd-demo3d, Copyright (c) 2026 Chris Giles),
// transcribed from the reference's Z-up frame to Unity's Y-up frame (reference (x, y, z) -> Unity (x, z, y)).

using Unity.Mathematics;

namespace Phys.AvbdGpu.Scenes
{
    /// <summary>Backend-agnostic scene construction: the CPU reference and the GPU world both implement it.</summary>
    public interface ISceneBuilder
    {
        /// <summary>Adds a box; density 0 makes it static. Returns the body index (sequential from 0).</summary>
        int AddBody(float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity);

        /// <summary>Ball-socket joint (+ angular lock when stiffnessAng > 0); bodyA = -1 anchors rA in world space.</summary>
        void AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture);

        /// <summary>Joint with snap fracture: it also breaks when the linear multiplier pulls B away from A along the snap axis by
        /// more than <paramref name="fractureTension"/>, when its part across the axis exceeds <paramref name="fractureLateral"/>, or
        /// when the anchors are <paramref name="breakDistance"/> apart. The axis is +-1 x, +-2 y, +-3 z of body B's frame.</summary>
        void AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture,
            float fractureLateral, float fractureTension, float breakDistance, int snapAxis);

        /// <summary>Distance spring; rest &lt; 0 measures the rest length from the initial pose.</summary>
        void AddSpring(int bodyA, int bodyB, float3 rA, float3 rB, float stiffness, float rest);

        void AddIgnoreCollision(int bodyA, int bodyB);

        /// <summary>The terrain every dynamic body collides with (one per world; replaces an earlier one). It occupies a body index
        /// (a static body at the identity pose) so that manifolds against it work like manifolds against a static box.</summary>
        int SetTerrain(Heightfield field, float friction);
    }

    public static class SceneBuilderExtensions
    {
        public static int AddBody(this ISceneBuilder b, float3 size, float density, float friction, float3 position)
            => b.AddBody(size, density, friction, position, quaternion.identity, float3.zero);

        public static int AddBody(this ISceneBuilder b, float3 size, float density, float friction, float3 position, float3 velocity)
            => b.AddBody(size, density, friction, position, quaternion.identity, velocity);

        public static void AddJoint(this ISceneBuilder b, int bodyA, int bodyB, float3 rA, float3 rB)
            => b.AddJoint(bodyA, bodyB, rA, rB, float.PositiveInfinity, 0f, float.PositiveInfinity);

        public static void AddJoint(this ISceneBuilder b, int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng)
            => b.AddJoint(bodyA, bodyB, rA, rB, stiffnessLin, stiffnessAng, float.PositiveInfinity);
    }

    public struct SceneInfo
    {
        public string Name;
        public string Description;
        public float3 CameraTarget;
        public float CameraDistance;
        /// <summary>Approximate body count, used to size GPU buffers.</summary>
        public int Bodies;
    }

    /// <summary>The 14 reference scenes plus GPU benchmark scenes. Indices are stable (used by the demo keys and the tests).</summary>
    public static class AvbdScenes
    {
        public const int Empty = 0, Ground = 1, DynamicFriction = 2, StaticFriction = 3, Pyramid = 4, Rope = 5, HeavyRope = 6, Spring = 7,
            SpringsRatio = 8, Stack = 9, StackRatio = 10, SoftBody = 11, Bridge = 12, Breakable = 13, PyramidLarge = 14, Pile = 15, PyramidHuge = 16,
            Terrain = 17;

        public static readonly SceneInfo[] All =
        {
            new SceneInfo { Name = "Empty", Description = "Nothing. Shoot boxes with the middle mouse button / Space.", CameraTarget = new float3(0, 5, 0), CameraDistance = 40, Bodies = 0 },
            new SceneInfo { Name = "Ground", Description = "A box dropped on the ground.", CameraTarget = new float3(0, 2, 0), CameraDistance = 12, Bodies = 2 },
            new SceneInfo { Name = "Dynamic friction", Description = "Eleven boxes launched at 10 m/s with friction 5 .. 0: stopping distance grows as friction falls.", CameraTarget = new float3(0, 1, -20), CameraDistance = 45, Bodies = 12 },
            new SceneInfo { Name = "Static friction", Description = "Cubes with friction 0.25 .. 0.5 on a 30 degree ramp of friction 1: combined sqrt(mu) spans 0.5 .. 0.71 around tan 30 = 0.58, so the low ones slide and the high ones stick.", CameraTarget = new float3(0, 6, 0), CameraDistance = 45, Bodies = 13 },
            new SceneInfo { Name = "Pyramid", Description = "16-row pyramid of 1 x 0.5 x 0.5 boxes held by frictional contacts (136 boxes).", CameraTarget = new float3(0, 6, 0), CameraDistance = 30, Bodies = 137 },
            new SceneInfo { Name = "Rope", Description = "20 links, hard ball-socket joints, first link static.", CameraTarget = new float3(10, 5, 0), CameraDistance = 30, Bodies = 21 },
            new SceneInfo { Name = "Heavy rope", Description = "Rope with a 5 m cube (1000 : 1 mass ratio) at the end.", CameraTarget = new float3(12, 4, 0), CameraDistance = 40, Bodies = 21 },
            new SceneInfo { Name = "Spring", Description = "A 2 m block hanging from a spring (k = 100 N/m, rest 4 m).", CameraTarget = new float3(0, 9, 0), CameraDistance = 25, Bodies = 3 },
            new SceneInfo { Name = "Springs ratio", Description = "Chain of springs alternating k = 10 and k = 10 000 (ratio 1000) between two static ends.", CameraTarget = new float3(0, 9, 0), CameraDistance = 40, Bodies = 9 },
            new SceneInfo { Name = "Stack", Description = "Ten unit cubes stacked with 0.5 m gaps.", CameraTarget = new float3(0, 7, 0), CameraDistance = 25, Bodies = 11 },
            new SceneInfo { Name = "Stack ratio", Description = "Cubes of size 1, 2, 4, 8 stacked smallest at the bottom (512 : 1 mass ratio).", CameraTarget = new float3(0, 7, 0), CameraDistance = 35, Bodies = 5 },
            new SceneInfo { Name = "Soft body", Description = "Three 4^3 lattices of cubes joined by soft joints (k = 1000 / 250) dropped on each other.", CameraTarget = new float3(0, 10, 0), CameraDistance = 40, Bodies = 193 },
            new SceneInfo { Name = "Bridge", Description = "40 planks on two hard hinges each, 50 boxes dropped on it.", CameraTarget = new float3(0, 9, 0), CameraDistance = 55, Bodies = 91 },
            new SceneInfo { Name = "Breakable", Description = "Chain of breakable joints (90 N) between two pillars, boxes dropped on it.", CameraTarget = new float3(0, 5, 0), CameraDistance = 30, Bodies = 19 },
            new SceneInfo { Name = "Pyramid 22k", Description = "GPU benchmark: square pyramid, base 40 x 40, 22 140 cubes.", CameraTarget = new float3(0, 12, 0), CameraDistance = 90, Bodies = 22141 },
            new SceneInfo { Name = "Pile 50k", Description = "GPU benchmark: 50 x 20 x 50 cubes dropped as a block.", CameraTarget = new float3(0, 12, 0), CameraDistance = 110, Bodies = 50001 },
            new SceneInfo { Name = "Pyramid 74k", Description = "GPU benchmark: square pyramid, base 60 x 60, 73 810 cubes.", CameraTarget = new float3(0, 18, 0), CameraDistance = 130, Bodies = 73811 },
            new SceneInfo { Name = "Terrain", Description = "A hundred boxes, some tilted, dropped on 128 x 128 m of hills (a heightfield, cell 1 m).", CameraTarget = new float3(0, 4, 0), CameraDistance = 45, Bodies = 101 },
        };

        public static int Count => All.Length;

        /// <summary>Ground box of the reference scenes: 100 x 100 x 1 centred at the given height.</summary>
        static int GroundBox(ISceneBuilder s, float y = 0f, float friction = 0.5f) => s.AddBody(new float3(100, 1, 100), 0f, friction, new float3(0, y, 0));

        public static void Build(ISceneBuilder s, int index)
        {
            switch (index)
            {
                case Empty: break;
                case Ground: SceneGround(s); break;
                case DynamicFriction: SceneDynamicFriction(s); break;
                case StaticFriction: SceneStaticFriction(s); break;
                case Pyramid: ScenePyramid(s); break;
                case Rope: SceneRope(s); break;
                case HeavyRope: SceneHeavyRope(s); break;
                case Spring: SceneSpring(s); break;
                case SpringsRatio: SceneSpringsRatio(s); break;
                case Stack: SceneStack(s); break;
                case StackRatio: SceneStackRatio(s); break;
                case SoftBody: SceneSoftBody(s); break;
                case Bridge: SceneBridge(s); break;
                case Breakable: SceneBreakable(s); break;
                case PyramidLarge: ScenePyramid3D(s, 40); break;
                case Pile: ScenePile(s, 50, 20, 50); break;
                case PyramidHuge: ScenePyramid3D(s, 60); break;
                case Terrain: SceneTerrain(s); break;
            }
        }

        public static void SceneGround(ISceneBuilder s)
        {
            GroundBox(s);
            s.AddBody(new float3(1, 1, 1), 1.0f, 0.5f, new float3(0, 4, 0));
        }

        public static void SceneDynamicFriction(ISceneBuilder s)
        {
            GroundBox(s);
            for (int x = 0; x <= 10; x++)
                s.AddBody(new float3(1, 0.5f, 1), 1.0f, 5.0f - (x / 10.0f * 5.0f), new float3(0, 0.75f, -30.0f + x * 2.0f), new float3(10.0f, 0, 0));
        }

        public static void SceneStaticFriction(ISceneBuilder s)
        {
            GroundBox(s, 0f, 0.5f);

            // Reference: rotation about its Y (depth) axis; in Y-up that is a rotation about Z tilting +x downwards.
            float angle = math.radians(30.0f);
            quaternion rampRot = new quaternion(0, 0, math.sin(-angle * 0.5f), math.cos(-angle * 0.5f));
            int ramp = s.AddBody(new float3(40, 1, 24), 0.0f, 1.0f, new float3(0, 3, 0), rampRot, float3.zero);
            float3 rampPos = new float3(0, 3, 0);

            float3 rampTangent = math.normalize(math.mul(rampRot, new float3(1, 0, 0)));
            float3 rampNormal = math.normalize(math.mul(rampRot, new float3(0, 1, 0)));

            for (int i = 0; i <= 10; i++)
            {
                float friction = i / 10.0f * 0.25f + 0.25f;
                float d = -10.0f + i * 2.0f;
                float3 pos = rampPos + rampTangent * -12.0f + new float3(0, 0, d) + rampNormal * 1.05f;
                s.AddBody(new float3(1, 1, 1), 1.0f, friction, pos);
            }
            _ = ramp;
        }

        public static void ScenePyramid(ISceneBuilder s, int size = 16)
        {
            s.AddBody(new float3(100, 1, 100), 0.0f, 0.5f, new float3(0.0f, -0.5f, 0.0f));
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size - y; x++)
                    s.AddBody(new float3(1, 0.5f, 0.5f), 1.0f, 0.5f, new float3(x * 1.01f + y * 0.5f - size / 2.0f, y * 0.85f + 0.5f, 0.0f));
        }

        public static void SceneRope(ISceneBuilder s, int n = 20)
        {
            GroundBox(s, -20f);
            int prev = -1;
            for (int i = 0; i < n; i++)
            {
                int curr = s.AddBody(new float3(1, 0.5f, 0.5f), i == 0 ? 0.0f : 1.0f, 0.5f, new float3(i, 10.0f, 0.0f));
                if (prev >= 0)
                    s.AddJoint(prev, curr, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0));
                prev = curr;
            }
        }

        public static void SceneHeavyRope(ISceneBuilder s)
        {
            const int N = 20;
            const float SIZE = 5;
            GroundBox(s, -20f);
            int prev = -1;
            for (int i = 0; i < N; i++)
            {
                int curr = s.AddBody(i == N - 1 ? new float3(SIZE, SIZE, SIZE) : new float3(1, 0.5f, 0.5f),
                    i == 0 ? 0.0f : 1.0f, 0.5f, new float3(i + (i == N - 1 ? SIZE / 2 : 0), 10.0f, 0.0f));
                if (prev >= 0)
                    s.AddJoint(prev, curr, new float3(0.5f, 0, 0), i == N - 1 ? new float3(-SIZE / 2, 0, 0) : new float3(-0.5f, 0, 0));
                prev = curr;
            }
        }

        public static void SceneSpring(ISceneBuilder s)
        {
            GroundBox(s);
            int anchor = s.AddBody(new float3(1, 1, 1), 0.0f, 0.5f, new float3(0, 14.0f, 0));
            int block = s.AddBody(new float3(2, 2, 2), 1.0f, 0.5f, new float3(0, 8.0f, 0));
            s.AddSpring(anchor, block, float3.zero, float3.zero, 100.0f, 4.0f);
        }

        public static void SceneSpringsRatio(ISceneBuilder s)
        {
            const int N = 8;
            GroundBox(s, -10f);
            int prev = -1;
            for (int i = 0; i < N; i++)
            {
                float x = (i - (N - 1) * 0.5f) * 3.0f;
                int curr = s.AddBody(new float3(1, 0.75f, 0.75f), i == 0 || i == N - 1 ? 0.0f : 1.0f, 0.5f, new float3(x, 12.0f, 0.0f));
                if (prev >= 0)
                    s.AddSpring(prev, curr, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0), i % 2 == 0 ? 10.0f : 10000.0f, 3.0f);
                prev = curr;
            }
        }

        public static void SceneStack(ISceneBuilder s, int n = 10)
        {
            GroundBox(s);
            for (int i = 0; i < n; i++)
                s.AddBody(new float3(1, 1, 1), 1.0f, 0.5f, new float3(0, i * 1.5f + 1.0f, 0));
        }

        public static void SceneStackRatio(ISceneBuilder s)
        {
            const float groundThickness = 1.0f;
            s.AddBody(new float3(100, groundThickness, 100), 0.0f, 0.5f, float3.zero);

            float top = groundThickness * 0.5f;
            float size = 1.0f;
            for (int i = 0; i < 4; i++)
            {
                float half = size * 0.5f;
                float centerY = top + half;
                s.AddBody(new float3(size, size, size), 1.0f, 0.5f, new float3(0, centerY, 0));
                top = centerY + half;
                size *= 2.0f;
            }
        }

        public static void SceneSoftBody(ISceneBuilder s)
        {
            GroundBox(s);

            const float Klin = 1000.0f;
            const float Kang = 250.0f;
            const int W = 4, D = 4, H = 4, N = 3;
            const float size = 0.8f;
            const float half = size * 0.5f;
            const float baseY = 8.0f;
            const float stackGap = 2.0f;

            for (int i = 0; i < N; i++)
            {
                var grid = new int[W, D, H];
                float stackY = i * (H * size + stackGap);

                for (int x = 0; x < W; x++)
                    for (int y = 0; y < D; y++)
                        for (int z = 0; z < H; z++)
                        {
                            float px = (x - (W - 1) * 0.5f) * size;
                            float pz = (y - (D - 1) * 0.5f) * size;  // reference y (depth) -> Unity z
                            float py = baseY + stackY + z * size;    // reference z (up) -> Unity y
                            grid[x, y, z] = s.AddBody(new float3(size, size, size), 1.0f, 0.5f, new float3(px, py, pz));
                        }

                for (int x = 1; x < W; x++)
                    for (int y = 0; y < D; y++)
                        for (int z = 0; z < H; z++)
                            s.AddJoint(grid[x - 1, y, z], grid[x, y, z], new float3(half, 0, 0), new float3(-half, 0, 0), Klin, Kang);

                for (int x = 0; x < W; x++)
                    for (int y = 1; y < D; y++)
                        for (int z = 0; z < H; z++)
                            s.AddJoint(grid[x, y - 1, z], grid[x, y, z], new float3(0, 0, half), new float3(0, 0, -half), Klin, Kang);

                for (int x = 0; x < W; x++)
                    for (int y = 0; y < D; y++)
                        for (int z = 1; z < H; z++)
                            s.AddJoint(grid[x, y, z - 1], grid[x, y, z], new float3(0, half, 0), new float3(0, -half, 0), Klin, Kang);

                for (int x = 1; x < W; x++)
                    for (int y = 0; y < D; y++)
                        for (int z = 1; z < H; z++)
                        {
                            s.AddIgnoreCollision(grid[x - 1, y, z - 1], grid[x, y, z]);
                            s.AddIgnoreCollision(grid[x, y, z - 1], grid[x - 1, y, z]);
                        }

                for (int x = 0; x < W; x++)
                    for (int y = 1; y < D; y++)
                        for (int z = 1; z < H; z++)
                        {
                            s.AddIgnoreCollision(grid[x, y - 1, z - 1], grid[x, y, z]);
                            s.AddIgnoreCollision(grid[x, y, z - 1], grid[x, y - 1, z]);
                        }

                for (int x = 1; x < W; x++)
                    for (int y = 1; y < D; y++)
                        for (int z = 0; z < H; z++)
                        {
                            s.AddIgnoreCollision(grid[x - 1, y - 1, z], grid[x, y, z]);
                            s.AddIgnoreCollision(grid[x, y - 1, z], grid[x - 1, y, z]);
                        }
            }
        }

        public static void SceneBridge(ISceneBuilder s, bool withBoxes = true)
        {
            const int N = 40;
            const float plankLength = 1.0f;
            const float plankWidth = 4.0f;
            const float plankHeight = 0.5f;
            const float halfLength = plankLength * 0.5f;
            const float halfWidth = plankWidth * 0.5f;

            GroundBox(s);

            int prev = -1;
            for (int i = 0; i < N; i++)
            {
                int curr = s.AddBody(new float3(plankLength, plankHeight, plankWidth), i == 0 || i == N - 1 ? 0.0f : 1.0f, 0.5f, new float3(i - N / 2.0f, 10.0f, 0.0f));
                if (prev >= 0)
                {
                    s.AddJoint(prev, curr, new float3(halfLength, 0, halfWidth), new float3(-halfLength, 0, halfWidth), float.PositiveInfinity, 0.0f);
                    s.AddJoint(prev, curr, new float3(halfLength, 0, -halfWidth), new float3(-halfLength, 0, -halfWidth), float.PositiveInfinity, 0.0f);
                }
                prev = curr;
            }

            if (!withBoxes) return;
            for (int x = 0; x < N / 4; x++)
                for (int y = 0; y < N / 8; y++)
                    s.AddBody(new float3(1, 1, 1), 1.0f, 0.5f, new float3(x - N / 8.0f, y + 12.0f, 0.0f));
        }

        public static void SceneBreakable(ISceneBuilder s)
        {
            const int N = 10;
            const int M = 5;
            const float breakForce = 90.0f;

            GroundBox(s);

            int prev = -1;
            for (int i = 0; i <= N; i++)
            {
                int curr = s.AddBody(new float3(1, 0.5f, 1), 1.0f, 0.5f, new float3(i - N / 2.0f, 6.0f, 0.0f));
                if (prev >= 0)
                    s.AddJoint(prev, curr, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0), float.PositiveInfinity, float.PositiveInfinity, breakForce);
                prev = curr;
            }

            s.AddBody(new float3(1, 5, 1), 0.0f, 0.5f, new float3(-N / 2.0f, 2.5f, 0));
            s.AddBody(new float3(1, 5, 1), 0.0f, 0.5f, new float3(N / 2.0f, 2.5f, 0));

            for (int i = 0; i < M; i++)
                s.AddBody(new float3(2, 1, 1), 1.0f, 0.5f, new float3(0, i * 2.0f + 8.0f, 0));
        }

        /// <summary>Square pyramid of unit cubes: layer k (from the top) is k x k, base n x n; sum k^2 cubes.</summary>
        public static void ScenePyramid3D(ISceneBuilder s, int n)
        {
            s.AddBody(new float3(n * 4f, 1, n * 4f), 0.0f, 0.5f, new float3(0, -0.5f, 0));
            const float pitch = 1.02f;
            for (int layer = 0; layer < n; layer++)
            {
                int k = n - layer;
                float y = layer + 0.5f;
                float offset = -(k - 1) * 0.5f * pitch;
                for (int i = 0; i < k; i++)
                    for (int j = 0; j < k; j++)
                        s.AddBody(new float3(1, 1, 1), 1.0f, 0.5f, new float3(offset + i * pitch, y, offset + j * pitch));
            }
        }

        /// <summary>Boxes of three sizes dropped from 6 m onto rolling hills, every third one tilted, so that corners, edges and
        /// faces meet the slopes; the terrain is the same on both solvers (<see cref="Heightfield.Generate"/> is deterministic).</summary>
        public static void SceneTerrain(ISceneBuilder s)
        {
            var field = TerrainField();
            s.SetTerrain(field, 0.6f);
            var rng = new Unity.Mathematics.Random(2024u);
            for (int x = 0; x < 10; x++)
                for (int z = 0; z < 10; z++)
                {
                    int i = x * 10 + z;
                    float3 size = i % 3 == 0 ? new float3(2, 0.5f, 1) : i % 3 == 1 ? new float3(1, 1, 1) : new float3(0.6f, 0.6f, 1.4f);
                    float2 xz = new float2(-13.5f + x * 3f, -13.5f + z * 3f);
                    float3 pos = new float3(xz.x, field.Height(xz) + 6f + (i % 4) * 0.7f, xz.y);
                    quaternion rot = i % 3 == 2 ? quaternion.Euler(rng.NextFloat3(-0.6f, 0.6f)) : quaternion.RotateY(rng.NextFloat(0f, math.PI));
                    s.AddBody(size, 1.0f, 0.5f, pos, rot, float3.zero);
                }
        }

        /// <summary>The Terrain scene's heightfield: 129 x 129 samples 1 m apart centred on the origin, hills of about 4 m.</summary>
        public static Heightfield TerrainField() => Heightfield.Generate(TerrainPreset.Hills, 7u, 129, 1f, 4f, 24f);

        /// <summary>w x h x d block of unit cubes with 0.3 m gaps dropped from 2 m, slightly jittered.</summary>
        public static void ScenePile(ISceneBuilder s, int w, int h, int d)
        {
            s.AddBody(new float3(math.max(w, d) * 4f, 1, math.max(w, d) * 4f), 0.0f, 0.5f, new float3(0, -0.5f, 0));
            const float pitch = 1.3f;
            var rng = new Unity.Mathematics.Random(12345u);
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                    for (int z = 0; z < d; z++)
                    {
                        float3 jitter = rng.NextFloat3(new float3(-0.05f), new float3(0.05f));
                        s.AddBody(new float3(1, 1, 1), 1.0f, 0.5f, new float3((x - (w - 1) * 0.5f) * pitch, 2f + y * pitch, (z - (d - 1) * 0.5f) * pitch) + jitter);
                    }
        }
    }
}
