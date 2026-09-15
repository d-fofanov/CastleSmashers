using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu
{
    /// <summary>Records the whole simulation step into one CommandBuffer. Every count-dependent dispatch is indirect, so the
    /// buffer only has to be re-recorded when the iteration count, the post-stabilisation flag or the active colour count change.</summary>
    public sealed class AvbdGpuPipeline
    {
        readonly AvbdGpuKernels m_K;
        readonly AvbdGpuBuffers m_B;
        readonly CommandBuffer m_Cb = new CommandBuffer { name = "AVBD step" };

        int m_Iterations = -1, m_ActiveColors = -1, m_ColorRounds = -1, m_TerrainVersion = -1;
        bool m_PostStabilize, m_Terrain;

        public CommandBuffer CommandBuffer => m_Cb;

        static readonly int s_Params = Shader.PropertyToID("AvbdParams");
        static readonly int s_Phase = Shader.PropertyToID("_Phase");
        static readonly int s_ScanN = Shader.PropertyToID("_ScanN");
        static readonly int s_ColorSlot = Shader.PropertyToID("_ColorSlot");
        static readonly int s_WriteToNew = Shader.PropertyToID("_WriteToNew");
        static readonly int s_AlphaIt = Shader.PropertyToID("_AlphaIt");
        static readonly int s_ColorIn = Shader.PropertyToID("_ColorIn");
        static readonly int s_ColorOut = Shader.PropertyToID("_ColorOut");
        static readonly int s_ScanIn = Shader.PropertyToID("_ScanIn");
        static readonly int s_ScanOut = Shader.PropertyToID("_ScanOut");

        const int ArgBodies = 0, ArgPairs = 1, ArgManifolds = 2, ArgConstraints = 3, ArgJoints = 4, ArgColor0 = 8;
        const int Threads = AvbdGpuConstants.ThreadGroupSize;
        const int ScanBlock = 1024;

        public AvbdGpuPipeline(AvbdGpuKernels kernels, AvbdGpuBuffers buffers)
        {
            m_K = kernels;
            m_B = buffers;
        }

        public void Dispose() => m_Cb.Release();

        static int Groups(int n, int per = Threads) => math.max(1, (n + per - 1) / per);

        /// <summary>Re-records the step when the configuration changed (<paramref name="terrain"/>: the world has a terrain, so the
        /// CollideTerrain pass is recorded and bound to the current terrain buffers).</summary>
        public void Ensure(int iterations, int activeColors, bool postStabilize, int colorRounds, float alpha, bool terrain)
        {
            int terrainVersion = terrain ? m_B.TerrainVersion : -1;
            if (iterations == m_Iterations && activeColors == m_ActiveColors && postStabilize == m_PostStabilize && colorRounds == m_ColorRounds && alpha == m_Alpha
                && terrain == m_Terrain && terrainVersion == m_TerrainVersion) return;
            m_Iterations = iterations; m_ActiveColors = activeColors; m_PostStabilize = postStabilize; m_ColorRounds = colorRounds; m_Alpha = alpha;
            m_Terrain = terrain; m_TerrainVersion = terrainVersion;
            Record();
        }

        float m_Alpha = -1f;

        void Bind(ComputeShader cs, int kernel, string name, GraphicsBuffer buffer) => m_Cb.SetComputeBufferParam(cs, kernel, name, buffer);

        void BindAll(ComputeShader cs, int kernel, params (string, GraphicsBuffer)[] bindings)
        {
            foreach (var (name, buffer) in bindings) Bind(cs, kernel, name, buffer);
        }

        void Indirect(ComputeShader cs, int kernel, int argSlot) => m_Cb.DispatchCompute(cs, kernel, m_B.DispatchArgs, (uint)(argSlot * 12));

        void Direct(ComputeShader cs, int kernel, int groups) => m_Cb.DispatchCompute(cs, kernel, groups, 1, 1);

        void RecordScan(GraphicsBuffer src, GraphicsBuffer dst, int n)
        {
            var cs = m_K.Scan;
            foreach (int k in new[] { m_K.ScanBlock, m_K.ScanTop, m_K.ScanAdd })
            {
                m_Cb.SetComputeBufferParam(cs, k, s_ScanIn, src);
                m_Cb.SetComputeBufferParam(cs, k, s_ScanOut, dst);
                Bind(cs, k, "_BlockSums", m_B.BlockSums);
            }
            m_Cb.SetComputeIntParam(cs, s_ScanN, n);
            Direct(cs, m_K.ScanBlock, Groups(n, ScanBlock));
            Direct(cs, m_K.ScanTop, 1);
            Direct(cs, m_K.ScanAdd, Groups(n, ScanBlock));
        }

        void Record()
        {
            var cb = m_Cb;
            var b = m_B;
            var k = m_K;
            var cfg = b.Config;
            cb.Clear();

            foreach (var cs in new[] { k.Util, k.Scan, k.Broadphase, k.Narrowphase, k.Constraints, k.Coloring, k.Solver })
                cb.SetComputeConstantBufferParam(cs, s_Params, b.Params, 0, GpuParams.Stride);

            // ---------------------------------------------------------------- util bindings
            foreach (int kk in new[] { k.BuildArgs, k.HashClear, k.CopyStats })
                BindAll(k.Util, kk, ("_Counters", b.Counters), ("_Stats", b.Stats), ("_DispatchArgs", b.DispatchArgs), ("_HashCur", b.HashCur));

            // ---------------------------------------------------------------- broadphase bindings
            foreach (int kk in new[] { k.BodyAabb, k.GridClear, k.GridCount, k.GridScatter, k.GridSortCell, k.LargeSort, k.PairGen })
                BindAll(k.Broadphase, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot),
                    ("_BodyAabbMin", b.BodyAabbMin), ("_BodyAabbMax", b.BodyAabbMax),
                    ("_CellCount", b.CellCount), ("_CellStart", b.CellStart), ("_CellCursor", b.CellCursor), ("_CellEntries", b.CellEntries),
                    ("_LargeBodies", b.LargeBodies), ("_Counters", b.Counters), ("_Pairs", b.Pairs),
                    ("_LinkStart", b.LinkStart), ("_LinkList", b.LinkList), ("_JointState", b.JointState));

            // ---------------------------------------------------------------- narrowphase bindings
            foreach (int kk in new[] { k.Collide, k.CollideTerrain })
                BindAll(k.Narrowphase, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot), ("_BodyVelLin", b.BodyVelLin), ("_Pairs", b.Pairs), ("_Counters", b.Counters),
                    ("_ManifoldPrev", b.ManifoldPrev), ("_ContactsPrev", b.ContactsPrev), ("_HashPrev", b.HashPrev),
                    ("_ManifoldCur", b.ManifoldCur), ("_ContactsCur", b.ContactsCur), ("_HashCur", b.HashCur), ("_BodyEvents", b.BodyEvents),
                    ("_BodyAabbMin", b.BodyAabbMin), ("_BodyAabbMax", b.BodyAabbMax), ("_TerrainHeights", b.TerrainHeights), ("_TerrainMaxMip", b.TerrainMaxMip));

            // ---------------------------------------------------------------- constraint bindings
            foreach (int kk in new[] { k.PrepareJoints, k.ConsClear, k.ConsCount, k.ConsFill, k.ConsSort })
                BindAll(k.Constraints, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot),
                    ("_JointDef", b.JointDef), ("_JointState", b.JointState), ("_SpringDef", b.SpringDef),
                    ("_ManifoldCur", b.ManifoldCur), ("_Counters", b.Counters),
                    ("_BodyConsCount", b.BodyConsCount), ("_BodyConsCursor", b.BodyConsCursor), ("_BodyConsStart", b.BodyConsStart), ("_BodyConsList", b.BodyConsList));

            // ---------------------------------------------------------------- colouring bindings
            foreach (int kk in new[] { k.ColorInvalidate, k.ColorRound, k.ColorFinalize, k.ColorScan, k.ColorScatter })
                BindAll(k.Coloring, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyConsStart", b.BodyConsStart), ("_BodyConsList", b.BodyConsList),
                    ("_ManifoldCur", b.ManifoldCur), ("_JointDef", b.JointDef), ("_SpringDef", b.SpringDef),
                    ("_ColorCount", b.ColorCount), ("_ColorStart", b.ColorStart), ("_ColorCursor", b.ColorCursor), ("_ColorList", b.ColorList),
                    ("_Counters", b.Counters), ("_DispatchArgs", b.DispatchArgs));

            // ---------------------------------------------------------------- solver bindings
            foreach (int kk in new[] { k.DriveKinematic, k.Predict, k.Primal, k.CommitOverflow, k.Dual, k.Velocity })
                BindAll(k.Solver, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot), ("_BodyPosNew", b.BodyPosNew), ("_BodyRotNew", b.BodyRotNew),
                    ("_BodyInitialLin", b.BodyInitialLin), ("_BodyInitialAng", b.BodyInitialAng),
                    ("_BodyInertialLin", b.BodyInertialLin), ("_BodyInertialAng", b.BodyInertialAng),
                    ("_BodyInitialLinW", b.BodyInitialLin), ("_BodyInitialAngW", b.BodyInitialAng),
                    ("_BodyInertialLinW", b.BodyInertialLin), ("_BodyInertialAngW", b.BodyInertialAng),
                    ("_BodyVelLin", b.BodyVelLin), ("_BodyVelAng", b.BodyVelAng), ("_BodyPrevVelLin", b.BodyPrevVelLin),
                    ("_BodyVelLinIn", b.BodyVelLin), ("_BodyVelAngIn", b.BodyVelAng), ("_BodyPrevVelLinIn", b.BodyPrevVelLin),
                    ("_BodyConsStart", b.BodyConsStart), ("_BodyConsList", b.BodyConsList),
                    ("_ColorStart", b.ColorStart), ("_ColorList", b.ColorList),
                    ("_ManifoldCur", b.ManifoldCur), ("_ContactsCur", b.ContactsCur), ("_ContactsCurRW", b.ContactsCur),
                    ("_JointDef", b.JointDef), ("_JointState", b.JointState), ("_JointStateRW", b.JointState), ("_SpringDef", b.SpringDef), ("_Counters", b.Counters),
                    ("_BodyDrive", b.BodyDrive));

            // ================================================================ step
            cb.BeginSample("AVBD broadphase");
            cb.SetComputeIntParam(k.Util, s_Phase, 0);
            Direct(k.Util, k.BuildArgs, 1);
            Direct(k.Util, k.HashClear, Groups(cfg.HashSize));

            Indirect(k.Solver, k.DriveKinematic, ArgBodies);   // heading / velocity-aligned orientations before the contacts are found
            Indirect(k.Broadphase, k.BodyAabb, ArgBodies);
            Direct(k.Broadphase, k.GridClear, Groups(cfg.CellCount));
            Indirect(k.Broadphase, k.GridCount, ArgBodies);
            RecordScan(b.CellCount, b.CellStart, cfg.CellCount);
            Indirect(k.Broadphase, k.GridScatter, ArgBodies);
            Direct(k.Broadphase, k.GridSortCell, Groups(cfg.CellCount));
            Direct(k.Broadphase, k.LargeSort, 1);
            Indirect(k.Broadphase, k.PairGen, ArgBodies);
            cb.SetComputeIntParam(k.Util, s_Phase, 1);
            Direct(k.Util, k.BuildArgs, 1);
            cb.EndSample("AVBD broadphase");

            cb.BeginSample("AVBD narrowphase");
            Indirect(k.Narrowphase, k.Collide, ArgPairs);
            if (m_Terrain) Indirect(k.Narrowphase, k.CollideTerrain, ArgBodies);   // appends to the same manifold pool
            cb.SetComputeIntParam(k.Util, s_Phase, 2);
            Direct(k.Util, k.BuildArgs, 1);
            cb.EndSample("AVBD narrowphase");

            cb.BeginSample("AVBD constraints");
            Indirect(k.Constraints, k.PrepareJoints, ArgJoints);
            Indirect(k.Constraints, k.ConsClear, ArgBodies);
            Indirect(k.Constraints, k.ConsCount, ArgConstraints);
            RecordScan(b.BodyConsCount, b.BodyConsStart, cfg.MaxBodies);
            Indirect(k.Constraints, k.ConsFill, ArgConstraints);
            Indirect(k.Constraints, k.ConsSort, ArgBodies);
            cb.EndSample("AVBD constraints");

            cb.BeginSample("AVBD coloring");
            // Invalidate: Color -> Tmp; rounds alternate; the last round must land in Tmp so Finalize writes Color.
            int rounds = m_ColorRounds;
            if ((rounds & 1) != 0) rounds++;
            var colorA = b.BodyColor; var colorB = b.BodyColorTmp;
            m_Cb.SetComputeBufferParam(k.Coloring, k.ColorInvalidate, s_ColorIn, colorA);
            m_Cb.SetComputeBufferParam(k.Coloring, k.ColorInvalidate, s_ColorOut, colorB);
            Indirect(k.Coloring, k.ColorInvalidate, ArgBodies);
            var cin = colorB; var cout = colorA;
            for (int r = 0; r < rounds; r++)
            {
                m_Cb.SetComputeBufferParam(k.Coloring, k.ColorRound, s_ColorIn, cin);
                m_Cb.SetComputeBufferParam(k.Coloring, k.ColorRound, s_ColorOut, cout);
                Indirect(k.Coloring, k.ColorRound, ArgBodies);
                (cin, cout) = (cout, cin);
            }
            // after an even number of rounds the latest colours are in colorB
            m_Cb.SetComputeBufferParam(k.Coloring, k.ColorFinalize, s_ColorIn, colorB);
            m_Cb.SetComputeBufferParam(k.Coloring, k.ColorFinalize, s_ColorOut, colorA);
            Indirect(k.Coloring, k.ColorFinalize, ArgBodies);
            Direct(k.Coloring, k.ColorScan, 1);
            m_Cb.SetComputeBufferParam(k.Coloring, k.ColorScatter, s_ColorIn, colorA);
            Indirect(k.Coloring, k.ColorScatter, ArgBodies);
            cb.EndSample("AVBD coloring");

            cb.BeginSample("AVBD solve");
            Indirect(k.Solver, k.Predict, ArgBodies);
            int total = m_Iterations + (m_PostStabilize ? 1 : 0);
            for (int it = 0; it < total; it++)
            {
                float alphaIt = m_PostStabilize ? (it < m_Iterations ? 1f : 0f) : m_Alpha;
                cb.SetComputeFloatParam(k.Solver, s_AlphaIt, alphaIt);
                cb.SetComputeIntParam(k.Solver, s_WriteToNew, 0);
                for (int c = 0; c < m_ActiveColors; c++)
                {
                    cb.SetComputeIntParam(k.Solver, s_ColorSlot, c);
                    Indirect(k.Solver, k.Primal, ArgColor0 + c);
                }
                cb.SetComputeIntParam(k.Solver, s_ColorSlot, m_ActiveColors);
                cb.SetComputeIntParam(k.Solver, s_WriteToNew, 1);
                Indirect(k.Solver, k.Primal, ArgColor0 + m_ActiveColors);
                Indirect(k.Solver, k.CommitOverflow, ArgColor0 + m_ActiveColors);

                if (it < m_Iterations) Indirect(k.Solver, k.Dual, ArgConstraints);
                if (it == m_Iterations - 1) Indirect(k.Solver, k.Velocity, ArgBodies);
            }
            cb.EndSample("AVBD solve");

            cb.BeginSample("AVBD end");
            cb.CopyBuffer(b.ManifoldCur, b.ManifoldPrev);
            cb.CopyBuffer(b.ContactsCur, b.ContactsPrev);
            cb.CopyBuffer(b.HashCur, b.HashPrev);
            Direct(k.Util, k.CopyStats, 1);
            cb.EndSample("AVBD end");
        }
    }
}
