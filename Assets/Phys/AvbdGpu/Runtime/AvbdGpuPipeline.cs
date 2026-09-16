using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu
{
    /// <summary>Records the whole simulation step into one CommandBuffer. Every count-dependent dispatch is indirect, so the
    /// buffer only has to be re-recorded when the iteration count, the post-stabilisation flag or the active colour count change.
    /// Two more command buffers rebuild the sleeping grid and compact the cold store; the world executes them every few steps
    /// and they decide on the GPU whether anything is due.</summary>
    public sealed class AvbdGpuPipeline
    {
        readonly AvbdGpuKernels m_K;
        readonly AvbdGpuBuffers m_B;
        readonly CommandBuffer m_Cb = new CommandBuffer { name = "AVBD step" };
        readonly CommandBuffer m_RebuildCb = new CommandBuffer { name = "AVBD sleeping grid" };
        readonly CommandBuffer m_CompactCb = new CommandBuffer { name = "AVBD cold store" };
        CommandBuffer m_Rec;   // the buffer the helpers record into

        int m_Iterations = -1, m_ActiveColors = -1, m_ColorRounds = -1, m_TerrainVersion = -1, m_LabelRounds = -1, m_SleepHops = -1;
        bool m_PostStabilize, m_Terrain, m_Sleep;

        public CommandBuffer CommandBuffer => m_Cb;
        /// <summary>The sleeping grid rebuild (gated on the GPU: nothing runs unless enough bodies fell asleep or woke).</summary>
        public CommandBuffer RebuildCommandBuffer => m_RebuildCb;
        /// <summary>The cold store compaction (gated on the GPU: nothing runs unless enough of the store was thawed or it is nearly full).</summary>
        public CommandBuffer CompactCommandBuffer => m_CompactCb;

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
        static readonly int s_LabelIn = Shader.PropertyToID("_LabelIn");
        static readonly int s_LabelOut = Shader.PropertyToID("_LabelOut");
        static readonly int s_LabelRound = Shader.PropertyToID("_LabelRound");
        static readonly int s_RestIn = Shader.PropertyToID("_RestIn");
        static readonly int s_RestOut = Shader.PropertyToID("_RestOut");
        static readonly int s_RestFirst = Shader.PropertyToID("_RestFirst");
        static readonly int s_SleepCells = Shader.PropertyToID("_SleepCells");
        static readonly int s_MaxBodies = Shader.PropertyToID("_MaxBodies");
        static readonly int s_MaxSleepCellEntries = Shader.PropertyToID("_MaxSleepCellEntries");
        static readonly int s_PairRound = Shader.PropertyToID("_PairRound");
        static readonly int s_ListSlot = Shader.PropertyToID("_ListSlot");
        static readonly int s_Chunk = Shader.PropertyToID("_Chunk");
        static readonly int s_ColdHashSize = Shader.PropertyToID("_ColdHashSize");

        // AvbdCommon.hlsl ARG_*
        const int ArgBodies = 0, ArgPairs = 1, ArgManifolds = 2, ArgConstraints = 3, ArgJoints = 4, ArgPairs2 = 5, ArgWakeList = 6, ArgColor0 = 8;
        const int ArgHot = ArgColor0 + AvbdGpuConstants.MaxColors + 1, ArgWoken = ArgHot + 1, ArgActiveJoints = ArgHot + 2, ArgActiveSprings = ArgHot + 3, ArgMarked = ArgHot + 4;
        const int ArgSleepFlag = ArgHot + 5, ArgSleepScan = ArgHot + 6, ArgSleepList = ArgHot + 7, ArgSleepCells = ArgHot + 8, ArgSleepCellScan = ArgHot + 9;
        const int ArgSleepScanTop = ArgHot + 10, ArgSleepCellScanTop = ArgHot + 11;
        const int ArgFreeze = ArgHot + 12, ArgFreezeScan = ArgHot + 13, ArgFreezeScanTop = ArgHot + 14;
        const int ArgColdFlag = ArgHot + 15, ArgColdScan = ArgHot + 16, ArgColdScanTop = ArgHot + 17, ArgColdChunk = ArgHot + 18, ArgColdHash = ArgHot + 19, ArgColdInsert = ArgHot + 20;
        // AvbdCommon.hlsl CNT_* the kernels take as parameters
        const int CntHot = 14, CntWokenList = 15;
        const int Threads = AvbdGpuConstants.ThreadGroupSize;
        const int ScanBlock = 1024;

        public AvbdGpuPipeline(AvbdGpuKernels kernels, AvbdGpuBuffers buffers)
        {
            m_K = kernels;
            m_B = buffers;
        }

        public void Dispose() { m_Cb.Release(); m_RebuildCb.Release(); m_CompactCb.Release(); }

        static int Groups(int n, int per = Threads) => math.max(1, (n + per - 1) / per);

        /// <summary>Re-records the step when the configuration changed (<paramref name="terrain"/>: the world has a terrain, so the
        /// CollideTerrain pass is recorded and bound to the current terrain buffers; <paramref name="sleep"/>: the sleeping passes
        /// are recorded, with <paramref name="labelRounds"/> island label rounds and <paramref name="sleepHops"/> rest rounds per step).</summary>
        public void Ensure(int iterations, int activeColors, bool postStabilize, int colorRounds, float alpha, bool terrain, bool sleep, int labelRounds, int sleepHops)
        {
            int terrainVersion = terrain ? m_B.TerrainVersion : -1;
            if (iterations == m_Iterations && activeColors == m_ActiveColors && postStabilize == m_PostStabilize && colorRounds == m_ColorRounds && alpha == m_Alpha
                && terrain == m_Terrain && terrainVersion == m_TerrainVersion && sleep == m_Sleep && labelRounds == m_LabelRounds && sleepHops == m_SleepHops) return;
            m_Iterations = iterations; m_ActiveColors = activeColors; m_PostStabilize = postStabilize; m_ColorRounds = colorRounds; m_Alpha = alpha;
            m_Terrain = terrain; m_TerrainVersion = terrainVersion; m_Sleep = sleep; m_LabelRounds = labelRounds; m_SleepHops = sleepHops;
            Record();
            RecordRebuild();
            RecordCompact();
        }

        float m_Alpha = -1f;

        void Bind(ComputeShader cs, int kernel, string name, GraphicsBuffer buffer) => m_Rec.SetComputeBufferParam(cs, kernel, name, buffer);

        void BindAll(ComputeShader cs, int kernel, params (string, GraphicsBuffer)[] bindings)
        {
            foreach (var (name, buffer) in bindings) Bind(cs, kernel, name, buffer);
        }

        void Indirect(ComputeShader cs, int kernel, int argSlot) => m_Rec.DispatchCompute(cs, kernel, m_B.DispatchArgs, (uint)(argSlot * 12));

        void Direct(ComputeShader cs, int kernel, int groups) => m_Rec.DispatchCompute(cs, kernel, groups, 1, 1);

        /// <summary>Exclusive scan of <paramref name="n"/> entries; with arg slots the three dispatches are indirect (the rebuild
        /// gate writes them, zero when nothing is due).</summary>
        void RecordScan(GraphicsBuffer src, GraphicsBuffer dst, int n, int argSlot = -1, int argTopSlot = -1)
        {
            var cs = m_K.Scan;
            foreach (int k in new[] { m_K.ScanBlock, m_K.ScanTop, m_K.ScanAdd })
            {
                m_Rec.SetComputeBufferParam(cs, k, s_ScanIn, src);
                m_Rec.SetComputeBufferParam(cs, k, s_ScanOut, dst);
                Bind(cs, k, "_BlockSums", m_B.BlockSums);
            }
            m_Rec.SetComputeIntParam(cs, s_ScanN, n);
            if (argSlot < 0)
            {
                Direct(cs, m_K.ScanBlock, Groups(n, ScanBlock));
                Direct(cs, m_K.ScanTop, 1);
                Direct(cs, m_K.ScanAdd, Groups(n, ScanBlock));
            }
            else
            {
                Indirect(cs, m_K.ScanBlock, argSlot);
                Indirect(cs, m_K.ScanTop, argTopSlot);
                Indirect(cs, m_K.ScanAdd, argSlot);
            }
        }

        void BuildArgs(int phase)
        {
            m_Rec.SetComputeIntParam(m_K.Util, s_Phase, phase);
            Direct(m_K.Util, m_K.BuildArgs, 1);
        }

        void Record()
        {
            var cb = m_Rec = m_Cb;
            var b = m_B;
            var k = m_K;
            var cfg = b.Config;
            cb.Clear();

            foreach (var cs in new[] { k.Util, k.Scan, k.Broadphase, k.Narrowphase, k.Constraints, k.Coloring, k.Solver, k.Sleep })
                cb.SetComputeConstantBufferParam(cs, s_Params, b.Params, 0, GpuParams.Stride);

            // ---------------------------------------------------------------- util bindings
            foreach (int kk in new[] { k.BuildArgs, k.HashClear, k.CopyStats, k.HotFlag, k.HotScatter })
                BindAll(k.Util, kk, ("_Counters", b.Counters), ("_Stats", b.Stats), ("_DispatchArgs", b.DispatchArgs), ("_HashCur", b.HashCur),
                    ("_BodyDef", b.BodyDef), ("_BodySleep", b.BodySleep), ("_HotFlags", b.HotFlags), ("_HotScan", b.HotScan), ("_HotList", b.HotList));

            // ---------------------------------------------------------------- broadphase bindings
            foreach (int kk in new[] { k.BodyAabb, k.GridClear, k.GridCount, k.GridScatter, k.GridSortCell, k.LargeSort, k.PairGen, k.PairGenWoken })
                BindAll(k.Broadphase, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot),
                    ("_BodyAabbMin", b.BodyAabbMin), ("_BodyAabbMax", b.BodyAabbMax),
                    ("_CellCount", b.CellCount), ("_CellStart", b.CellStart), ("_CellCursor", b.CellCursor), ("_CellEntries", b.CellEntries),
                    ("_LargeBodies", b.LargeBodies), ("_Counters", b.Counters), ("_Pairs", b.Pairs),
                    ("_LinkStart", b.LinkStart), ("_LinkList", b.LinkList), ("_JointState", b.JointState), ("_BodySleep", b.BodySleep),
                    ("_HotList", b.HotList), ("_WokenList", b.WokenList), ("_SleepCellStart", b.SleepCellStart), ("_SleepCellEntries", b.SleepCellEntries));

            // ---------------------------------------------------------------- narrowphase bindings
            foreach (int kk in new[] { k.Collide, k.CollideTerrain })
                BindAll(k.Narrowphase, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot), ("_BodyVelLin", b.BodyVelLin), ("_Pairs", b.Pairs), ("_Counters", b.Counters),
                    ("_ManifoldPrev", b.ManifoldPrev), ("_ContactsPrev", b.ContactsPrev), ("_HashPrev", b.HashPrev),
                    ("_ManifoldCur", b.ManifoldCur), ("_ContactsCur", b.ContactsCur), ("_HashCur", b.HashCur), ("_BodyEvents", b.BodyEvents),
                    ("_BodyAabbMin", b.BodyAabbMin), ("_BodyAabbMax", b.BodyAabbMax), ("_TerrainHeights", b.TerrainHeights), ("_TerrainMaxMip", b.TerrainMaxMip),
                    ("_BodySleep", b.BodySleep), ("_HotList", b.HotList), ("_BodyGen", b.BodyGen),
                    ("_ColdManifoldsRW", b.ColdManifolds), ("_ColdContacts", b.ColdContacts), ("_ColdHash", b.ColdHash));

            // ---------------------------------------------------------------- cold store bindings (the freeze passes)
            foreach (int kk in new[] { k.FreezeCount, k.FreezeCopy, k.FreezeFinish })
                BindAll(k.Cold, kk,
                    ("_BodyDef", b.BodyDef), ("_BodySleep", b.BodySleep), ("_BodyGen", b.BodyGen), ("_ManifoldCur", b.ManifoldCur), ("_ContactsCur", b.ContactsCur),
                    ("_FreezeFlag", b.FreezeFlag), ("_FreezeContacts", b.FreezeContacts), ("_FreezeFlagScan", b.FreezeFlagScan), ("_FreezeContactScan", b.FreezeContactScan),
                    ("_ColdManifolds", b.ColdManifolds), ("_ColdContacts", b.ColdContacts), ("_ColdHash", b.ColdHash), ("_Counters", b.Counters), ("_DispatchArgs", b.DispatchArgs));

            // ---------------------------------------------------------------- constraint bindings
            foreach (int kk in new[] { k.JointList, k.JointListWoken, k.PrepareJoints, k.ConsClear, k.ConsCount, k.ConsFill, k.ConsSort })
                BindAll(k.Constraints, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot),
                    ("_JointDef", b.JointDef), ("_JointState", b.JointState), ("_SpringDef", b.SpringDef),
                    ("_ManifoldCur", b.ManifoldCur), ("_Counters", b.Counters),
                    ("_BodyConsCount", b.BodyConsCount), ("_BodyConsCursor", b.BodyConsCursor), ("_BodyConsStart", b.BodyConsStart), ("_BodyConsList", b.BodyConsList),
                    ("_BodySleep", b.BodySleep), ("_HotList", b.HotList), ("_WokenList", b.WokenList), ("_LinkStart", b.LinkStart), ("_LinkList", b.LinkList),
                    ("_ActiveJoints", b.ActiveJoints), ("_ActiveSprings", b.ActiveSprings));

            // ---------------------------------------------------------------- colouring bindings
            foreach (int kk in new[] { k.ColorInvalidate, k.ColorRound, k.ColorFinalize, k.ColorScan, k.ColorScatter })
                BindAll(k.Coloring, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyConsStart", b.BodyConsStart), ("_BodyConsList", b.BodyConsList),
                    ("_ManifoldCur", b.ManifoldCur), ("_JointDef", b.JointDef), ("_SpringDef", b.SpringDef),
                    ("_ColorCount", b.ColorCount), ("_ColorStart", b.ColorStart), ("_ColorCursor", b.ColorCursor), ("_ColorList", b.ColorList),
                    ("_Counters", b.Counters), ("_DispatchArgs", b.DispatchArgs), ("_BodySleep", b.BodySleep), ("_HotList", b.HotList));

            // ---------------------------------------------------------------- sleeping bindings
            foreach (int kk in new[] { k.WakeList, k.WakeTouch, k.WakeApply, k.WakeClear, k.LabelRound, k.SleepTimer, k.RestSpread, k.RestSleep })
                BindAll(k.Sleep, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot), ("_BodyVelLinIn", b.BodyVelLin), ("_WakeList", b.WakeList),
                    ("_BodyConsStart", b.BodyConsStart), ("_BodyConsList", b.BodyConsList), ("_ManifoldCur", b.ManifoldCur),
                    ("_JointDef", b.JointDef), ("_JointState", b.JointState), ("_SpringDef", b.SpringDef),
                    ("_BodySleep", b.BodySleep), ("_BodyLabel", b.BodyLabel), ("_WakeMark", b.WakeMark), ("_BodyRestPose", b.BodyRestPose),
                    ("_BodyVelLin", b.BodyVelLin), ("_BodyVelAng", b.BodyVelAng), ("_BodyPrevVelLin", b.BodyPrevVelLin), ("_Counters", b.Counters),
                    ("_HotList", b.HotList), ("_WokenList", b.WokenList), ("_ActiveJoints", b.ActiveJoints), ("_ActiveSprings", b.ActiveSprings),
                    ("_BodyAabbMin", b.BodyAabbMin), ("_BodyAabbMax", b.BodyAabbMax), ("_CellStart", b.CellStart), ("_CellEntries", b.CellEntries),
                    ("_SleepCellStart", b.SleepCellStart), ("_SleepCellEntries", b.SleepCellEntries), ("_BodyGen", b.BodyGen));

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
                    ("_BodyDrive", b.BodyDrive), ("_BodySleep", b.BodySleep), ("_HotList", b.HotList), ("_ActiveJoints", b.ActiveJoints));

            // ================================================================ step
            cb.BeginSample("AVBD hot list");
            BuildArgs(0);
            Direct(k.Util, k.HashClear, Groups(cfg.HashSize));
            if (m_Sleep) Indirect(k.Sleep, k.WakeList, ArgWakeList);   // CPU wakes, before the pairs: a woken body gets fresh contacts
            // the hot list: the bodies of every per-body pass (index order through the scan), then their joints and springs
            Indirect(k.Util, k.HotFlag, ArgBodies);
            RecordScan(b.HotFlags, b.HotScan, cfg.MaxBodies);
            Indirect(k.Util, k.HotScatter, ArgBodies);
            BuildArgs(3);
            Indirect(k.Constraints, k.JointList, ArgHot);
            cb.EndSample("AVBD hot list");

            cb.BeginSample("AVBD broadphase");
            Indirect(k.Solver, k.DriveKinematic, ArgHot);   // heading / velocity-aligned orientations before the contacts are found
            Indirect(k.Broadphase, k.BodyAabb, ArgHot);
            Direct(k.Broadphase, k.GridClear, Groups(cfg.CellCount));
            Indirect(k.Broadphase, k.GridCount, ArgHot);
            RecordScan(b.CellCount, b.CellStart, cfg.CellCount);
            Indirect(k.Broadphase, k.GridScatter, ArgHot);
            Direct(k.Broadphase, k.GridSortCell, Groups(cfg.CellCount));
            Direct(k.Broadphase, k.LargeSort, 1);
            Indirect(k.Broadphase, k.PairGen, ArgHot);
            BuildArgs(1);
            cb.EndSample("AVBD broadphase");

            cb.BeginSample("AVBD narrowphase");
            cb.SetComputeIntParam(k.Narrowphase, s_PairRound, 0);
            Indirect(k.Narrowphase, k.Collide, ArgPairs);
            cb.SetComputeIntParam(k.Narrowphase, s_ListSlot, CntHot);
            if (m_Terrain) Indirect(k.Narrowphase, k.CollideTerrain, ArgHot);   // appends to the same manifold pool
            BuildArgs(2);
            cb.EndSample("AVBD narrowphase");

            if (m_Sleep)
            {
                // a touch by an awake body wakes the whole island of the touched body before anything is solved; the woken
                // bodies join the hot list, get their contacts in a second narrowphase round (warm started from the cold
                // store) and their joints join the active lists
                cb.BeginSample("AVBD wake");
                Indirect(k.Sleep, k.WakeTouch, ArgConstraints);
                BuildArgs(5);
                Indirect(k.Sleep, k.WakeApply, ArgMarked);
                Indirect(k.Sleep, k.WakeClear, ArgMarked);
                BuildArgs(4);
                Indirect(k.Broadphase, k.PairGenWoken, ArgWoken);
                BuildArgs(7);
                cb.SetComputeIntParam(k.Narrowphase, s_PairRound, 1);
                Indirect(k.Narrowphase, k.Collide, ArgPairs2);
                if (m_Terrain)
                {
                    cb.SetComputeBufferParam(k.Narrowphase, k.CollideTerrain, "_HotList", b.WokenList);
                    cb.SetComputeIntParam(k.Narrowphase, s_ListSlot, CntWokenList);
                    Indirect(k.Narrowphase, k.CollideTerrain, ArgWoken);
                    cb.SetComputeBufferParam(k.Narrowphase, k.CollideTerrain, "_HotList", b.HotList);
                }
                Indirect(k.Constraints, k.JointListWoken, ArgWoken);
                BuildArgs(6);
                cb.EndSample("AVBD wake");
            }

            cb.BeginSample("AVBD constraints");
            Indirect(k.Constraints, k.PrepareJoints, ArgActiveJoints);
            Indirect(k.Constraints, k.ConsClear, ArgBodies);
            Indirect(k.Constraints, k.ConsCount, ArgConstraints);
            RecordScan(b.BodyConsCount, b.BodyConsStart, cfg.MaxBodies);
            Indirect(k.Constraints, k.ConsFill, ArgConstraints);
            Indirect(k.Constraints, k.ConsSort, ArgHot);
            cb.EndSample("AVBD constraints");

            if (m_Sleep)
            {
                // island labels: an even number of ping-pong rounds so that the result lands back in BodyLabel
                cb.BeginSample("AVBD islands");
                int labelRounds = math.max(2, m_LabelRounds + (m_LabelRounds & 1));
                for (int r = 0; r < labelRounds; r++)
                {
                    cb.SetComputeBufferParam(k.Sleep, k.LabelRound, s_LabelIn, (r & 1) == 0 ? b.BodyLabel : b.BodyLabelTmp);
                    cb.SetComputeBufferParam(k.Sleep, k.LabelRound, s_LabelOut, (r & 1) == 0 ? b.BodyLabelTmp : b.BodyLabel);
                    cb.SetComputeIntParam(k.Sleep, s_LabelRound, r);
                    Indirect(k.Sleep, k.LabelRound, ArgHot);
                }
                cb.EndSample("AVBD islands");
            }

            cb.BeginSample("AVBD coloring");
            // Invalidate: Color -> Tmp; rounds alternate; the last round must land in Tmp so Finalize writes Color.
            int rounds = m_ColorRounds;
            if ((rounds & 1) != 0) rounds++;
            var colorA = b.BodyColor; var colorB = b.BodyColorTmp;
            cb.SetComputeBufferParam(k.Coloring, k.ColorInvalidate, s_ColorIn, colorA);
            cb.SetComputeBufferParam(k.Coloring, k.ColorInvalidate, s_ColorOut, colorB);
            Indirect(k.Coloring, k.ColorInvalidate, ArgHot);
            var cin = colorB; var cout = colorA;
            for (int r = 0; r < rounds; r++)
            {
                cb.SetComputeBufferParam(k.Coloring, k.ColorRound, s_ColorIn, cin);
                cb.SetComputeBufferParam(k.Coloring, k.ColorRound, s_ColorOut, cout);
                Indirect(k.Coloring, k.ColorRound, ArgHot);
                (cin, cout) = (cout, cin);
            }
            // after an even number of rounds the latest colours are in colorB
            cb.SetComputeBufferParam(k.Coloring, k.ColorFinalize, s_ColorIn, colorB);
            cb.SetComputeBufferParam(k.Coloring, k.ColorFinalize, s_ColorOut, colorA);
            Indirect(k.Coloring, k.ColorFinalize, ArgHot);
            Direct(k.Coloring, k.ColorScan, 1);
            cb.SetComputeBufferParam(k.Coloring, k.ColorScatter, s_ColorIn, colorA);
            Indirect(k.Coloring, k.ColorScatter, ArgHot);
            cb.EndSample("AVBD coloring");

            cb.BeginSample("AVBD solve");
            Indirect(k.Solver, k.Predict, ArgHot);
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
                if (it == m_Iterations - 1) Indirect(k.Solver, k.Velocity, ArgHot);
            }
            cb.EndSample("AVBD solve");

            if (m_Sleep)
            {
                // rest counters, their minimum over each body's neighbourhood (one round per hop), and the bodies whose
                // neighbourhood has rested long enough fall asleep
                cb.BeginSample("AVBD sleep");
                Indirect(k.Sleep, k.SleepTimer, ArgHot);
                int hops = math.max(1, m_SleepHops);
                var restIn = b.RestMin; var restOut = b.RestMinTmp;
                for (int r = 0; r < hops; r++)
                {
                    cb.SetComputeIntParam(k.Sleep, s_RestFirst, r == 0 ? 1 : 0);
                    cb.SetComputeBufferParam(k.Sleep, k.RestSpread, s_RestIn, restIn);
                    cb.SetComputeBufferParam(k.Sleep, k.RestSpread, s_RestOut, restOut);
                    Indirect(k.Sleep, k.RestSpread, ArgHot);
                    (restIn, restOut) = (restOut, restIn);
                }
                cb.SetComputeBufferParam(k.Sleep, k.RestSleep, s_RestIn, restIn);   // the last round wrote here
                Indirect(k.Sleep, k.RestSleep, ArgHot);
                cb.EndSample("AVBD sleep");

                // the manifolds of the bodies that fell asleep move into the cold store (nothing runs when none did)
                cb.BeginSample("AVBD freeze");
                BuildArgs(8);
                Indirect(k.Cold, k.FreezeCount, ArgFreeze);
                RecordScan(b.FreezeFlag, b.FreezeFlagScan, cfg.MaxManifolds, ArgFreezeScan, ArgFreezeScanTop);
                RecordScan(b.FreezeContacts, b.FreezeContactScan, cfg.MaxManifolds, ArgFreezeScan, ArgFreezeScanTop);
                Indirect(k.Cold, k.FreezeCopy, ArgFreeze);
                Indirect(k.Cold, k.FreezeFinish, ArgFreezeScanTop);
                cb.EndSample("AVBD freeze");
            }

            cb.BeginSample("AVBD end");
            cb.CopyBuffer(b.ManifoldCur, b.ManifoldPrev);
            cb.CopyBuffer(b.ContactsCur, b.ContactsPrev);
            cb.CopyBuffer(b.HashCur, b.HashPrev);
            Direct(k.Util, k.CopyStats, 1);
            cb.EndSample("AVBD end");
        }

        /// <summary>The sleeping grid rebuild: flags the inactive bodies, compacts them into a list, counting-sorts them into the
        /// grid's cells and marks them. Every dispatch is indirect from the gate's arguments, so an undue rebuild costs nothing
        /// but the empty dispatches.</summary>
        void RecordRebuild()
        {
            var cb = m_Rec = m_RebuildCb;
            var b = m_B;
            var k = m_K;
            var cfg = b.Config;
            cb.Clear();
            cb.SetComputeConstantBufferParam(k.SleepGrid, s_Params, b.Params, 0, GpuParams.Stride);
            cb.SetComputeConstantBufferParam(k.Scan, s_Params, b.Params, 0, GpuParams.Stride);
            foreach (int kk in new[] { k.RebuildGate, k.SleepFlag, k.SleepScatter, k.RebuildListArgs, k.SleepGridClear, k.SleepGridCount, k.SleepGridScatter, k.SleepGridSortCell, k.SleepMark })
                BindAll(k.SleepGrid, kk,
                    ("_BodyDef", b.BodyDef), ("_BodyPos", b.BodyPos), ("_BodyRot", b.BodyRot), ("_BodySleep", b.BodySleep),
                    ("_BodyAabbMin", b.BodyAabbMin), ("_BodyAabbMax", b.BodyAabbMax),
                    ("_SleepFlags", b.SleepFlags), ("_SleepScan", b.SleepScan), ("_SleepList", b.SleepList),
                    ("_SleepCellCount", b.SleepCellCount), ("_SleepCellStart", b.SleepCellStart), ("_SleepCellCursor", b.SleepCellCursor), ("_SleepCellEntries", b.SleepCellEntries),
                    ("_Counters", b.Counters), ("_DispatchArgs", b.DispatchArgs));
            cb.SetComputeIntParam(k.SleepGrid, s_SleepCells, cfg.SleepCellCount);
            cb.SetComputeIntParam(k.SleepGrid, s_MaxBodies, cfg.MaxBodies);
            cb.SetComputeIntParam(k.SleepGrid, s_MaxSleepCellEntries, cfg.MaxSleepCellEntries);

            cb.BeginSample("AVBD sleeping grid");
            Direct(k.SleepGrid, k.RebuildGate, 1);
            Indirect(k.SleepGrid, k.SleepFlag, ArgSleepFlag);
            RecordScan(b.SleepFlags, b.SleepScan, cfg.MaxBodies, ArgSleepScan, ArgSleepScanTop);
            Indirect(k.SleepGrid, k.SleepScatter, ArgSleepFlag);
            Direct(k.SleepGrid, k.RebuildListArgs, 1);
            Indirect(k.SleepGrid, k.SleepGridClear, ArgSleepCells);
            Indirect(k.SleepGrid, k.SleepGridCount, ArgSleepList);
            RecordScan(b.SleepCellCount, b.SleepCellStart, cfg.SleepCellCount, ArgSleepCellScan, ArgSleepCellScanTop);
            Indirect(k.SleepGrid, k.SleepGridScatter, ArgSleepList);
            Indirect(k.SleepGrid, k.SleepGridSortCell, ArgSleepCells);
            Indirect(k.SleepGrid, k.SleepMark, ArgSleepList);
            cb.EndSample("AVBD sleeping grid");
            m_Rec = m_Cb;
        }

        /// <summary>The cold store compaction: flags the live manifolds, scans them and their contacts, and squeezes them down in
        /// place chunk by chunk through the scratch (the contacts lie in manifold order, so a chunk's compacted position never
        /// reaches into a later chunk), then rebuilds the hash. Every dispatch is indirect from the gate's arguments.</summary>
        void RecordCompact()
        {
            var cb = m_Rec = m_CompactCb;
            var b = m_B;
            var k = m_K;
            var cfg = b.Config;
            cb.Clear();
            cb.SetComputeConstantBufferParam(k.Cold, s_Params, b.Params, 0, GpuParams.Stride);
            foreach (int kk in new[] { k.ColdGate, k.ColdFlag, k.ColdGather, k.ColdWrite, k.ColdFinish, k.ColdHashClear, k.ColdHashInsert })
                BindAll(k.Cold, kk,
                    ("_BodyDef", b.BodyDef), ("_BodySleep", b.BodySleep), ("_BodyGen", b.BodyGen),
                    ("_ColdManifolds", b.ColdManifolds), ("_ColdContacts", b.ColdContacts), ("_ColdHash", b.ColdHash),
                    ("_ColdFlag", b.ColdFlag), ("_ColdContactCount", b.ColdContactCount), ("_ColdFlagScan", b.ColdFlagScan), ("_ColdContactScan", b.ColdContactScan),
                    ("_ColdScratch", b.ColdScratch), ("_ColdScratchContacts", b.ColdScratchContacts),
                    ("_Counters", b.Counters), ("_DispatchArgs", b.DispatchArgs));
            cb.SetComputeIntParam(k.Cold, s_ColdHashSize, cfg.ColdHashSize);

            cb.BeginSample("AVBD cold store");
            Direct(k.Cold, k.ColdGate, 1);
            Indirect(k.Cold, k.ColdFlag, ArgColdFlag);
            RecordScan(b.ColdFlag, b.ColdFlagScan, cfg.MaxColdManifolds, ArgColdScan, ArgColdScanTop);
            RecordScan(b.ColdContactCount, b.ColdContactScan, cfg.MaxColdManifolds, ArgColdScan, ArgColdScanTop);
            int chunks = (cfg.MaxColdManifolds + AvbdGpuConfig.ColdChunk - 1) / AvbdGpuConfig.ColdChunk;
            for (int c = 0; c < chunks; c++)
            {
                cb.SetComputeIntParam(k.Cold, s_Chunk, c);
                Indirect(k.Cold, k.ColdGather, ArgColdChunk);
                Indirect(k.Cold, k.ColdWrite, ArgColdChunk);
            }
            Indirect(k.Cold, k.ColdFinish, ArgColdScanTop);
            Indirect(k.Cold, k.ColdHashClear, ArgColdHash);
            Indirect(k.Cold, k.ColdHashInsert, ArgColdInsert);
            cb.EndSample("AVBD cold store");
            m_Rec = m_Cb;
        }
    }
}
