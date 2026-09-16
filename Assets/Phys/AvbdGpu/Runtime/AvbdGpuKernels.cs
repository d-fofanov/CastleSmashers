using System;
using UnityEngine;

namespace Phys.AvbdGpu
{
    /// <summary>The compute shaders (loaded from Resources/AvbdGpu) and their kernel indices.</summary>
    public sealed class AvbdGpuKernels
    {
        public readonly ComputeShader Util, Scan, Broadphase, Narrowphase, Constraints, Coloring, Solver, Sleep, SleepGrid, Cold;

        public readonly int BuildArgs, HotFlag, HotScatter, HashClear, CopyStats, ClearUints, SpawnBodies;
        public readonly int ScanBlock, ScanTop, ScanAdd;
        public readonly int BodyAabb, GridClear, GridCount, GridScatter, GridSortCell, LargeSort, PairGen, PairGenWoken;
        public readonly int Collide, CollideTerrain;
        public readonly int JointList, JointListWoken, PrepareJoints, ConsClear, ConsCount, ConsFill, ConsSort;
        public readonly int ColorInvalidate, ColorRound, ColorFinalize, ColorScan, ColorScatter;
        public readonly int DriveKinematic, Predict, Primal, CommitOverflow, Dual, Velocity;
        public readonly int WakeList, WakeTouch, WakeApply, WakeClear, LabelRound, SleepTimer, RestSpread, RestSleep;
        public readonly int RebuildGate, SleepFlag, SleepScatter, RebuildListArgs, SleepGridClear, SleepGridCount, SleepGridScatter, SleepGridSortCell, SleepMark;
        public readonly int FreezeCount, FreezeCopy, FreezeFinish, ColdGate, ColdFlag, ColdGather, ColdWrite, ColdFinish, ColdHashClear, ColdHashInsert;

        public static bool Supported => SystemInfo.supportsComputeShaders;

        public AvbdGpuKernels()
        {
            Util = Load("AvbdUtil");
            Scan = Load("AvbdScan");
            Broadphase = Load("AvbdBroadphase");
            Narrowphase = Load("AvbdNarrowphase");
            Constraints = Load("AvbdConstraints");
            Coloring = Load("AvbdColoring");
            Solver = Load("AvbdSolver");
            Sleep = Load("AvbdSleep");
            SleepGrid = Load("AvbdSleepGrid");
            Cold = Load("AvbdCold");

            BuildArgs = Util.FindKernel("BuildArgs");
            HotFlag = Util.FindKernel("HotFlag");
            HotScatter = Util.FindKernel("HotScatter");
            HashClear = Util.FindKernel("HashClear");
            CopyStats = Util.FindKernel("CopyStats");
            ClearUints = Util.FindKernel("ClearUints");
            SpawnBodies = Util.FindKernel("SpawnBodies");
            ScanBlock = Scan.FindKernel("ScanBlock");
            ScanTop = Scan.FindKernel("ScanTop");
            ScanAdd = Scan.FindKernel("ScanAdd");
            BodyAabb = Broadphase.FindKernel("BodyAabb");
            GridClear = Broadphase.FindKernel("GridClear");
            GridCount = Broadphase.FindKernel("GridCount");
            GridScatter = Broadphase.FindKernel("GridScatter");
            GridSortCell = Broadphase.FindKernel("GridSortCell");
            LargeSort = Broadphase.FindKernel("LargeSort");
            PairGen = Broadphase.FindKernel("PairGen");
            PairGenWoken = Broadphase.FindKernel("PairGenWoken");
            Collide = Narrowphase.FindKernel("Collide");
            CollideTerrain = Narrowphase.FindKernel("CollideTerrain");
            JointList = Constraints.FindKernel("JointList");
            JointListWoken = Constraints.FindKernel("JointListWoken");
            PrepareJoints = Constraints.FindKernel("PrepareJoints");
            ConsClear = Constraints.FindKernel("ConsClear");
            ConsCount = Constraints.FindKernel("ConsCount");
            ConsFill = Constraints.FindKernel("ConsFill");
            ConsSort = Constraints.FindKernel("ConsSort");
            ColorInvalidate = Coloring.FindKernel("ColorInvalidate");
            ColorRound = Coloring.FindKernel("ColorRound");
            ColorFinalize = Coloring.FindKernel("ColorFinalize");
            ColorScan = Coloring.FindKernel("ColorScan");
            ColorScatter = Coloring.FindKernel("ColorScatter");
            DriveKinematic = Solver.FindKernel("DriveKinematic");
            Predict = Solver.FindKernel("Predict");
            Primal = Solver.FindKernel("Primal");
            CommitOverflow = Solver.FindKernel("CommitOverflow");
            Dual = Solver.FindKernel("Dual");
            Velocity = Solver.FindKernel("Velocity");
            WakeList = Sleep.FindKernel("WakeList");
            WakeTouch = Sleep.FindKernel("WakeTouch");
            WakeApply = Sleep.FindKernel("WakeApply");
            WakeClear = Sleep.FindKernel("WakeClear");
            LabelRound = Sleep.FindKernel("LabelRound");
            SleepTimer = Sleep.FindKernel("SleepTimer");
            RestSpread = Sleep.FindKernel("RestSpread");
            RestSleep = Sleep.FindKernel("RestSleep");
            RebuildGate = SleepGrid.FindKernel("RebuildGate");
            SleepFlag = SleepGrid.FindKernel("SleepFlag");
            SleepScatter = SleepGrid.FindKernel("SleepScatter");
            RebuildListArgs = SleepGrid.FindKernel("RebuildListArgs");
            SleepGridClear = SleepGrid.FindKernel("SleepGridClear");
            SleepGridCount = SleepGrid.FindKernel("SleepGridCount");
            SleepGridScatter = SleepGrid.FindKernel("SleepGridScatter");
            SleepGridSortCell = SleepGrid.FindKernel("SleepGridSortCell");
            SleepMark = SleepGrid.FindKernel("SleepMark");
            FreezeCount = Cold.FindKernel("FreezeCount");
            FreezeCopy = Cold.FindKernel("FreezeCopy");
            FreezeFinish = Cold.FindKernel("FreezeFinish");
            ColdGate = Cold.FindKernel("ColdGate");
            ColdFlag = Cold.FindKernel("ColdFlag");
            ColdGather = Cold.FindKernel("ColdGather");
            ColdWrite = Cold.FindKernel("ColdWrite");
            ColdFinish = Cold.FindKernel("ColdFinish");
            ColdHashClear = Cold.FindKernel("ColdHashClear");
            ColdHashInsert = Cold.FindKernel("ColdHashInsert");
        }

        static ComputeShader Load(string name)
        {
            var cs = Resources.Load<ComputeShader>("AvbdGpu/" + name);
            if (cs == null) throw new InvalidOperationException($"Compute shader Resources/AvbdGpu/{name} not found");
            return cs;
        }
    }
}
