using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu
{
    /// <summary>Buffer capacities. Everything is allocated once; appends beyond a capacity are dropped and reported in the stats.
    /// <see cref="MaxBodies"/> bounds the bodies of the world (the per-body arrays), <see cref="MaxActive"/> the awake ones (the
    /// hot grid, the pairs and the colour list are sized for them; sleeping bodies live in their own grid).</summary>
    [Serializable]
    public struct AvbdGpuConfig
    {
        public int MaxBodies;
        public int MaxActive;
        public int MaxJoints;
        public int MaxSprings;
        public int MaxLinks;          // joint + spring + ignore entries, two per link
        public int MaxManifolds;
        public int MaxContacts;
        public int MaxPairs;
        public int MaxCellEntries;
        public int MaxLargeBodies;
        public int LargeBodyCells;    // bodies spanning more grid cells than this bypass the grid
        public int HashSize;          // power of two, >= 2 * MaxManifolds
        public int MaxColdManifolds;  // the cold store: manifolds of sleeping bodies and their contacts
        public int MaxColdContacts;
        public int ColdHashSize;      // power of two, >= 2 * MaxColdManifolds
        public int CellCount;         // power of two: the hot grid
        public int SleepCellCount;    // power of two: the sleeping grid
        public int MaxSleepCellEntries;
        public int MaxSpawns;         // spawn records uploaded per step (larger batches are flushed in chunks)
        public int MaxWakes;          // wake list entries per step (more wake everything)

        /// <summary>Capacities for <paramref name="bodies"/> bodies of which at most <paramref name="active"/> are awake at a time
        /// (0: all of them). The manifold pools are sized for the awake bodies, the cold store (the manifolds of the sleeping ones)
        /// for four manifolds per body; scenes whose bodies rarely touch (snapped bricks) can lower <see cref="MaxColdManifolds"/>.</summary>
        public static AvbdGpuConfig ForBodies(int bodies, int active = 0)
        {
            bodies = math.max(bodies, 1024);
            active = active <= 0 ? bodies : math.clamp(active, 1024, bodies);
            int manifolds = active * 4;
            int cold = bodies * 4;
            return new AvbdGpuConfig
            {
                MaxBodies = bodies,
                MaxActive = active,
                MaxJoints = math.max(bodies, 4096),
                MaxSprings = math.max(bodies / 4, 1024),
                MaxLinks = math.max(bodies * 4, 16384),
                MaxManifolds = manifolds,
                MaxContacts = manifolds * 4,
                MaxPairs = active * 10,      // tightly packed piles pair every box with ~6-8 AABB neighbours
                MaxCellEntries = active * 8,
                MaxLargeBodies = 256,
                LargeBodyCells = 64,
                HashSize = math.ceilpow2(manifolds * 2),
                MaxColdManifolds = cold,
                MaxColdContacts = cold * 4,
                ColdHashSize = math.ceilpow2(cold * 2),
                CellCount = math.ceilpow2(active * 2),
                SleepCellCount = math.ceilpow2(bodies * 2),
                MaxSleepCellEntries = bodies * 8,
                MaxSpawns = 4096,
                MaxWakes = 4096,
            };
        }

        public static AvbdGpuConfig Default => ForBodies(65536);

        public int MaxConstraintRefs => 2 * MaxManifolds + 2 * (MaxJoints + MaxSprings);
        /// <summary>Manifolds per in-place chunk of the cold store compaction (AvbdCommon.hlsl COLD_CHUNK).</summary>
        public const int ColdChunk = 16384;

        public long EstimatedBytes =>
            (long)MaxBodies * (GpuBodyDef.Stride + GpuBodyDrive.Stride + 16 * 16 + 32 + 8 * 4) + (long)MaxSpawns * GpuSpawnRecord.Stride + (long)MaxWakes * 8 + (long)MaxLinks * 8 +
            (long)CellCount * 12 + 4 + (long)MaxCellEntries * 4 + (long)MaxPairs * 8 + (long)SleepCellCount * 12 + 4 + (long)MaxSleepCellEntries * 4 +
            2L * MaxManifolds * GpuManifold.Stride + 2L * MaxContacts * GpuContact.Stride + 2L * HashSize * 4 + (long)MaxManifolds * 16 + 8 +
            (long)MaxColdManifolds * (GpuManifold.Stride + 16) + 8 + (long)MaxColdContacts * GpuContact.Stride + (long)ColdHashSize * 4 +
            (long)ColdChunk * (GpuManifold.Stride + 8 * GpuContact.Stride) +
            (long)MaxJoints * (GpuJointDef.Stride + GpuJointState.Stride + 4) + (long)MaxSprings * (GpuSpringDef.Stride + 4) +
            (long)MaxConstraintRefs * 4;
    }

    /// <summary>All GPU buffers of a world. Names match the HLSL declarations (used for binding).</summary>
    public sealed class AvbdGpuBuffers : IDisposable
    {
        public readonly AvbdGpuConfig Config;
        readonly List<GraphicsBuffer> m_All = new List<GraphicsBuffer>();

        // bodies
        public GraphicsBuffer BodyDef, BodyPos, BodyRot, BodyPosNew, BodyRotNew, BodyInitialLin, BodyInitialAng, BodyInertialLin, BodyInertialAng,
            BodyVelLin, BodyVelAng, BodyPrevVelLin, BodyAabbMin, BodyAabbMax, BodyColor, BodyColorTmp, BodyDrive, BodyEvents, SpawnRecords;
        // sleeping: sleep words, island labels (ping-pong), neighbourhood rest minima (ping-pong), wake marks, rest anchors, the CPU wake list
        public GraphicsBuffer BodySleep, BodyLabel, BodyLabelTmp, RestMin, RestMinTmp, WakeMark, BodyRestPose, WakeList;
        // the hot list (flags, their scan, the list), the bodies woken by a touch, the active joint and spring lists
        public GraphicsBuffer HotFlags, HotScan, HotList, WokenList, ActiveJoints, ActiveSprings;
        // the sleeping grid: its rebuild's flags, scan and body list, and the grid itself
        public GraphicsBuffer SleepFlags, SleepScan, SleepList, SleepCellCount, SleepCellStart, SleepCellCursor, SleepCellEntries;
        // the cold store: sleep generations, the pool (manifolds, contacts, hash), the freeze flags / counts and their scans,
        // the compaction flags / counts, their scans and the chunk scratch
        public GraphicsBuffer BodyGen, ColdManifolds, ColdContacts, ColdHash, FreezeFlag, FreezeContacts, FreezeFlagScan, FreezeContactScan,
            ColdFlag, ColdContactCount, ColdFlagScan, ColdContactScan, ColdScratch, ColdScratchContacts;
        // links
        public GraphicsBuffer LinkStart, LinkList;
        // the hot grid
        public GraphicsBuffer CellCount, CellStart, CellCursor, CellEntries, LargeBodies, BlockSums;
        // pairs and manifolds
        public GraphicsBuffer Pairs, ManifoldPrev, ManifoldCur, ContactsPrev, ContactsCur, HashPrev, HashCur;
        // joints and springs
        public GraphicsBuffer JointDef, JointState, SpringDef;
        // CSR
        public GraphicsBuffer BodyConsCount, BodyConsCursor, BodyConsStart, BodyConsList;
        // colours
        public GraphicsBuffer ColorCount, ColorStart, ColorCursor, ColorList;
        // terrain (sized by the first SetTerrain, grown when a larger field arrives; TerrainVersion changes with the objects)
        public GraphicsBuffer TerrainHeights, TerrainMaxMip;
        public int TerrainVersion { get; private set; }
        // misc
        public GraphicsBuffer Counters, Stats, DispatchArgs, Params;

        public const int ArgSlots = 8 + AvbdGpuConstants.MaxColors + 1 + 24;

        public AvbdGpuBuffers(AvbdGpuConfig config)
        {
            Config = config;
            int nb = config.MaxBodies;
            BodyDef = Structured(nb, GpuBodyDef.Stride);
            BodyPos = Structured(nb, 16); BodyRot = Structured(nb, 16);
            BodyPosNew = Structured(nb, 16); BodyRotNew = Structured(nb, 16);
            BodyInitialLin = Structured(nb, 16); BodyInitialAng = Structured(nb, 16);
            BodyInertialLin = Structured(nb, 16); BodyInertialAng = Structured(nb, 16);
            BodyVelLin = Structured(nb, 16); BodyVelAng = Structured(nb, 16); BodyPrevVelLin = Structured(nb, 16);
            BodyAabbMin = Structured(nb, 16); BodyAabbMax = Structured(nb, 16);
            BodyColor = Structured(nb, 4); BodyColorTmp = Structured(nb, 4);
            BodyDrive = Structured(nb, GpuBodyDrive.Stride); BodyEvents = Structured(nb, 4);
            SpawnRecords = Structured(math.max(config.MaxSpawns, 1), GpuSpawnRecord.Stride);
            BodySleep = Structured(nb, 4); BodyLabel = Structured(nb, 4); BodyLabelTmp = Structured(nb, 4);
            RestMin = Structured(nb, 4); RestMinTmp = Structured(nb, 4); WakeMark = Structured(nb, 4); BodyRestPose = Structured(nb, 32);
            WakeList = Structured(math.max(config.MaxWakes, 1), 8);
            HotFlags = Structured(nb, 4); HotScan = Structured(nb + 1, 4); HotList = Structured(nb, 4); WokenList = Structured(nb, 4);
            ActiveJoints = Structured(config.MaxJoints, 4); ActiveSprings = Structured(config.MaxSprings, 4);
            SleepFlags = Structured(nb, 4); SleepScan = Structured(nb + 1, 4); SleepList = Structured(nb, 4);
            SleepCellCount = Structured(config.SleepCellCount, 4); SleepCellStart = Structured(config.SleepCellCount + 1, 4);
            SleepCellCursor = Structured(config.SleepCellCount, 4); SleepCellEntries = Structured(config.MaxSleepCellEntries, 4);
            BodyGen = Structured(nb, 4);
            ColdManifolds = Structured(config.MaxColdManifolds, GpuManifold.Stride); ColdContacts = Structured(config.MaxColdContacts, GpuContact.Stride);
            ColdHash = Structured(config.ColdHashSize, 4);
            FreezeFlag = Structured(config.MaxManifolds, 4); FreezeContacts = Structured(config.MaxManifolds, 4);
            FreezeFlagScan = Structured(config.MaxManifolds + 1, 4); FreezeContactScan = Structured(config.MaxManifolds + 1, 4);
            ColdFlag = Structured(config.MaxColdManifolds, 4); ColdContactCount = Structured(config.MaxColdManifolds, 4);
            ColdFlagScan = Structured(config.MaxColdManifolds + 1, 4); ColdContactScan = Structured(config.MaxColdManifolds + 1, 4);
            ColdScratch = Structured(AvbdGpuConfig.ColdChunk, GpuManifold.Stride); ColdScratchContacts = Structured(AvbdGpuConfig.ColdChunk * 8, GpuContact.Stride);
            LinkStart = Structured(nb + 1, 4); LinkList = Structured(math.max(config.MaxLinks, 1), 8);
            CellCount = Structured(config.CellCount, 4); CellStart = Structured(config.CellCount + 1, 4); CellCursor = Structured(config.CellCount, 4);
            CellEntries = Structured(config.MaxCellEntries, 4); LargeBodies = Structured(config.MaxLargeBodies, 4);
            BlockSums = Structured(1024, 4);
            Pairs = Structured(config.MaxPairs, 8);
            ManifoldPrev = Copyable(config.MaxManifolds, GpuManifold.Stride); ManifoldCur = Copyable(config.MaxManifolds, GpuManifold.Stride);
            ContactsPrev = Copyable(config.MaxContacts, GpuContact.Stride); ContactsCur = Copyable(config.MaxContacts, GpuContact.Stride);
            HashPrev = Copyable(config.HashSize, 4); HashCur = Copyable(config.HashSize, 4);
            JointDef = Structured(config.MaxJoints, GpuJointDef.Stride); JointState = Structured(config.MaxJoints, GpuJointState.Stride);
            SpringDef = Structured(config.MaxSprings, GpuSpringDef.Stride);
            BodyConsCount = Structured(nb, 4); BodyConsCursor = Structured(nb, 4); BodyConsStart = Structured(nb + 1, 4);
            BodyConsList = Structured(config.MaxConstraintRefs, 4);
            ColorCount = Structured(AvbdGpuConstants.MaxColors + 1, 4); ColorStart = Structured(AvbdGpuConstants.MaxColors + 2, 4);
            ColorCursor = Structured(AvbdGpuConstants.MaxColors + 1, 4); ColorList = Structured(nb, 4);
            TerrainHeights = Structured(1, 4); TerrainMaxMip = Structured(1, 4);
            Counters = Structured((int)StatSlot.Count, 4); Stats = Structured((int)StatSlot.Count, 4);
            DispatchArgs = Track(new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments | GraphicsBuffer.Target.Raw, ArgSlots * 3, 4));
            Params = Track(new GraphicsBuffer(GraphicsBuffer.Target.Constant, 1, GpuParams.Stride));
            ClearAll();
        }

        GraphicsBuffer Structured(int count, int stride) => Track(new GraphicsBuffer(GraphicsBuffer.Target.Structured, math.max(count, 1), stride));

        /// <summary>Structured buffer that CommandBuffer.CopyBuffer can copy from and to (the prev/cur manifold state).</summary>
        GraphicsBuffer Copyable(int count, int stride) => Track(new GraphicsBuffer(GraphicsBuffer.Target.Structured | GraphicsBuffer.Target.CopySource | GraphicsBuffer.Target.CopyDestination, math.max(count, 1), stride));

        GraphicsBuffer Track(GraphicsBuffer b) { m_All.Add(b); return b; }

        /// <summary>Makes the terrain buffers hold at least the given sample and mip block counts (reallocating: the pipeline
        /// re-binds when <see cref="TerrainVersion"/> changes).</summary>
        public void EnsureTerrain(int samples, int blocks)
        {
            int oldSamples = TerrainHeights.count, oldBlocks = TerrainMaxMip.count;
            if (oldSamples >= samples && oldBlocks >= blocks) return;
            m_All.Remove(TerrainHeights); TerrainHeights.Dispose();
            m_All.Remove(TerrainMaxMip); TerrainMaxMip.Dispose();
            TerrainHeights = Structured(math.max(samples, oldSamples), 4);
            TerrainMaxMip = Structured(math.max(blocks, oldBlocks), 4);
            TerrainVersion++;
        }

        /// <summary>Zero every buffer (fresh GPU memory is undefined; the solver relies on zeroed counts, colours and hash tables).</summary>
        public void ClearAll()
        {
            int maxBytes = 0;
            foreach (var b in m_All) maxBytes = math.max(maxBytes, b.count * b.stride);
            var zeros = new byte[maxBytes];
            foreach (var b in m_All)
            {
                if (b == Params) continue;
                b.SetData(zeros, 0, 0, b.count * b.stride);
            }
        }

        /// <summary>Zero one buffer (scene reset of the hash tables and colours).</summary>
        public static void Clear(GraphicsBuffer b)
        {
            b.SetData(new byte[b.count * b.stride]);
        }

        public void Dispose()
        {
            foreach (var b in m_All) b?.Dispose();
            m_All.Clear();
        }
    }
}
