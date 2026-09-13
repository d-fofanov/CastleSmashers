using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu
{
    /// <summary>Buffer capacities. Everything is allocated once; appends beyond a capacity are dropped and reported in the stats.</summary>
    [Serializable]
    public struct AvbdGpuConfig
    {
        public int MaxBodies;
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
        public int CellCount;         // power of two

        public static AvbdGpuConfig ForBodies(int bodies)
        {
            bodies = math.max(bodies, 1024);
            int manifolds = bodies * 4;
            return new AvbdGpuConfig
            {
                MaxBodies = bodies,
                MaxJoints = math.max(bodies, 4096),
                MaxSprings = math.max(bodies / 4, 1024),
                MaxLinks = math.max(bodies * 4, 16384),
                MaxManifolds = manifolds,
                MaxContacts = manifolds * 4,
                MaxPairs = manifolds,
                MaxCellEntries = bodies * 8,
                MaxLargeBodies = 256,
                LargeBodyCells = 64,
                HashSize = math.ceilpow2(manifolds * 2),
                CellCount = math.ceilpow2(bodies * 2),
            };
        }

        public static AvbdGpuConfig Default => ForBodies(65536);

        public int MaxConstraintRefs => 2 * MaxManifolds + 2 * (MaxJoints + MaxSprings);

        public long EstimatedBytes =>
            (long)MaxBodies * (GpuBodyDef.Stride + 16 * 14 + 8) + (long)MaxLinks * 8 +
            (long)CellCount * 12 + 4 + (long)MaxCellEntries * 4 + (long)MaxPairs * 8 +
            2L * MaxManifolds * GpuManifold.Stride + 2L * MaxContacts * GpuContact.Stride + 2L * HashSize * 4 +
            (long)MaxJoints * (GpuJointDef.Stride + GpuJointState.Stride) + (long)MaxSprings * GpuSpringDef.Stride +
            (long)MaxConstraintRefs * 4;
    }

    /// <summary>All GPU buffers of a world. Names match the HLSL declarations (used for binding).</summary>
    public sealed class AvbdGpuBuffers : IDisposable
    {
        public readonly AvbdGpuConfig Config;
        readonly List<GraphicsBuffer> m_All = new List<GraphicsBuffer>();

        // bodies
        public GraphicsBuffer BodyDef, BodyPos, BodyRot, BodyPosNew, BodyRotNew, BodyInitialLin, BodyInitialAng, BodyInertialLin, BodyInertialAng,
            BodyVelLin, BodyVelAng, BodyPrevVelLin, BodyAabbMin, BodyAabbMax, BodyColor, BodyColorTmp;
        // links
        public GraphicsBuffer LinkStart, LinkList;
        // grid
        public GraphicsBuffer CellCount, CellStart, CellCursor, CellEntries, LargeBodies, BlockSums;
        // pairs and manifolds
        public GraphicsBuffer Pairs, ManifoldPrev, ManifoldCur, ContactsPrev, ContactsCur, HashPrev, HashCur;
        // joints and springs
        public GraphicsBuffer JointDef, JointState, SpringDef;
        // CSR
        public GraphicsBuffer BodyConsCount, BodyConsCursor, BodyConsStart, BodyConsList;
        // colours
        public GraphicsBuffer ColorCount, ColorStart, ColorCursor, ColorList;
        // misc
        public GraphicsBuffer Counters, Stats, DispatchArgs, Params;

        public const int ArgSlots = 8 + AvbdGpuConstants.MaxColors + 1;

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
            Counters = Structured((int)StatSlot.Count, 4); Stats = Structured((int)StatSlot.Count, 4);
            DispatchArgs = Track(new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments | GraphicsBuffer.Target.Raw, ArgSlots * 3, 4));
            Params = Track(new GraphicsBuffer(GraphicsBuffer.Target.Constant, 1, GpuParams.Stride));
            ClearAll();
        }

        GraphicsBuffer Structured(int count, int stride) => Track(new GraphicsBuffer(GraphicsBuffer.Target.Structured, math.max(count, 1), stride));

        /// <summary>Structured buffer that CommandBuffer.CopyBuffer can copy from and to (the prev/cur manifold state).</summary>
        GraphicsBuffer Copyable(int count, int stride) => Track(new GraphicsBuffer(GraphicsBuffer.Target.Structured | GraphicsBuffer.Target.CopySource | GraphicsBuffer.Target.CopyDestination, math.max(count, 1), stride));

        GraphicsBuffer Track(GraphicsBuffer b) { m_All.Add(b); return b; }

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
