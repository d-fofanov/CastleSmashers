using System;
using System.Collections.Generic;
using System.Diagnostics;
using Phys.AvbdGpu.Scenes;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace Phys.AvbdGpu
{
    /// <summary>Solver parameters (defaults = avbd-demo3d).</summary>
    [Serializable]
    public struct AvbdGpuParams
    {
        public float Dt;
        public float3 Gravity;
        public int Iterations;
        public int Substeps;
        public float Alpha;
        public float BetaLin;
        public float BetaAng;
        public float Gamma;
        /// <summary>Extra position-only iteration with alpha 0 (2D reference option); off = 3D reference behaviour.</summary>
        public bool PostStabilize;
        /// <summary>Use the rotated inertia R I R^T (paper Eq. 8) instead of the body-frame diagonal (reference).</summary>
        public bool RotatedInertia;
        /// <summary>Grid cell size; 0 = automatic (twice the mean extent of the dynamic bodies).</summary>
        public float CellSize;
        /// <summary>Jones-Plassmann colouring rounds per step (rounded up to even).</summary>
        public int ColorRounds;

        public static AvbdGpuParams Default => new AvbdGpuParams
        {
            Dt = 1f / 60f, Gravity = new float3(0, -10f, 0), Iterations = 10, Substeps = 1,
            Alpha = 0.99f, BetaLin = 10000f, BetaAng = 100f, Gamma = 0.999f,
            PostStabilize = false, RotatedInertia = false, CellSize = 0f, ColorRounds = 8,
        };
    }

    /// <summary>A GPU AVBD world: bodies, joints, springs and collision links are created on the CPU and uploaded once;
    /// the whole step then runs on the GPU. Body indices are creation order (also the ISceneBuilder ids).</summary>
    public sealed class AvbdGpuWorld : ISceneBuilder, IDisposable
    {
        public readonly AvbdGpuConfig Config;
        public readonly AvbdGpuBuffers Buffers;
        public readonly AvbdGpuKernels Kernels;
        public AvbdGpuParams Params = AvbdGpuParams.Default;

        readonly AvbdGpuPipeline m_Pipeline;

        // CPU staging (capacity sized)
        readonly GpuBodyDef[] m_BodyDefs;
        readonly float4[] m_Pos, m_Rot, m_Vel;
        readonly uint[] m_Uncolored;
        int m_BodyCount, m_UploadedBodies;
        double m_ExtentSum; int m_ExtentCount;

        readonly GpuJointDef[] m_JointDefs;
        readonly GpuJointState[] m_ZeroJointStates;
        int m_JointCount, m_UploadedJoints;
        readonly List<int> m_FreeJoints = new List<int>();
        readonly HashSet<int> m_DirtyJoints = new HashSet<int>();

        readonly GpuSpringDef[] m_SpringDefs;
        int m_SpringCount, m_UploadedSprings;

        readonly List<(int a, int b, uint r)> m_Links = new List<(int, int, uint)>();
        bool m_LinksDirty;
        readonly uint[] m_LinkStart;
        readonly uint2[] m_LinkList;

        GpuParams m_GpuParams;
        readonly GpuParams[] m_GpuParamsArray = new GpuParams[1];

        // stats and readback
        AvbdGpuStats m_Stats;
        int m_ActiveColors = 8;
        int m_ShrinkCounter;
        readonly Stopwatch m_Watch = new Stopwatch();
        double m_StepMsSum; int m_StepCount;
        public bool ReadbackPoses;
        float4[] m_PosRead, m_RotRead;
        int m_PosReadCount;
        bool m_PoseRequestPending, m_StatsRequestPending;

        public int BodyCount => m_BodyCount;
        public int JointCount => m_JointCount;
        public int SpringCount => m_SpringCount;
        public AvbdGpuStats Stats => m_Stats;
        public int ActiveColors => m_ActiveColors;
        public int StepIndex { get; private set; }

        public AvbdGpuWorld(AvbdGpuConfig config)
        {
            if (!AvbdGpuKernels.Supported) throw new NotSupportedException("Compute shaders are not supported on this device");
            Config = config;
            Kernels = new AvbdGpuKernels();
            Buffers = new AvbdGpuBuffers(config);
            m_Pipeline = new AvbdGpuPipeline(Kernels, Buffers);

            m_BodyDefs = new GpuBodyDef[config.MaxBodies];
            m_Pos = new float4[config.MaxBodies];
            m_Rot = new float4[config.MaxBodies];
            m_Vel = new float4[config.MaxBodies];
            m_Uncolored = new uint[config.MaxBodies];
            for (int i = 0; i < m_Uncolored.Length; i++) m_Uncolored[i] = 0xFFFFFFFFu;
            m_JointDefs = new GpuJointDef[config.MaxJoints];
            m_ZeroJointStates = new GpuJointState[config.MaxJoints];
            m_SpringDefs = new GpuSpringDef[config.MaxSprings];
            m_LinkStart = new uint[config.MaxBodies + 1];
            m_LinkList = new uint2[math.max(config.MaxLinks, 1)];
        }

        public AvbdGpuWorld() : this(AvbdGpuConfig.Default) { }

        public void Dispose()
        {
            m_Pipeline?.Dispose();
            Buffers?.Dispose();
        }

        // ------------------------------------------------------------------------------------------------ scene construction

        public int AddBody(float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity)
        {
            if (m_BodyCount >= Config.MaxBodies) throw new InvalidOperationException($"AvbdGpuWorld: body capacity {Config.MaxBodies} exceeded");
            float mass = size.x * size.y * size.z * density;
            var def = new GpuBodyDef
            {
                Size = size,
                Mass = mass,
                Moment = new float3(
                    (size.y * size.y + size.z * size.z) / 12.0f * mass,
                    (size.x * size.x + size.z * size.z) / 12.0f * mass,
                    (size.x * size.x + size.y * size.y) / 12.0f * mass),
                Friction = friction,
                Radius = math.length(size * 0.5f),
                Flags = mass <= 0f ? GpuBodyDef.FlagStatic : 0u,
            };
            int i = m_BodyCount++;
            m_BodyDefs[i] = def;
            m_Pos[i] = new float4(position, 0);
            m_Rot[i] = rotation.value;
            m_Vel[i] = new float4(velocity, 0);
            if (mass > 0f) { m_ExtentSum += math.cmax(size); m_ExtentCount++; }
            return i;
        }

        public void AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture)
        {
            AddJointIndexed(bodyA, bodyB, rA, rB, stiffnessLin, stiffnessAng, fracture);
        }

        /// <summary>Adds a joint and returns its index (needed to move or remove it later).</summary>
        public int AddJointIndexed(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin = float.PositiveInfinity, float stiffnessAng = 0f, float fracture = float.PositiveInfinity)
        {
            float3 sizeA = bodyA >= 0 ? m_BodyDefs[bodyA].Size : float3.zero;
            var def = new GpuJointDef
            {
                BodyA = bodyA, BodyB = bodyB, RA = rA, RB = rB,
                StiffnessLin = AvbdGpuConstants.ToGpuStiffness(stiffnessLin),
                StiffnessAng = AvbdGpuConstants.ToGpuStiffness(stiffnessAng),
                Fracture = AvbdGpuConstants.ToGpuStiffness(fracture),
                TorqueArm = math.lengthsq(sizeA + m_BodyDefs[bodyB].Size),
            };
            int j;
            if (m_FreeJoints.Count > 0)
            {
                j = m_FreeJoints[m_FreeJoints.Count - 1];
                m_FreeJoints.RemoveAt(m_FreeJoints.Count - 1);
                m_DirtyJoints.Add(j);
            }
            else
            {
                if (m_JointCount >= Config.MaxJoints) throw new InvalidOperationException($"AvbdGpuWorld: joint capacity {Config.MaxJoints} exceeded");
                j = m_JointCount++;
            }
            m_JointDefs[j] = def;
            if (bodyA >= 0) AddLink(bodyA, bodyB, ConsRef.Make(ConsRef.Joint, (uint)j));
            return j;
        }

        /// <summary>Moves the world anchor (bodyA = -1) or the local anchor of a joint.</summary>
        public void SetJointAnchor(int joint, float3 rA)
        {
            m_JointDefs[joint].RA = rA;
            if (joint < m_UploadedJoints) m_DirtyJoints.Add(joint);
        }

        /// <summary>Disables a joint (its slot is reused by the next AddJoint).</summary>
        public void RemoveJoint(int joint)
        {
            var def = m_JointDefs[joint];
            m_JointDefs[joint] = new GpuJointDef { BodyA = -1, BodyB = 0, StiffnessLin = 0, StiffnessAng = 0, Fracture = AvbdGpuConstants.HardStiffness };
            if (def.BodyA >= 0) RemoveLink(def.BodyA, def.BodyB, ConsRef.Make(ConsRef.Joint, (uint)joint));
            if (joint < m_UploadedJoints) m_DirtyJoints.Add(joint);
            m_FreeJoints.Add(joint);
        }

        public void AddSpring(int bodyA, int bodyB, float3 rA, float3 rB, float stiffness, float rest)
        {
            if (m_SpringCount >= Config.MaxSprings) throw new InvalidOperationException($"AvbdGpuWorld: spring capacity {Config.MaxSprings} exceeded");
            if (rest < 0f)
            {
                float3 pA = math.mul(new quaternion(m_Rot[bodyA]), rA) + m_Pos[bodyA].xyz;
                float3 pB = math.mul(new quaternion(m_Rot[bodyB]), rB) + m_Pos[bodyB].xyz;
                rest = math.length(pA - pB);
            }
            int s = m_SpringCount++;
            m_SpringDefs[s] = new GpuSpringDef { BodyA = bodyA, BodyB = bodyB, RA = rA, RB = rB, Stiffness = stiffness, Rest = rest };
            AddLink(bodyA, bodyB, ConsRef.Make(ConsRef.Spring, (uint)s));
        }

        public void AddIgnoreCollision(int bodyA, int bodyB)
        {
            AddLink(bodyA, bodyB, ConsRef.Make(ConsRef.Ignore, 0));
        }

        void AddLink(int a, int b, uint r)
        {
            m_Links.Add((a, b, r));
            m_Links.Add((b, a, r));
            m_LinksDirty = true;
        }

        void RemoveLink(int a, int b, uint r)
        {
            m_Links.RemoveAll(l => l.r == r && ((l.a == a && l.b == b) || (l.a == b && l.b == a)));
            m_LinksDirty = true;
        }

        /// <summary>Removes everything (GPU buffers keep their allocation).</summary>
        public void Clear()
        {
            m_BodyCount = m_UploadedBodies = 0;
            m_JointCount = m_UploadedJoints = 0;
            m_SpringCount = m_UploadedSprings = 0;
            m_FreeJoints.Clear();
            m_DirtyJoints.Clear();
            m_Links.Clear();
            m_LinksDirty = true;
            m_ExtentSum = 0; m_ExtentCount = 0;
            m_StepMsSum = 0; m_StepCount = 0;
            m_Stats = default;
            StepIndex = 0;
            AvbdGpuBuffers.Clear(Buffers.HashPrev);
            AvbdGpuBuffers.Clear(Buffers.HashCur);
            AvbdGpuBuffers.Clear(Buffers.BodyConsCount);
            AvbdGpuBuffers.Clear(Buffers.LinkStart);
        }

        public void BuildScene(int scene)
        {
            Clear();
            AvbdScenes.Build(this, scene);
        }

        // ------------------------------------------------------------------------------------------------ upload

        /// <summary>Uploads everything created since the last step. Called by Step(); public for tests.</summary>
        public void Upload()
        {
            var b = Buffers;
            if (m_UploadedBodies < m_BodyCount)
            {
                int start = m_UploadedBodies, count = m_BodyCount - start;
                b.BodyDef.SetData(m_BodyDefs, start, start, count);
                b.BodyPos.SetData(m_Pos, start, start, count);
                b.BodyRot.SetData(m_Rot, start, start, count);
                b.BodyVelLin.SetData(m_Vel, start, start, count);
                b.BodyPrevVelLin.SetData(m_Vel, start, start, count);
                b.BodyVelAng.SetData(new float4[count], 0, start, count);
                b.BodyColor.SetData(m_Uncolored, start, start, count);
                m_UploadedBodies = m_BodyCount;
            }
            if (m_UploadedJoints < m_JointCount)
            {
                int start = m_UploadedJoints, count = m_JointCount - start;
                b.JointDef.SetData(m_JointDefs, start, start, count);
                b.JointState.SetData(m_ZeroJointStates, start, start, count);
                m_UploadedJoints = m_JointCount;
            }
            if (m_DirtyJoints.Count > 0)
            {
                foreach (int j in m_DirtyJoints)
                {
                    if (j >= m_UploadedJoints) continue;
                    b.JointDef.SetData(m_JointDefs, j, j, 1);
                    b.JointState.SetData(m_ZeroJointStates, j, j, 1);
                }
                m_DirtyJoints.Clear();
            }
            if (m_UploadedSprings < m_SpringCount)
            {
                int start = m_UploadedSprings, count = m_SpringCount - start;
                b.SpringDef.SetData(m_SpringDefs, start, start, count);
                m_UploadedSprings = m_SpringCount;
            }
            if (m_LinksDirty)
            {
                UploadLinks();
                m_LinksDirty = false;
            }
        }

        void UploadLinks()
        {
            m_Links.Sort((x, y) => x.a != y.a ? x.a.CompareTo(y.a) : x.b != y.b ? x.b.CompareTo(y.b) : x.r.CompareTo(y.r));
            int n = math.min(m_Links.Count, m_LinkList.Length);
            if (m_Links.Count > m_LinkList.Length) UnityEngine.Debug.LogError($"AvbdGpuWorld: link capacity {m_LinkList.Length} exceeded ({m_Links.Count})");
            int cursor = 0;
            for (int body = 0; body <= Config.MaxBodies; body++)
            {
                m_LinkStart[body] = (uint)cursor;
                while (cursor < n && m_Links[cursor].a == body) { m_LinkList[cursor] = new uint2((uint)m_Links[cursor].b, m_Links[cursor].r); cursor++; }
            }
            Buffers.LinkStart.SetData(m_LinkStart);
            if (n > 0) Buffers.LinkList.SetData(m_LinkList, 0, 0, n);
        }

        float AutoCellSize()
        {
            if (Params.CellSize > 0f) return Params.CellSize;
            float mean = m_ExtentCount > 0 ? (float)(m_ExtentSum / m_ExtentCount) : 1f;
            return math.clamp(2f * mean, 0.05f, 1000f);
        }

        void FillParams(float dt)
        {
            var p = Params;
            var c = Config;
            float gMag = math.length(p.Gravity);
            float cell = AutoCellSize();
            m_GpuParams = new GpuParams
            {
                Dt = dt, Alpha = p.Alpha, BetaLin = p.BetaLin, BetaAng = p.BetaAng,
                Gamma = p.Gamma, LambdaDecay = p.PostStabilize ? 1f : p.Alpha * p.Gamma, CellSize = cell, InvCellSize = 1f / cell,
                Gravity = p.Gravity, GravityMag = gMag,
                GravityDir = gMag > 0f ? p.Gravity / gMag : float3.zero, BodyCount = (uint)m_BodyCount,
                JointCount = (uint)m_JointCount, SpringCount = (uint)m_SpringCount, ActiveColors = (uint)m_ActiveColors, RotatedInertia = p.RotatedInertia ? 1u : 0u,
                HashMask = (uint)(c.HashSize - 1), CellMask = (uint)(c.CellCount - 1), MaxPairs = (uint)c.MaxPairs, MaxManifolds = (uint)c.MaxManifolds,
                MaxContacts = (uint)c.MaxContacts, MaxCellEntries = (uint)c.MaxCellEntries, MaxLargeBodies = (uint)c.MaxLargeBodies, LargeBodyCells = (uint)c.LargeBodyCells,
                ColorRounds = (uint)p.ColorRounds, Substep = 0,
            };
            m_GpuParamsArray[0] = m_GpuParams;
            Buffers.Params.SetData(m_GpuParamsArray);
        }

        // ------------------------------------------------------------------------------------------------ stepping

        /// <summary>Advances the world by Params.Dt (Params.Substeps substeps). Nothing is read back synchronously.</summary>
        public void Step()
        {
            Upload();
            int substeps = math.max(1, Params.Substeps);
            FillParams(Params.Dt / substeps);
            m_Pipeline.Ensure(math.max(1, Params.Iterations), m_ActiveColors, Params.PostStabilize, math.max(2, Params.ColorRounds), Params.Alpha);

            m_Watch.Restart();
            for (int s = 0; s < substeps; s++)
                Graphics.ExecuteCommandBuffer(m_Pipeline.CommandBuffer);
            m_Watch.Stop();
            double ms = m_Watch.Elapsed.TotalMilliseconds;
            m_StepMsSum += ms; m_StepCount++;
            m_Stats.LastStepMs = (float)ms;
            m_Stats.AvgStepMs = (float)(m_StepMsSum / m_StepCount);
            m_Stats.MaxStepMs = math.max(m_Stats.MaxStepMs, (float)ms);
            StepIndex++;

            if (!m_StatsRequestPending)
            {
                m_StatsRequestPending = true;
                AsyncGPUReadback.Request(Buffers.Stats, OnStats);
            }
            if (ReadbackPoses && !m_PoseRequestPending && m_BodyCount > 0)
            {
                m_PoseRequestPending = true;
                int count = m_BodyCount;
                AsyncGPUReadback.Request(Buffers.BodyPos, count * 16, 0, r => OnPoses(r, count, true));
                AsyncGPUReadback.Request(Buffers.BodyRot, count * 16, 0, r => OnPoses(r, count, false));
            }
        }

        void OnStats(AsyncGPUReadbackRequest r)
        {
            m_StatsRequestPending = false;
            if (r.hasError) return;
            var data = r.GetData<uint>();
            m_Stats.Pairs = (int)data[(int)StatSlot.Pairs];
            m_Stats.Manifolds = (int)data[(int)StatSlot.Manifolds];
            m_Stats.Contacts = (int)data[(int)StatSlot.Contacts];
            m_Stats.LargeBodies = (int)data[(int)StatSlot.LargeBodies];
            m_Stats.OverflowFlags = (int)data[(int)StatSlot.Overflow];
            m_Stats.OverflowBodies = (int)data[(int)StatSlot.OverflowBodies];
            m_Stats.ColorsUsed = (int)data[(int)StatSlot.ColorsUsed];
            m_Stats.Constraints = (int)data[(int)StatSlot.Constraints];
            m_Stats.ActiveColors = m_ActiveColors;
            m_Stats.Frame++;
            AdaptColors();
        }

        void AdaptColors()
        {
            if (m_Stats.OverflowBodies > 0 && m_Stats.ColorsUsed >= m_ActiveColors && m_ActiveColors < AvbdGpuConstants.MaxColors)
            {
                m_ActiveColors = math.min(AvbdGpuConstants.MaxColors, m_ActiveColors + 4);
                m_ShrinkCounter = 0;
            }
            else if (m_Stats.ColorsUsed + 3 < m_ActiveColors && m_ActiveColors > 4)
            {
                if (++m_ShrinkCounter > 120) { m_ActiveColors--; m_ShrinkCounter = 0; }
            }
            else m_ShrinkCounter = 0;
        }

        void OnPoses(AsyncGPUReadbackRequest r, int count, bool positions)
        {
            if (!positions) m_PoseRequestPending = false;
            if (r.hasError) return;
            var data = r.GetData<float4>();
            if (positions)
            {
                if (m_PosRead == null || m_PosRead.Length < count) m_PosRead = new float4[math.max(count, 1024)];
                NativeArray<float4>.Copy(data, m_PosRead, count);
            }
            else
            {
                if (m_RotRead == null || m_RotRead.Length < count) m_RotRead = new float4[math.max(count, 1024)];
                NativeArray<float4>.Copy(data, m_RotRead, count);
                m_PosReadCount = count;
            }
        }

        /// <summary>Poses from the last asynchronous readback (one or two frames old); null until the first arrives.</summary>
        public float4[] ReadPositions => m_PosRead;
        public float4[] ReadRotations => m_RotRead;
        public int ReadCount => m_PosReadCount;

        /// <summary>Synchronous readback of the body poses (tests, tools).</summary>
        public void GetPosesSync(out float4[] positions, out float4[] rotations)
        {
            positions = new float4[m_BodyCount];
            rotations = new float4[m_BodyCount];
            if (m_BodyCount == 0) return;
            Buffers.BodyPos.GetData(positions, 0, 0, m_BodyCount);
            Buffers.BodyRot.GetData(rotations, 0, 0, m_BodyCount);
        }

        public void GetVelocitiesSync(out float4[] linear, out float4[] angular)
        {
            linear = new float4[m_BodyCount];
            angular = new float4[m_BodyCount];
            if (m_BodyCount == 0) return;
            Buffers.BodyVelLin.GetData(linear, 0, 0, m_BodyCount);
            Buffers.BodyVelAng.GetData(angular, 0, 0, m_BodyCount);
        }

        /// <summary>Synchronous stats readback (tests).</summary>
        public AvbdGpuStats GetStatsSync()
        {
            var data = new uint[(int)StatSlot.Count];
            Buffers.Stats.GetData(data);
            var s = m_Stats;
            s.Pairs = (int)data[(int)StatSlot.Pairs];
            s.Manifolds = (int)data[(int)StatSlot.Manifolds];
            s.Contacts = (int)data[(int)StatSlot.Contacts];
            s.LargeBodies = (int)data[(int)StatSlot.LargeBodies];
            s.OverflowFlags = (int)data[(int)StatSlot.Overflow];
            s.OverflowBodies = (int)data[(int)StatSlot.OverflowBodies];
            s.ColorsUsed = (int)data[(int)StatSlot.ColorsUsed];
            s.Constraints = (int)data[(int)StatSlot.Constraints];
            s.ActiveColors = m_ActiveColors;
            m_Stats = s;
            return s;
        }

        /// <summary>Forces the active colour count (tests / benchmarks).</summary>
        public void SetActiveColors(int n) => m_ActiveColors = math.clamp(n, 1, AvbdGpuConstants.MaxColors);

        public GpuBodyDef GetBodyDef(int i) => m_BodyDefs[i];
        public GpuJointDef GetJointDef(int j) => m_JointDefs[j];

        public GpuJointState[] GetJointStatesSync()
        {
            var arr = new GpuJointState[math.max(m_JointCount, 1)];
            if (m_JointCount > 0) Buffers.JointState.GetData(arr, 0, 0, m_JointCount);
            return arr;
        }

        public GpuManifold[] GetManifoldsSync(out GpuContact[] contacts)
        {
            var stats = GetStatsSync();
            var m = new GpuManifold[math.max(stats.Manifolds, 1)];
            contacts = new GpuContact[math.max(stats.Contacts, 1)];
            if (stats.Manifolds > 0) Buffers.ManifoldPrev.GetData(m, 0, 0, stats.Manifolds);
            if (stats.Contacts > 0) Buffers.ContactsPrev.GetData(contacts, 0, 0, stats.Contacts);
            return m;
        }

        // ------------------------------------------------------------------------------------------------ picking

        /// <summary>Ray vs the dynamic OBBs of the last read-back poses (Solver::pick): returns the body or -1; local is the hit in body space.</summary>
        public int Pick(float3 origin, float3 dir, out float3 local, out float distance)
        {
            const float epsilon = 1.0e-6f;
            float bestT = float.PositiveInfinity;
            int best = -1;
            local = float3.zero;
            distance = 0f;
            if (m_PosRead == null || m_RotRead == null) return -1;
            int n = math.min(m_PosReadCount, m_BodyCount);
            for (int i = 0; i < n; i++)
            {
                if (m_BodyDefs[i].Mass <= 0f) continue;
                quaternion invRot = math.conjugate(new quaternion(m_RotRead[i]));
                float3 o = math.mul(invRot, origin - m_PosRead[i].xyz);
                float3 d = math.mul(invRot, dir);
                float3 half = m_BodyDefs[i].Size * 0.5f;
                float tEnter = 0f, tExit = float.PositiveInfinity;
                bool hit = true;
                for (int k = 0; k < 3; k++)
                {
                    if (math.abs(d[k]) < epsilon)
                    {
                        if (o[k] < -half[k] || o[k] > half[k]) { hit = false; break; }
                        continue;
                    }
                    float invD = 1f / d[k];
                    float t0 = (-half[k] - o[k]) * invD, t1 = (half[k] - o[k]) * invD;
                    if (t0 > t1) (t0, t1) = (t1, t0);
                    tEnter = math.max(tEnter, t0);
                    tExit = math.min(tExit, t1);
                    if (tEnter > tExit) { hit = false; break; }
                }
                if (!hit) continue;
                float tHit = tEnter >= 0f ? tEnter : tExit;
                if (tHit < 0f || tHit >= bestT) continue;
                bestT = tHit;
                best = i;
                local = o + d * tHit;
            }
            distance = bestT;
            return best;
        }
    }
}
