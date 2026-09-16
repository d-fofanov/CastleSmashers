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
        /// <summary>Sleeping: a body that has stayed within <see cref="SleepDistance"/> / <see cref="SleepAngle"/> of its rest pose for
        /// <see cref="SleepTime"/> seconds, with every body within <see cref="SleepHops"/> contacts or joints of it resting as well, is
        /// frozen until something touches or changes it. Off: every body is simulated every step (the reference behaviour).</summary>
        public bool Sleep;
        public float SleepTime;
        public float SleepDistance;
        public float SleepAngle;
        /// <summary>How far (in contacts / joints) the neighbourhood must rest before a body sleeps.</summary>
        public int SleepHops;
        /// <summary>A touch by an awake body faster than this wakes the whole island of the touched body (an impact, a projectile, a
        /// walking unit) for at least <see cref="WakeHold"/> seconds; a slower one wakes the touched body alone (it sleeps again as
        /// soon as it and its neighbourhood rest), and a resting one wakes nothing.</summary>
        public float WakeSpeed;
        public float WakeHold;
        /// <summary>Island label propagation rounds per step (each doubles the reach; rounded up to even).</summary>
        public int SleepLabelRounds;
        /// <summary>Every this many steps the awake bodies restart their island labels, so that bodies which came apart stop
        /// sharing an island.</summary>
        public int SleepRelabelSteps;

        public static AvbdGpuParams Default => new AvbdGpuParams
        {
            Dt = 1f / 60f, Gravity = new float3(0, -10f, 0), Iterations = 10, Substeps = 1,
            Alpha = 0.99f, BetaLin = 10000f, BetaAng = 100f, Gamma = 0.999f,
            PostStabilize = false, RotatedInertia = false, CellSize = 0f, ColorRounds = 8,
            Sleep = true, SleepTime = 0.5f, SleepDistance = 0.02f, SleepAngle = 0.02f, SleepHops = 2, WakeSpeed = 0.25f, WakeHold = 0.15f,
            SleepLabelRounds = 4, SleepRelabelSteps = 60,
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
        readonly GpuBodyDrive[] m_Drives;
        readonly float4[] m_Pos, m_Rot, m_Vel;
        readonly uint[] m_Uncolored, m_ZeroUints, m_Identity;
        int m_BodyCount, m_UploadedBodies;
        double m_ExtentSum; int m_ExtentCount;
        // definitions and drives are CPU-owned: changes to uploaded bodies are re-sent as one range each
        int m_DirtyDefMin = int.MaxValue, m_DirtyDefMax = -1, m_DirtyDriveMin = int.MaxValue, m_DirtyDriveMax = -1;
        readonly List<GpuSpawnRecord> m_SpawnQueue = new List<GpuSpawnRecord>();
        readonly GpuSpawnRecord[] m_SpawnChunk;
        static readonly int s_SpawnCount = Shader.PropertyToID("_SpawnCount");
        // sleeping: the wake requests of this step (applied by the WakeList kernel), or everything at once
        readonly List<uint2> m_WakeList = new List<uint2>();
        readonly uint2[] m_WakeArray;
        bool m_WakeAll, m_SleepWasOn;
        float3 m_LastGravity;

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
        int m_PosReadCount, m_PosReadStep = -1, m_PosRequestStep;
        bool m_PoseRequestPending, m_StatsRequestPending;
        /// <summary>Body ranges whose event words are read back asynchronously every step (the unit and projectile pools).</summary>
        public readonly List<(int start, int count)> EventRanges = new List<(int, int)>();
        uint[] m_EventsRead;
        int m_EventRequestsPending;
        /// <summary>Incremented when a full set of event ranges has arrived.</summary>
        public int EventsFrame { get; private set; }

        Heightfield m_Terrain;
        int m_TerrainBody = -1;

        public int BodyCount => m_BodyCount;
        public int JointCount => m_JointCount;
        /// <summary>The heightfield the dynamic bodies collide with (null: none) and its body slot (-1: none).</summary>
        public Heightfield Terrain => m_Terrain;
        public int TerrainBody => m_TerrainBody;
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
            m_Drives = new GpuBodyDrive[config.MaxBodies];
            m_Pos = new float4[config.MaxBodies];
            m_Rot = new float4[config.MaxBodies];
            m_Vel = new float4[config.MaxBodies];
            m_Uncolored = new uint[config.MaxBodies];
            m_ZeroUints = new uint[config.MaxBodies];
            for (int i = 0; i < m_Uncolored.Length; i++) m_Uncolored[i] = 0xFFFFFFFFu;
            m_Identity = new uint[config.MaxBodies];
            for (int i = 0; i < m_Identity.Length; i++) m_Identity[i] = (uint)i;
            m_WakeArray = new uint2[math.max(config.MaxWakes, 1)];
            m_SpawnChunk = new GpuSpawnRecord[math.max(config.MaxSpawns, 1)];
            m_EventsRead = new uint[config.MaxBodies];
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
            return AddBody(size, density, friction, position, rotation, velocity, 0u);
        }

        /// <summary>Adds a box with the given <see cref="GpuBodyDef"/> flags (rotation lock, heading, events, kind, ...) and drive.</summary>
        public int AddBody(float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity, uint flags, GpuBodyDrive drive = default)
        {
            if (m_BodyCount >= Config.MaxBodies) throw new InvalidOperationException($"AvbdGpuWorld: body capacity {Config.MaxBodies} exceeded");
            int i = m_BodyCount++;
            m_BodyDefs[i] = MakeDef(size, density, friction, flags);
            m_Drives[i] = drive;
            m_Pos[i] = new float4(position, 0);
            m_Rot[i] = rotation.value;
            m_Vel[i] = new float4(velocity, 0);
            if (m_BodyDefs[i].Mass > 0f) { m_ExtentSum += math.cmax(size); m_ExtentCount++; }
            return i;
        }

        static GpuBodyDef MakeDef(float3 size, float density, float friction, uint flags)
        {
            float mass = size.x * size.y * size.z * density;
            if (mass <= 0f) flags |= GpuBodyDef.FlagStatic;
            return new GpuBodyDef
            {
                Size = size,
                Mass = mass,
                Moment = new float3(
                    (size.y * size.y + size.z * size.z) / 12.0f * mass,
                    (size.x * size.x + size.z * size.z) / 12.0f * mass,
                    (size.x * size.x + size.y * size.y) / 12.0f * mass),
                Friction = friction,
                Radius = math.length(size * 0.5f),
                Flags = flags,
            };
        }

        /// <summary>Sets the terrain: a static body slot at the identity pose flagged <see cref="GpuBodyDef.FlagTerrain"/> (the partner
        /// of every terrain manifold, friction = <paramref name="friction"/>) and the heightfield sampled by the narrowphase, uploaded
        /// at once. A second call replaces the field and keeps the slot.</summary>
        public int SetTerrain(Heightfield field, float friction)
        {
            if (field == null) throw new ArgumentNullException(nameof(field));
            if (m_TerrainBody < 0)
                m_TerrainBody = AddBody(float3.zero, 0f, friction, float3.zero, quaternion.identity, float3.zero, GpuBodyDef.FlagTerrain);
            else
            {
                m_BodyDefs[m_TerrainBody].Friction = friction;
                MarkDef(m_TerrainBody);
            }
            m_Terrain = field;
            UpdateTerrain();
            WakeAll();
            return m_TerrainBody;
        }

        /// <summary>Re-uploads the terrain's samples and max mip after they were edited in place (call <see cref="Heightfield.BuildMaxMip"/>
        /// or an editing method first).</summary>
        public void UpdateTerrain()
        {
            if (m_Terrain == null) return;
            Buffers.EnsureTerrain(m_Terrain.SampleCount, m_Terrain.MaxMip.Length);
            Buffers.TerrainHeights.SetData(m_Terrain.Heights);
            Buffers.TerrainMaxMip.SetData(m_Terrain.MaxMip);
            WakeAll();   // sleeping bodies may now hang above (or sit below) the new surface
        }

        // ------------------------------------------------------------------------------------------------ body pools

        /// <summary>Appends <paramref name="count"/> retired (dead) slots and returns the first index: a contiguous pool that
        /// <see cref="SpawnBody"/> fills and <see cref="RetireBody"/> empties without changing any other body's index.</summary>
        public int ReserveBodies(int count)
        {
            if (m_BodyCount + count > Config.MaxBodies) throw new InvalidOperationException($"AvbdGpuWorld: body capacity {Config.MaxBodies} exceeded");
            int first = m_BodyCount;
            for (int i = 0; i < count; i++)
            {
                int slot = m_BodyCount++;
                m_BodyDefs[slot] = MakeDef(new float3(1, 1, 1), 1f, 0.5f, GpuBodyDef.FlagDead);
                m_Drives[slot] = default;
                m_Pos[slot] = float4.zero;
                m_Rot[slot] = quaternion.identity.value;
                m_Vel[slot] = float4.zero;
            }
            return first;
        }

        /// <summary>Brings a retired slot back to life as a new box. The GPU-owned state (pose, velocities, colour, events) is
        /// written by a kernel before the next step; a slot must not be respawned in the step it was retired in (see
        /// <see cref="BodyPool"/>), so that no manifold of the old body is warm started. Pool bodies do not enter the
        /// automatic cell size.</summary>
        public void SpawnBody(int slot, float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity, uint flags, GpuBodyDrive drive = default)
        {
            if (slot < 0 || slot >= m_BodyCount) throw new ArgumentOutOfRangeException(nameof(slot));
            if (!m_BodyDefs[slot].IsDead) throw new InvalidOperationException($"AvbdGpuWorld: body {slot} is alive");
            m_BodyDefs[slot] = MakeDef(size, density, friction, flags & ~GpuBodyDef.FlagDead);
            m_Drives[slot] = drive;
            m_Pos[slot] = new float4(position, 0);
            m_Rot[slot] = rotation.value;
            m_Vel[slot] = new float4(velocity, 0);
            MarkDef(slot);
            MarkDrive(slot);
            if (slot < m_UploadedBodies)
                m_SpawnQueue.Add(new GpuSpawnRecord { Slot = (uint)slot, Pos = m_Pos[slot], Rot = m_Rot[slot], Vel = m_Vel[slot] });
            Wake(slot, WakeMode.Spawn);
        }

        /// <summary>Retires a body: it stops colliding, moving and drawing, and its slot can be spawned into after the next step.
        /// Joints, springs and links on the body must have been removed.</summary>
        public void RetireBody(int slot)
        {
            if (m_BodyDefs[slot].IsDead) return;
            for (int i = 0; i < m_Links.Count; i++)
                if (m_Links[i].a == slot) throw new InvalidOperationException($"AvbdGpuWorld: body {slot} still has a joint, spring or link");
            m_BodyDefs[slot].Flags |= GpuBodyDef.FlagDead;
            m_SpawnQueue.RemoveAll(r => r.Slot == (uint)slot);
            MarkDef(slot);
            Wake(slot, WakeMode.Neighbours);   // whatever rested on or against it must react
        }

        public bool IsAlive(int slot) => !m_BodyDefs[slot].IsDead;
        public uint GetBodyFlags(int slot) => m_BodyDefs[slot].Flags;

        /// <summary>Replaces the flags of a body (the static and dead bits are kept as they are).</summary>
        public void SetBodyFlags(int slot, uint flags)
        {
            const uint keep = GpuBodyDef.FlagStatic | GpuBodyDef.FlagDead;
            m_BodyDefs[slot].Flags = (m_BodyDefs[slot].Flags & keep) | (flags & ~keep);
            MarkDef(slot);
            Wake(slot, WakeMode.Self);
        }

        public GpuBodyDrive GetBodyDrive(int slot) => m_Drives[slot];

        /// <summary>Sets the drive of a body; the body needs <see cref="GpuBodyDef.FlagDriven"/> for it to act. A changed drive
        /// wakes the body (an identical one does not, so a unit holding its position may sleep).</summary>
        public void SetBodyDrive(int slot, in GpuBodyDrive drive)
        {
            if (SameDrive(m_Drives[slot], drive)) return;
            m_Drives[slot] = drive;
            MarkDrive(slot);
            Wake(slot, WakeMode.Self);
        }

        static bool SameDrive(in GpuBodyDrive a, in GpuBodyDrive b) =>
            a.Mode == b.Mode && math.all(a.Target == b.Target) && math.all(a.Mask == b.Mask) && a.Limit == b.Limit && a.Yaw == b.Yaw;

        // ------------------------------------------------------------------------------------------------ sleeping

        /// <summary>Wakes a body for at least <see cref="AvbdGpuParams.SleepTime"/> (applied at the start of the next step); what it
        /// then touches wakes in turn, the whole island of a touched body once it moves faster than <see cref="AvbdGpuParams.WakeSpeed"/>.
        /// Gameplay changes made through the world (drives, flags, joints, spawns, retirements, the terrain) wake by themselves.</summary>
        public void WakeBody(int slot) => Wake(slot, WakeMode.Self);

        /// <summary>Wakes every body (applied at the start of the next step).</summary>
        public void WakeAll() { m_WakeAll = true; m_WakeList.Clear(); }

        void Wake(int slot, uint mode)
        {
            if (m_WakeAll || slot < 0 || slot >= m_UploadedBodies) return;   // a body not uploaded yet starts awake
            if (m_WakeList.Count >= m_WakeArray.Length) { WakeAll(); return; }
            m_WakeList.Add(new uint2((uint)slot, mode));
        }

        /// <summary>Synchronous readback of the sleep words (<see cref="GpuBodySleep"/>) of every body (tests).</summary>
        public uint[] GetSleepSync()
        {
            var words = new uint[math.max(m_BodyCount, 1)];
            if (m_BodyCount > 0) Buffers.BodySleep.GetData(words, 0, 0, m_BodyCount);
            return words;
        }

        /// <summary>Synchronous readback of the island labels (the index of each island's representative body) of every body (tests).</summary>
        public uint[] GetLabelsSync()
        {
            var labels = new uint[math.max(m_BodyCount, 1)];
            if (m_BodyCount > 0) Buffers.BodyLabel.GetData(labels, 0, 0, m_BodyCount);
            return labels;
        }

        void MarkDef(int slot)
        {
            if (slot >= m_UploadedBodies) return;
            m_DirtyDefMin = math.min(m_DirtyDefMin, slot);
            m_DirtyDefMax = math.max(m_DirtyDefMax, slot);
        }

        void MarkDrive(int slot)
        {
            if (slot >= m_UploadedBodies) return;
            m_DirtyDriveMin = math.min(m_DirtyDriveMin, slot);
            m_DirtyDriveMax = math.max(m_DirtyDriveMax, slot);
        }

        public void AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture)
        {
            AddJointIndexed(bodyA, bodyB, rA, rB, stiffnessLin, stiffnessAng, fracture);
        }

        public void AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture,
            float fractureLateral, float fractureTension, float breakDistance, int snapAxis)
        {
            AddJointIndexed(bodyA, bodyB, rA, rB, stiffnessLin, stiffnessAng, fracture, fractureLateral, fractureTension, breakDistance, snapAxis);
        }

        /// <summary>Adds a joint and returns its index (needed to move or remove it later). <paramref name="fracture"/> is the
        /// reference's limit on the angular multiplier; the snap limits (see <see cref="GpuJointDef"/>) act on the linear one.</summary>
        public int AddJointIndexed(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin = float.PositiveInfinity, float stiffnessAng = 0f, float fracture = float.PositiveInfinity,
            float fractureLateral = float.PositiveInfinity, float fractureTension = float.PositiveInfinity, float breakDistance = float.PositiveInfinity, int snapAxis = 2)
        {
            float3 sizeA = bodyA >= 0 ? m_BodyDefs[bodyA].Size : float3.zero;
            var def = new GpuJointDef
            {
                BodyA = bodyA, BodyB = bodyB, RA = rA, RB = rB,
                StiffnessLin = AvbdGpuConstants.ToGpuStiffness(stiffnessLin),
                StiffnessAng = AvbdGpuConstants.ToGpuStiffness(stiffnessAng),
                Fracture = AvbdGpuConstants.ToGpuStiffness(fracture),
                TorqueArm = math.lengthsq(sizeA + m_BodyDefs[bodyB].Size),
                FractureLateral = AvbdGpuConstants.ToGpuStiffness(fractureLateral),
                FractureTension = AvbdGpuConstants.ToGpuStiffness(fractureTension),
                BreakDistance = AvbdGpuConstants.ToGpuStiffness(breakDistance),
                SnapAxis = snapAxis,
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
            Wake(bodyA, WakeMode.Self);
            Wake(bodyB, WakeMode.Self);
            return j;
        }

        /// <summary>Moves the world anchor (bodyA = -1) or the local anchor of a joint.</summary>
        public void SetJointAnchor(int joint, float3 rA)
        {
            m_JointDefs[joint].RA = rA;
            if (joint < m_UploadedJoints) m_DirtyJoints.Add(joint);
            Wake(m_JointDefs[joint].BodyA, WakeMode.Self);
            Wake(m_JointDefs[joint].BodyB, WakeMode.Self);
        }

        /// <summary>Disables a joint (its slot is reused by the next AddJoint).</summary>
        public void RemoveJoint(int joint)
        {
            var def = m_JointDefs[joint];
            m_JointDefs[joint] = new GpuJointDef
            {
                BodyA = -1, BodyB = 0, StiffnessLin = 0, StiffnessAng = 0, Fracture = AvbdGpuConstants.HardStiffness,
                FractureLateral = AvbdGpuConstants.HardStiffness, FractureTension = AvbdGpuConstants.HardStiffness, BreakDistance = AvbdGpuConstants.HardStiffness,
            };
            if (def.BodyA >= 0) RemoveLink(def.BodyA, def.BodyB, ConsRef.Make(ConsRef.Joint, (uint)joint));
            if (joint < m_UploadedJoints) m_DirtyJoints.Add(joint);
            m_FreeJoints.Add(joint);
            Wake(def.BodyA, WakeMode.Self);
            Wake(def.BodyB, WakeMode.Self);
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
            Wake(bodyA, WakeMode.Self);
            Wake(bodyB, WakeMode.Self);
        }

        public void AddIgnoreCollision(int bodyA, int bodyB)
        {
            AddLink(bodyA, bodyB, ConsRef.Make(ConsRef.Ignore, 0));
            Wake(bodyA, WakeMode.Self);   // a carried manifold of the pair must go
            Wake(bodyB, WakeMode.Self);
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
            m_Terrain = null; m_TerrainBody = -1;
            m_StepMsSum = 0; m_StepCount = 0;
            m_Stats = default;
            StepIndex = 0;
            m_DirtyDefMin = m_DirtyDriveMin = int.MaxValue; m_DirtyDefMax = m_DirtyDriveMax = -1;
            m_PosReadStep = -1;
            m_SpawnQueue.Clear();
            m_WakeList.Clear(); m_WakeAll = false;
            EventRanges.Clear();
            Array.Clear(m_EventsRead, 0, m_EventsRead.Length);
            AvbdGpuBuffers.Clear(Buffers.HashPrev);
            AvbdGpuBuffers.Clear(Buffers.HashCur);
            AvbdGpuBuffers.Clear(Buffers.BodyConsCount);
            AvbdGpuBuffers.Clear(Buffers.LinkStart);
            AvbdGpuBuffers.Clear(Buffers.BodySleep);
            AvbdGpuBuffers.Clear(Buffers.WakeMark);
            AvbdGpuBuffers.Clear(Buffers.Counters);   // CarrySleeping runs over last step's manifold count
            AvbdGpuBuffers.Clear(Buffers.Stats);
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
                b.BodyDrive.SetData(m_Drives, start, start, count);
                b.BodyPos.SetData(m_Pos, start, start, count);
                b.BodyRot.SetData(m_Rot, start, start, count);
                b.BodyVelLin.SetData(m_Vel, start, start, count);
                b.BodyPrevVelLin.SetData(m_Vel, start, start, count);
                b.BodyVelAng.SetData(new float4[count], 0, start, count);
                b.BodyColor.SetData(m_Uncolored, start, start, count);
                b.BodyEvents.SetData(m_ZeroUints, start, start, count);
                b.BodySleep.SetData(m_ZeroUints, start, start, count);   // awake, and each its own island
                b.BodyLabel.SetData(m_Identity, start, start, count);
                m_UploadedBodies = m_BodyCount;
            }
            if (m_WakeAll)
            {
                if (m_BodyCount > 0) b.BodySleep.SetData(m_ZeroUints, 0, 0, m_BodyCount);
                m_WakeAll = false;
                m_WakeList.Clear();
            }
            if (m_WakeList.Count > 0 && Params.Sleep)
            {
                m_WakeList.CopyTo(m_WakeArray, 0);
                b.WakeList.SetData(m_WakeArray, 0, 0, m_WakeList.Count);
            }
            if (m_DirtyDefMax >= 0)
            {
                b.BodyDef.SetData(m_BodyDefs, m_DirtyDefMin, m_DirtyDefMin, m_DirtyDefMax - m_DirtyDefMin + 1);
                m_DirtyDefMin = int.MaxValue; m_DirtyDefMax = -1;
            }
            if (m_DirtyDriveMax >= 0)
            {
                b.BodyDrive.SetData(m_Drives, m_DirtyDriveMin, m_DirtyDriveMin, m_DirtyDriveMax - m_DirtyDriveMin + 1);
                m_DirtyDriveMin = int.MaxValue; m_DirtyDriveMax = -1;
            }
            FlushSpawns();
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

        /// <summary>Writes the queued spawns' GPU state (one upload and one dispatch per chunk of Config.MaxSpawns).</summary>
        void FlushSpawns()
        {
            var cs = Kernels.Util; int k = Kernels.SpawnBodies; var b = Buffers;
            for (int done = 0; done < m_SpawnQueue.Count; done += m_SpawnChunk.Length)
            {
                int n = math.min(m_SpawnChunk.Length, m_SpawnQueue.Count - done);
                m_SpawnQueue.CopyTo(done, m_SpawnChunk, 0, n);
                b.SpawnRecords.SetData(m_SpawnChunk, 0, 0, n);
                cs.SetBuffer(k, "_SpawnRecords", b.SpawnRecords);
                cs.SetBuffer(k, "_BodyPos", b.BodyPos);
                cs.SetBuffer(k, "_BodyRot", b.BodyRot);
                cs.SetBuffer(k, "_BodyVelLin", b.BodyVelLin);
                cs.SetBuffer(k, "_BodyVelAng", b.BodyVelAng);
                cs.SetBuffer(k, "_BodyPrevVelLin", b.BodyPrevVelLin);
                cs.SetBuffer(k, "_BodyColor", b.BodyColor);
                cs.SetBuffer(k, "_BodyEvents", b.BodyEvents);
                cs.SetInt(s_SpawnCount, n);
                cs.Dispatch(k, (n + AvbdGpuConstants.ThreadGroupSize - 1) / AvbdGpuConstants.ThreadGroupSize, 1, 1);
            }
            m_SpawnQueue.Clear();
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
                TerrainSlot = GpuParams.NoTerrain,
                SleepSteps = p.Sleep ? (uint)math.max(1, (int)math.ceil(p.SleepTime / dt)) : 0u,
                WakeCount = p.Sleep ? (uint)m_WakeList.Count : 0u,
                SleepDistSq = p.SleepDistance * p.SleepDistance, SleepAngleSq = p.SleepAngle * p.SleepAngle, WakeSpeedSq = p.WakeSpeed * p.WakeSpeed,
                Relabel = StepIndex % math.max(1, p.SleepRelabelSteps) == 0 ? 1u : 0u,
                WakeHoldSteps = (uint)math.max(1, (int)math.ceil(p.WakeHold / dt)),
            };
            if (m_Terrain != null && m_TerrainBody >= 0)
            {
                var t = m_Terrain;
                m_GpuParams.TerrainOrigin = t.Origin; m_GpuParams.TerrainCell = t.Cell; m_GpuParams.TerrainInvCell = 1f / t.Cell;
                m_GpuParams.TerrainResX = (uint)t.ResX; m_GpuParams.TerrainResZ = (uint)t.ResZ;
                m_GpuParams.TerrainMipX = (uint)t.MipX; m_GpuParams.TerrainMipZ = (uint)t.MipZ;
                m_GpuParams.TerrainSlot = (uint)m_TerrainBody; m_GpuParams.TerrainMaxHeight = t.MaxHeight;
            }
            m_GpuParamsArray[0] = m_GpuParams;
            Buffers.Params.SetData(m_GpuParamsArray);
        }

        // ------------------------------------------------------------------------------------------------ stepping

        /// <summary>Advances the world by Params.Dt (Params.Substeps substeps). Nothing is read back synchronously.</summary>
        public void Step()
        {
            // a change of gravity or switching sleeping off must reach the sleeping bodies
            if (math.any(Params.Gravity != m_LastGravity) || (m_SleepWasOn && !Params.Sleep)) WakeAll();
            m_LastGravity = Params.Gravity; m_SleepWasOn = Params.Sleep;
            Upload();
            int substeps = math.max(1, Params.Substeps);
            FillParams(Params.Dt / substeps);
            m_Pipeline.Ensure(math.max(1, Params.Iterations), m_ActiveColors, Params.PostStabilize, math.max(2, Params.ColorRounds), Params.Alpha, m_Terrain != null,
                Params.Sleep, math.max(2, Params.SleepLabelRounds), math.max(1, Params.SleepHops));

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
            m_WakeList.Clear();

            if (!m_StatsRequestPending)
            {
                m_StatsRequestPending = true;
                AsyncGPUReadback.Request(Buffers.Stats, OnStats);
            }
            if (ReadbackPoses && !m_PoseRequestPending && m_BodyCount > 0)
            {
                m_PoseRequestPending = true;
                m_PosRequestStep = StepIndex;
                int count = m_BodyCount;
                AsyncGPUReadback.Request(Buffers.BodyPos, count * 16, 0, r => OnPoses(r, count, true));
                AsyncGPUReadback.Request(Buffers.BodyRot, count * 16, 0, r => OnPoses(r, count, false));
            }
            if (m_EventRequestsPending == 0 && EventRanges.Count > 0)
            {
                foreach (var (start, count) in EventRanges)
                {
                    if (count <= 0 || start < 0 || start + count > m_BodyCount) continue;
                    m_EventRequestsPending++;
                    int s0 = start, n = count;
                    AsyncGPUReadback.Request(Buffers.BodyEvents, n * 4, s0 * 4, r => OnEvents(r, s0, n));
                }
            }
        }

        void OnEvents(AsyncGPUReadbackRequest r, int start, int count)
        {
            if (!r.hasError) NativeArray<uint>.Copy(r.GetData<uint>(), 0, m_EventsRead, start, count);
            if (--m_EventRequestsPending == 0) EventsFrame++;
        }

        /// <summary>Event words from the last asynchronous readback of <see cref="EventRanges"/> (one or two frames old), indexed by body.</summary>
        public uint[] ReadEvents => m_EventsRead;

        /// <summary>Synchronous readback of the event words of a body range (tests).</summary>
        public uint[] GetEventsSync(int start, int count)
        {
            var data = new uint[math.max(count, 1)];
            if (count > 0) Buffers.BodyEvents.GetData(data, 0, start, count);
            return data;
        }

        /// <summary>Fills <see cref="ReadEvents"/> for every event range synchronously (tests; no frame loop, no async readback).</summary>
        public void ReadEventsSync()
        {
            foreach (var (start, count) in EventRanges)
            {
                if (count <= 0 || start < 0 || start + count > m_BodyCount) continue;
                Buffers.BodyEvents.GetData(m_EventsRead, start, start, count);
            }
            EventsFrame++;
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
            m_Stats.TerrainManifolds = (int)data[(int)StatSlot.TerrainManifolds];
            m_Stats.Sleeping = (int)data[(int)StatSlot.Sleeping];
            m_Stats.CarriedManifolds = (int)data[(int)StatSlot.CarriedManifolds];
            m_Stats.Woken = (int)data[(int)StatSlot.Woken];
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
                m_PosReadStep = m_PosRequestStep;
            }
        }

        /// <summary>Poses from the last asynchronous readback (one or two frames old); null until the first arrives.</summary>
        public float4[] ReadPositions => m_PosRead;
        public float4[] ReadRotations => m_RotRead;
        public int ReadCount => m_PosReadCount;
        /// <summary>The <see cref="StepIndex"/> after which the read-back poses were taken (-1 before the first readback): bodies
        /// spawned at or after that step are not in them yet.</summary>
        public int ReadStep => m_PosReadStep;

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
            s.TerrainManifolds = (int)data[(int)StatSlot.TerrainManifolds];
            s.Sleeping = (int)data[(int)StatSlot.Sleeping];
            s.CarriedManifolds = (int)data[(int)StatSlot.CarriedManifolds];
            s.Woken = (int)data[(int)StatSlot.Woken];
            s.ActiveColors = m_ActiveColors;
            m_Stats = s;
            AdaptColors();
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
                if (m_BodyDefs[i].IsStatic) continue;
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
