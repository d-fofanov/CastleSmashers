using System.Collections.Generic;
using Unity.Mathematics;

namespace Phys.AvbdGpu
{
    /// <summary>A contiguous range of body slots reserved in an <see cref="AvbdGpuWorld"/> for bodies that come and go (units,
    /// projectiles): spawns take free slots, retired slots go back to the free list once the world has stepped past the
    /// retirement (a retired body produces no pairs, so after one step no manifold of it is left to warm start a newcomer).
    /// The range stays contiguous, so it can be drawn with one mesh range and read back as one event range.</summary>
    public sealed class BodyPool
    {
        public readonly AvbdGpuWorld World;
        public readonly int Start, Capacity;
        readonly Stack<int> m_Free = new Stack<int>();
        readonly Queue<(int slot, int step)> m_Cooling = new Queue<(int, int)>();
        int m_Alive;

        public int Alive => m_Alive;
        public int End => Start + Capacity;
        /// <summary>Slots that can be spawned into right now (retired slots return once the world has stepped).</summary>
        public int Free { get { Recycle(); return m_Free.Count; } }
        public bool Contains(int slot) => slot >= Start && slot < End;

        public BodyPool(AvbdGpuWorld world, int capacity)
        {
            World = world;
            Capacity = capacity;
            Start = world.ReserveBodies(capacity);
            for (int i = capacity - 1; i >= 0; i--) m_Free.Push(Start + i);   // low slots first
            world.EventRanges.Add((Start, capacity));
        }

        /// <summary>Spawns a box into a free slot; returns the body index or -1 when the pool is full.</summary>
        public int Spawn(float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity, uint flags, GpuBodyDrive drive = default)
        {
            Recycle();
            if (m_Free.Count == 0) return -1;
            int slot = m_Free.Pop();
            World.SpawnBody(slot, size, density, friction, position, rotation, velocity, flags, drive);
            m_Alive++;
            return slot;
        }

        /// <summary>Retires a body of the pool; its slot becomes free after the next step.</summary>
        public void Retire(int slot)
        {
            if (!Contains(slot) || !World.IsAlive(slot)) return;
            World.RetireBody(slot);
            m_Cooling.Enqueue((slot, World.StepIndex));
            m_Alive--;
        }

        /// <summary>Retires every live body of the pool.</summary>
        public void Clear()
        {
            for (int i = Start; i < End; i++) Retire(i);
        }

        void Recycle()
        {
            while (m_Cooling.Count > 0 && m_Cooling.Peek().step < World.StepIndex) m_Free.Push(m_Cooling.Dequeue().slot);
        }
    }
}
