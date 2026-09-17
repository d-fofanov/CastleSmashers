using UnityEngine;

namespace Phys.AvbdGpu.Siege
{
    /// <summary>What a projectile does where it lands: the physics (solver metres, N s) and the visual (a mesh spawned at the impact,
    /// scaled and faded over its life). Created through Phys / Siege / Hit Effect or by Phys / Create Siege Configs.</summary>
    [CreateAssetMenu(menuName = "Phys/Siege/Hit Effect", fileName = "HitEffect")]
    public sealed class HitEffectConfig : ScriptableObject
    {
        [Header("Physics (solver metres)")]
        [Tooltip("Every joint anchored within this radius of the impact breaks, unconditionally (0: none).")]
        public float PulverizeRadius = 1f;
        [Tooltip("Every body within this radius gets an impulse away from the impact, falling off linearly to the radius (0: none).")]
        public float ImpactRadius = 3f;
        [Tooltip("The impulse at the centre (N s): a 1 kg brick there gets this many m/s.")]
        public float Impulse = 10f;
        [Tooltip("Upward bias of the impulse direction (0 radial, 1 = 45 degrees up for a body beside the impact).")]
        public float Lift = 0.3f;
        [Tooltip("Units within this radius of the impact die (0: none); the impulse tosses them either way.")]
        public float KillRadius = 2f;

        [Header("Visual")]
        [Tooltip("The mesh spawned at the impact (an explosion model, impact-centre pivot); nothing when unset.")]
        public Mesh Mesh;
        [Tooltip("Colour and opacity at the start (fades to transparent over the life).")]
        public Color32 Color = new Color32(255, 140, 60, 230);
        [Tooltip("Uniform scale of the mesh at the start and the end of its life (solver metres per model metre).")]
        public float StartScale = 2f;
        public float EndScale = 8f;
        [Tooltip("Steps the visual lives (60 per second).")]
        public int LifeSteps = 40;
        [Tooltip("Spin about the vertical (degrees per second).")]
        public float SpinDegPerSec = 90f;
        [Range(0f, 1f)] [Tooltip("0: lit like the bodies; 1: the flat colour (a glow).")]
        public float Unlit = 1f;

        public HitEffect ToHitEffect() => new HitEffect
        {
            PulverizeRadius = Mathf.Max(PulverizeRadius, 0f), ImpactRadius = Mathf.Max(ImpactRadius, 0f), Impulse = Mathf.Max(Impulse, 0f), Lift = Mathf.Max(Lift, 0f), KillRadius = Mathf.Max(KillRadius, 0f),
        };
    }
}
