using System;
using Unity.Mathematics;
using UnityEngine;

namespace Phys.AvbdGpu.Siege
{
    /// <summary>A kind of unit: its body model and box, how it moves, its weapon model and where it is held, its projectile model, mass
    /// and trajectory, and the hit effect of its shots. Model metres throughout (the figure scale, 0.48 m tall), relative to the model's
    /// pivot; <see cref="ToArchetype"/> scales them into the solver's units. Created through Phys / Siege / Unit or by Phys / Create
    /// Siege Configs, which fills the bounds from the meshes.</summary>
    [CreateAssetMenu(menuName = "Phys/Siege/Unit", fileName = "Unit")]
    public sealed class UnitConfig : ScriptableObject
    {
        /// <summary>A mesh of the weapon and where it sits relative to the figure's pivot (model metres, degrees).</summary>
        [Serializable]
        public struct WeaponPart
        {
            public Mesh Mesh;
            public Vector3 LocalPosition;
            public Vector3 LocalEuler;
            public float Scale;
        }

        public string DisplayName = "Unit";

        [Header("Body (model metres)")]
        [Tooltip("The body model (foot-centre pivot, facing +Z).")]
        public Mesh BodyMesh;
        [Tooltip("The body model's bounds (the collision box; filled from the mesh by Create Siege Configs).")]
        public Vector3 BodyBoundsCenter = new Vector3(0f, 0.24f, 0f);
        public Vector3 BodyBoundsSize = new Vector3(0.354f, 0.48f, 0.12f);
        public Color32 Tint = new Color32(200, 60, 50, 255);

        [Header("Motion (solver units)")]
        public float Mass = 1f;
        public float Friction = 0.6f;
        [Tooltip("Walking speed (m/s) and the motor's force cap (N).")]
        public float Speed = 3f;
        public float Force = 12f;
        [Tooltip("Fast projectile hits that kill it.")]
        public int HitPoints = 1;

        [Header("Weapon (model metres, relative to the body pivot)")]
        public WeaponPart[] Weapon = new WeaponPart[0];
        [Tooltip("The weapon swings about this point and axis when the unit shoots (a trebuchet's axle); 0 degrees = no swing.")]
        public Vector3 SwingPivot;
        public Vector3 SwingAxis = Vector3.right;
        public float SwingDegrees;
        [Tooltip("Steps of the swing forward (the shot) and back to rest.")]
        public int SwingSteps = 15;
        public int ResetSteps = 90;

        [Header("Projectile (model metres)")]
        [Tooltip("The projectile models (centre pivot, pointing +Z); several are visual variants the shots cycle through.")]
        public Mesh[] ProjectileMeshes = new Mesh[0];
        public Vector3 ProjectileBoundsCenter;
        public Vector3 ProjectileBoundsSize = new Vector3(0.034f, 0.035f, 0.28f);
        [Tooltip("Mass (kg) in the solver.")]
        public float ProjectileMass = 0.02f;
        [Tooltip("The projectile keeps its +Z along its velocity (arrows, spells); off for a rock.")]
        public bool AlignToVelocity = true;
        public Color32 ProjectileColor = new Color32(222, 200, 150, 255);
        [Range(0f, 1f)] public float ProjectileUnlit;
        [Tooltip("A mesh attached to the projectile (a spell on an arrow's tip), its place relative to the projectile's pivot, its scale.")]
        public Mesh TipMesh;
        public Vector3 TipLocalPosition = new Vector3(0f, 0f, 0.14f);
        public float TipScale = 0.5f;
        public Color32 TipColor = new Color32(255, 120, 40, 255);
        [Range(0f, 1f)] public float TipUnlit = 1f;

        [Header("Trajectory (solver units)")]
        public Trajectory Trajectory = Trajectory.Elevation;
        public float ElevationDeg = 55f;
        [Tooltip("Launch speed (m/s) for the arc, straight and homing trajectories; the top speed for the elevation one.")]
        public float LaunchSpeed = 35f;
        public float MaxSpeed = 40f;
        [Tooltip("Random velocity scatter as a fraction of the speed.")]
        public float Spread = 0.03f;
        [Tooltip("Thrust (N) of a straight shot along its direction, or the pull of a homing shot towards its aim.")]
        public float Thrust;
        [Tooltip("Where the shot leaves relative to the body pivot (model metres: the shoulder, a trebuchet's arm).")]
        public Vector3 LaunchPoint = new Vector3(0f, 0.36f, 0.06f);
        [Tooltip("How far along its velocity the projectile spawns (solver metres), clear of the shooter's box.")]
        public float LaunchOffset = 2.6f;
        [Tooltip("Steps between the start of the shot (the swing) and the launch.")]
        public int LaunchDelaySteps;
        [Tooltip("Steps a spent projectile lies about before its slot is freed (0: gone at the impact).")]
        public int ProjectileRetireDelay = 120;

        [Header("Engagement (solver metres, steps)")]
        public float Range = 30f;
        public int CooldownSteps = 180;
        [Tooltip("Shoots at the nearest enemy in range on its own when it has no order.")]
        public bool AutoEngage = true;
        public HitEffectConfig HitEffect;

        /// <summary>The archetype at the solver scale (solver metres per model metre) with the collision margin: the box is the bounds
        /// scaled plus one margin in height, the mesh offset puts the pivot so that the model's feet rest on the ground.</summary>
        public UnitArchetype ToArchetype(float scale, float margin)
        {
            UnitArchetype.BoxOf(BodyBoundsCenter, BodyBoundsSize, scale, margin, out float3 box, out float3 meshOffset);
            UnitArchetype.BoxOf(ProjectileBoundsCenter, ProjectileBoundsSize, scale, margin, out float3 pbox, out float3 pmeshOffset);
            return new UnitArchetype
            {
                Name = string.IsNullOrEmpty(DisplayName) ? name : DisplayName,
                BoxSize = box, MeshOffset = meshOffset, StandHeight = box.y * 0.5f - margin,
                Mass = Mass > 0f ? Mass : 1f, Friction = Friction > 0f ? Friction : 0.6f, Speed = Speed > 0f ? Speed : 3f, Force = Force > 0f ? Force : 12f,
                HitPoints = math.max(HitPoints, 1),
                Range = Range > 0f ? Range : 30f, CooldownSteps = math.max(CooldownSteps, 1), AutoEngage = AutoEngage,
                Trajectory = Trajectory, ElevationDeg = ElevationDeg, LaunchSpeed = LaunchSpeed > 0f ? LaunchSpeed : 35f, MaxSpeed = MaxSpeed > 0f ? MaxSpeed : 40f,
                Spread = math.max(Spread, 0f), Thrust = math.max(Thrust, 0f),
                LaunchLocal = (float3)LaunchPoint * scale + meshOffset, LaunchOffset = LaunchOffset > 0f ? LaunchOffset : 2.6f, LaunchDelaySteps = math.max(LaunchDelaySteps, 0),
                ProjectileBoxSize = pbox, ProjectileMeshOffset = pmeshOffset, ProjectileMass = ProjectileMass > 0f ? ProjectileMass : 0.02f, ProjectileAlign = AlignToVelocity,
                ProjectileVariants = math.max(ProjectileMeshes != null ? ProjectileMeshes.Length : 0, 1), ProjectileRetireDelay = math.max(ProjectileRetireDelay, 0),
                Hit = HitEffect != null ? HitEffect.ToHitEffect() : default,
            };
        }
    }
}
