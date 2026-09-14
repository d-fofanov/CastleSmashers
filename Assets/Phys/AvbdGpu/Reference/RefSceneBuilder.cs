using System.Collections.Generic;
using Phys.AvbdGpu.Scenes;
using Unity.Mathematics;

namespace Phys.AvbdRef
{
    /// <summary>Builds scenes from the shared catalog into the CPU reference solver; body index = creation order.</summary>
    public sealed class RefSceneBuilder : ISceneBuilder
    {
        public readonly Solver Solver;
        public readonly List<Rigid> Bodies = new List<Rigid>();
        public readonly List<Joint> Joints = new List<Joint>();

        public RefSceneBuilder(Solver solver) { Solver = solver; }

        public static RefSceneBuilder Build(int scene, Solver solver = null)
        {
            var b = new RefSceneBuilder(solver ?? new Solver());
            AvbdScenes.Build(b, scene);
            return b;
        }

        public int AddBody(float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity)
        {
            var body = new Rigid(Solver, size, density, friction, position, velocity) { positionAng = Quat.FromUnity(rotation), initialAng = Quat.FromUnity(rotation) };
            Bodies.Add(body);
            return Bodies.Count - 1;
        }

        public void AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture)
        {
            Joints.Add(new Joint(Solver, bodyA < 0 ? null : Bodies[bodyA], Bodies[bodyB], rA, rB, stiffnessLin, stiffnessAng, fracture));
        }

        public void AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin, float stiffnessAng, float fracture,
            float fractureLateral, float fractureTension, float breakDistance, int snapAxis)
        {
            Joints.Add(new Joint(Solver, bodyA < 0 ? null : Bodies[bodyA], Bodies[bodyB], rA, rB, stiffnessLin, stiffnessAng, fracture)
                { fractureLateral = fractureLateral, fractureTension = fractureTension, breakDistance = breakDistance, snapAxis = snapAxis });
        }

        public void AddSpring(int bodyA, int bodyB, float3 rA, float3 rB, float stiffness, float rest)
        {
            new Spring(Solver, Bodies[bodyA], Bodies[bodyB], rA, rB, stiffness, rest);
        }

        public void AddIgnoreCollision(int bodyA, int bodyB)
        {
            new IgnoreCollision(Solver, Bodies[bodyA], Bodies[bodyB]);
        }

        public float3 Position(int i) => Bodies[i].positionLin;
        public quaternion Rotation(int i) => Bodies[i].positionAng.ToUnity();
        public float3 Velocity(int i) => Bodies[i].velocityLin;
    }
}
