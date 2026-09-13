// Port of rigid.cpp / force.cpp / solver.h from avbd-demo3d (https://github.com/savant117/avbd-demo3d).
//
// Copyright (c) 2026 Chris Giles
//
// Permission to use, copy, modify, distribute and sell this software
// and its documentation for any purpose is hereby granted without fee,
// provided that the above copyright notice appear in all copies.
// Chris Giles makes no representations about the suitability
// of this software for any purpose.
// It is provided "as is" without express or implied warranty.

using Unity.Mathematics;

namespace Phys.AvbdRef
{
    public static class RefConstants
    {
        public const float PenaltyMin = 1.0f;              // Minimum penalty parameter
        public const float PenaltyMax = 10000000000.0f;    // Maximum penalty parameter
        public const float CollisionMargin = 0.01f;        // Margin for collision detection to avoid flickering contacts
        public const float StickThresh = 0.00001f;         // Position threshold for sticking contacts (ie static friction)
    }

    /// <summary>Holds all the state for a single rigid body that is needed by AVBD.</summary>
    public sealed class Rigid
    {
        public Solver solver;
        public Force forces;
        public Rigid next;
        public float3 positionLin;
        public Quat positionAng;
        public float3 initialLin;
        public Quat initialAng;
        public float3 inertialLin;
        public Quat inertialAng;
        public float3 velocityLin;
        public float3 velocityAng;
        public float3 prevVelocityLin;
        public float3 size; // Full widths in each dimension
        public float mass;
        public float3 moment;
        public float friction;
        public float radius;

        public Rigid(Solver solver, float3 size, float density, float friction, float3 position, float3 velocity = default)
        {
            this.solver = solver;
            forces = null;
            positionLin = position;
            positionAng = Quat.Identity;
            velocityLin = velocity;
            velocityAng = float3.zero;
            prevVelocityLin = velocity;
            this.size = size;
            this.friction = friction;

            // Add to linked list
            next = solver.bodies;
            solver.bodies = this;
            solver.OrderOf(this);

            // Compute mass properties and bounding radius
            mass = size.x * size.y * size.z * density;
            moment = new float3(
                (size.y * size.y + size.z * size.z) / 12.0f * mass,
                (size.x * size.x + size.z * size.z) / 12.0f * mass,
                (size.x * size.x + size.y * size.y) / 12.0f * mass);
            radius = math.length(size * 0.5f);
        }

        /// <summary>Removes the body from the solver (the C++ destructor).</summary>
        public void Destroy()
        {
            if (solver.bodies == this) { solver.bodies = next; return; }
            for (Rigid p = solver.bodies; p != null; p = p.next)
                if (p.next == this) { p.next = next; return; }
        }

        /// <summary>Check if this body is constrained to the other body.</summary>
        public bool ConstrainedTo(Rigid other)
        {
            for (Force f = forces; f != null; f = f.bodyA == this ? f.nextA : f.nextB)
                if ((f.bodyA == this && f.bodyB == other) || (f.bodyA == other && f.bodyB == this))
                    return true;
            return false;
        }
    }

    /// <summary>Holds all user defined and derived constraint parameters, and provides a common interface for all forces.</summary>
    public abstract class Force
    {
        public Solver solver;
        public Rigid bodyA;
        public Rigid bodyB;
        public Force nextA;
        public Force nextB;
        public Force next;

        protected Force(Solver solver, Rigid bodyA, Rigid bodyB)
        {
            this.solver = solver;
            this.bodyA = bodyA;
            this.bodyB = bodyB;

            // Add to solver linked list
            next = solver.forces;
            solver.forces = this;

            // Add to body linked lists
            if (bodyA != null)
            {
                nextA = bodyA.forces;
                bodyA.forces = this;
            }
            if (bodyB != null)
            {
                nextB = bodyB.forces;
                bodyB.forces = this;
            }
        }

        /// <summary>Removes the force from the solver and body lists (the C++ destructor).</summary>
        public void Destroy()
        {
            // Remove from solver linked list
            if (solver.forces == this) solver.forces = next;
            else
                for (Force p = solver.forces; p != null; p = p.next)
                    if (p.next == this) { p.next = next; break; }

            // Remove from body linked lists
            if (bodyA != null) Unlink(bodyA, nextA);
            if (bodyB != null) Unlink(bodyB, nextB);
        }

        void Unlink(Rigid body, Force replacement)
        {
            if (body.forces == this) { body.forces = replacement; return; }
            for (Force p = body.forces; p != null;)
            {
                Force pn = p.bodyA == body ? p.nextA : p.nextB;
                if (pn == this)
                {
                    if (p.bodyA == body) p.nextA = replacement; else p.nextB = replacement;
                    return;
                }
                p = pn;
            }
        }

        public abstract bool Initialize();
        public abstract void UpdatePrimal(Rigid body, float alpha, ref Mat3 lhsLin, ref Mat3 lhsAng, ref Mat3 lhsCross, ref float3 rhsLin, ref float3 rhsAng);
        public abstract void UpdateDual(float alpha);
    }

    /// <summary>Force which has no physical effect, but is used to ignore collisions between two bodies.</summary>
    public sealed class IgnoreCollision : Force
    {
        public IgnoreCollision(Solver solver, Rigid bodyA, Rigid bodyB) : base(solver, bodyA, bodyB) { }
        public override bool Initialize() => true;
        public override void UpdatePrimal(Rigid body, float alpha, ref Mat3 lhsLin, ref Mat3 lhsAng, ref Mat3 lhsCross, ref float3 rhsLin, ref float3 rhsAng) { }
        public override void UpdateDual(float alpha) { }
    }
}
