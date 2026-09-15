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

    /// <summary>External drive of a body (mirror of the GPU solver's BodyDrive): an acceleration of the inertial pose.</summary>
    public struct Drive
    {
        public const int None = 0, Force = 1, ToPoint = 2, Motor = 3;
        public int mode;
        /// <summary>Force (N) for Force, the point for ToPoint, target velocity (m/s) for Motor.</summary>
        public float3 target;
        /// <summary>Motor: the axes it acts on (1 / 0).</summary>
        public float3 mask;
        /// <summary>ToPoint: force magnitude (N); Motor: force cap (N).</summary>
        public float limit;
        /// <summary>Heading about +y (rad) of a heading body.</summary>
        public float yaw;

        public float3 Acceleration(float3 pos, float3 vel, float mass, float dt)
        {
            float3 F = float3.zero;
            if (mode == Force)
                F = target;
            else if (mode == ToPoint)
            {
                float3 dir = target - pos;
                float len = math.length(dir);
                if (len > 1.0e-6f) F = dir * (limit / len);
            }
            else if (mode == Motor)
            {
                F = (target - vel) * mask * (mass / dt);
                float f = math.length(F);
                if (f > limit && f > 0) F *= limit / f;
            }
            return F / mass;
        }
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

        // Extensions mirrored from the GPU solver (off by default): frozen angular degrees of freedom, a kinematic
        // orientation source and an external drive.
        public bool lockRotation;
        /// <summary>Orientation set to the drive's yaw about +y at the start of every step (implies lockRotation).</summary>
        public bool heading;
        /// <summary>Orientation set to point +z along the velocity while faster than 1 m/s (implies lockRotation).</summary>
        public bool alignVelocity;
        public Drive drive;
        /// <summary>The terrain body (mirror of the GPU solver's terrain slot): static, at the identity pose, colliding through the
        /// solver's heightfield instead of its (empty) box.</summary>
        public bool terrain;
        public bool LockedRotation => lockRotation || heading || alignVelocity;

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
