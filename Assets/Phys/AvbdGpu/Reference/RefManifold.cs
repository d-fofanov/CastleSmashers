// Port of manifold.cpp from avbd-demo3d (https://github.com/savant117/avbd-demo3d).
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
    /// <summary>Collision manifold between two rigid bodies, which contains up to eight frictional contact points.</summary>
    public sealed class Manifold : Force
    {
        public const int MaxContacts = 8;

        /// <summary>Contact point information for a single contact.</summary>
        public struct Contact
        {
            public int feature;   // FeaturePair key
            public float3 rA;     // contact offset in A's local space (relative to center)
            public float3 rB;     // contact offset in B's local space (relative to center)
            public float3 C0;
            public float3 penalty;
            public float3 lambda;
            public bool stick;
        }

        public Contact[] contacts = new Contact[MaxContacts];
        public Mat3 basis; // Normal in the first row (pointing from B to A), and tangents in the second and third rows
        public int numContacts;
        public float friction;

        public Manifold(Solver solver, Rigid bodyA, Rigid bodyB) : base(solver, bodyA, bodyB)
        {
            numContacts = 0;
        }

        public override bool Initialize()
        {
            // Compute friction
            friction = math.sqrt(bodyA.friction * bodyB.friction);

            // Compute new contacts
            var newContacts = new Contact[MaxContacts];
            int newNumContacts = RefCollide.Collide(bodyA, bodyB, newContacts, out basis);

            // Merge old contact data with new contacts
            for (int i = 0; i < newNumContacts; i++)
            {
                for (int j = 0; j < numContacts; j++)
                {
                    if (newContacts[i].feature == contacts[j].feature)
                    {
                        float3 newRA = newContacts[i].rA;
                        float3 newRB = newContacts[i].rB;
                        newContacts[i] = contacts[j];

                        // If no static friction in last frame, use the new contact point locations
                        if (!contacts[j].stick)
                        {
                            newContacts[i].rA = newRA;
                            newContacts[i].rB = newRB;
                        }
                        break;
                    }
                }
            }

            // Copy new contacts to the manifold
            numContacts = newNumContacts;
            for (int i = 0; i < numContacts; i++)
                contacts[i] = newContacts[i];

            // Compute error at q- and update penalty and lambdas
            for (int i = 0; i < numContacts; i++)
            {
                // Error at q-
                float3 xA = RefMath.Transform(bodyA.positionLin, bodyA.positionAng, contacts[i].rA);
                float3 xB = RefMath.Transform(bodyB.positionLin, bodyB.positionAng, contacts[i].rB);
                contacts[i].C0 = basis * (xA - xB) + new float3(RefConstants.CollisionMargin, 0, 0);

                // Warmstart the dual variables and penalty parameters (Eq. 19)
                // Penalty is safely clamped to a minimum and maximum value
                contacts[i].lambda = contacts[i].lambda * solver.alpha * solver.gamma;
                contacts[i].penalty = RefMath.Clamp(contacts[i].penalty * solver.gamma, RefConstants.PenaltyMin, RefConstants.PenaltyMax);
            }

            return numContacts > 0;
        }

        public override void UpdatePrimal(Rigid body, float alpha, ref Mat3 lhsLin, ref Mat3 lhsAng, ref Mat3 lhsCross, ref float3 rhsLin, ref float3 rhsAng)
        {
            float3 dqALin = bodyA.positionLin - bodyA.initialLin;
            float3 dqAAng = bodyA.positionAng - bodyA.initialAng;
            float3 dqBLin = bodyB.positionLin - bodyB.initialLin;
            float3 dqBAng = bodyB.positionAng - bodyB.initialAng;

            for (int i = 0; i < numContacts; i++)
            {
                float3 rAWorld = RefMath.Rotate(bodyA.positionAng, contacts[i].rA);
                float3 rBWorld = RefMath.Rotate(bodyB.positionAng, contacts[i].rB);

                // Compute the Taylor series approximation of the constraint function C(x) (Sec 4)
                Mat3 jALin = basis;
                Mat3 jBLin = -basis;
                Mat3 jAAng = new Mat3(math.cross(rAWorld, jALin[0]), math.cross(rAWorld, jALin[1]), math.cross(rAWorld, jALin[2]));
                Mat3 jBAng = new Mat3(math.cross(rBWorld, jBLin[0]), math.cross(rBWorld, jBLin[1]), math.cross(rBWorld, jBLin[2]));

                Mat3 K = RefMath.Diagonal(contacts[i].penalty.x, contacts[i].penalty.y, contacts[i].penalty.z);
                float3 C = contacts[i].C0 * (1 - alpha) + jALin * dqALin + jBLin * dqBLin + jAAng * dqAAng + jBAng * dqBAng;

                // Compute force
                float3 F = K * C + contacts[i].lambda;

                // Clamp normal force
                F[0] = math.min(F[0], 0.0f);

                // Clamp norm of friction forces to achieve a friction cone
                float bounds = math.abs(F[0]) * friction;
                float frictionScale = math.length(new float2(F[1], F[2]));
                if (frictionScale > bounds && frictionScale > 0)
                {
                    F[1] *= bounds / frictionScale;
                    F[2] *= bounds / frictionScale;
                }

                // Choose jacobian depending on input body
                Mat3 jLin = body == bodyA ? jALin : jBLin;
                Mat3 jAng = body == bodyA ? jAAng : jBAng;

                // Stamp into LHS
                Mat3 jLinT = RefMath.Transpose(jLin);
                Mat3 jAngT = RefMath.Transpose(jAng);
                Mat3 jAngTk = jAngT * K;

                lhsLin += jLinT * K * jLin;
                lhsAng += jAngTk * jAng;
                lhsCross += jAngTk * jLin;

                // Stamp into RHS
                rhsLin += jLinT * F;
                rhsAng += jAngT * F;
            }
        }

        public override void UpdateDual(float alpha)
        {
            float3 dqALin = bodyA.positionLin - bodyA.initialLin;
            float3 dqAAng = bodyA.positionAng - bodyA.initialAng;
            float3 dqBLin = bodyB.positionLin - bodyB.initialLin;
            float3 dqBAng = bodyB.positionAng - bodyB.initialAng;

            for (int i = 0; i < numContacts; i++)
            {
                float3 rAWorld = RefMath.Rotate(bodyA.positionAng, contacts[i].rA);
                float3 rBWorld = RefMath.Rotate(bodyB.positionAng, contacts[i].rB);

                // Compute the Taylor series approximation of the constraint function C(x) (Sec 4)
                Mat3 jALin = basis;
                Mat3 jBLin = -basis;
                Mat3 jAAng = new Mat3(math.cross(rAWorld, jALin[0]), math.cross(rAWorld, jALin[1]), math.cross(rAWorld, jALin[2]));
                Mat3 jBAng = new Mat3(math.cross(rBWorld, jBLin[0]), math.cross(rBWorld, jBLin[1]), math.cross(rBWorld, jBLin[2]));

                Mat3 K = RefMath.Diagonal(contacts[i].penalty.x, contacts[i].penalty.y, contacts[i].penalty.z);
                float3 C = contacts[i].C0 * (1 - alpha) + jALin * dqALin + jBLin * dqBLin + jAAng * dqAAng + jBAng * dqBAng;

                // Compute force
                float3 F = K * C + contacts[i].lambda;

                // Clamp normal force
                F[0] = math.min(F[0], 0.0f);

                // Clamp norm of friction forces to achieve a friction cone
                float bounds = math.abs(F[0]) * friction;
                float frictionScale = math.length(new float2(F[1], F[2]));
                if (frictionScale > bounds && frictionScale > 0)
                {
                    F[1] *= bounds / frictionScale;
                    F[2] *= bounds / frictionScale;
                }

                // Store updated force
                contacts[i].lambda = F;

                // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
                if (F[0] < 0)
                    contacts[i].penalty[0] = math.min(contacts[i].penalty[0] + solver.betaLin * math.abs(C[0]), RefConstants.PenaltyMax);
                if (frictionScale <= bounds)
                {
                    contacts[i].penalty[1] = math.min(contacts[i].penalty[1] + solver.betaLin * math.abs(C[1]), RefConstants.PenaltyMax);
                    contacts[i].penalty[2] = math.min(contacts[i].penalty[2] + solver.betaLin * math.abs(C[2]), RefConstants.PenaltyMax);
                    contacts[i].stick = math.length(new float2(C[1], C[2])) < RefConstants.StickThresh;
                }
            }
        }
    }
}
