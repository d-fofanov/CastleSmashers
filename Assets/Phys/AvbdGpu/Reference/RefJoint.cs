// Port of joint.cpp from avbd-demo3d (https://github.com/savant117/avbd-demo3d).
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
    /// <summary>Ball-socket joint + angular lock between two rigid bodies, with optional fracture.</summary>
    public sealed class Joint : Force
    {
        public float3 rA, rB;
        public float3 C0Lin, C0Ang;
        public float3 penaltyLin, penaltyAng;
        public float3 lambdaLin, lambdaAng;
        public float stiffnessLin, stiffnessAng, fracture;
        public float torqueArm;
        public bool broken;

        static Mat3 GeometricStiffnessBallSocket(int k, float3 v)
        {
            Mat3 m = RefMath.Diagonal(-v[k], -v[k], -v[k]);
            m[0, k] += v[0];
            m[1, k] += v[1];
            m[2, k] += v[2];
            return m;
        }

        public Joint(Solver solver, Rigid bodyA, Rigid bodyB, float3 rA, float3 rB, float stiffnessLin = float.PositiveInfinity, float stiffnessAng = 0.0f, float fracture = float.PositiveInfinity)
            : base(solver, bodyA, bodyB)
        {
            this.rA = rA;
            this.rB = rB;
            this.stiffnessLin = stiffnessLin;
            this.stiffnessAng = stiffnessAng;
            this.fracture = fracture;
            broken = false;
            penaltyLin = penaltyAng = float3.zero;
            lambdaLin = lambdaAng = float3.zero;
            torqueArm = math.lengthsq((bodyA != null ? bodyA.size : float3.zero) + bodyB.size);
        }

        public override bool Initialize()
        {
            // Store constraint function at beginning of timestep C(x-)
            // Note: if bodyA is null, it is assumed that the joint connects a body to the world space position rA
            C0Lin = (bodyA != null ? RefMath.Transform(bodyA.positionLin, bodyA.positionAng, rA) : rA) - RefMath.Transform(bodyB.positionLin, bodyB.positionAng, rB);
            C0Ang = ((bodyA != null ? bodyA.positionAng : Quat.Identity) - bodyB.positionAng) * torqueArm;

            // Warmstart the dual variables and penalty parameters (Eq. 19)
            // Penalty is safely clamped to a minimum and maximum value
            lambdaLin = lambdaLin * solver.alpha * solver.gamma;
            lambdaAng = lambdaAng * solver.alpha * solver.gamma;
            penaltyLin = RefMath.Clamp(penaltyLin * solver.gamma, RefConstants.PenaltyMin, RefConstants.PenaltyMax);
            penaltyAng = RefMath.Clamp(penaltyAng * solver.gamma, RefConstants.PenaltyMin, RefConstants.PenaltyMax);

            // Clamp penalty to material stiffness
            penaltyLin = RefMath.Min(penaltyLin, stiffnessLin);
            penaltyAng = RefMath.Min(penaltyAng, stiffnessAng);

            return !broken;
        }

        public override void UpdatePrimal(Rigid body, float alpha, ref Mat3 lhsLin, ref Mat3 lhsAng, ref Mat3 lhsCross, ref float3 rhsLin, ref float3 rhsAng)
        {
            // Linear constraint
            if (math.lengthsq(penaltyLin) > 0)
            {
                // Compute constraint and jacobians
                Mat3 K = RefMath.Diagonal(penaltyLin.x, penaltyLin.y, penaltyLin.z);
                float3 C = (bodyA != null ? RefMath.Transform(bodyA.positionLin, bodyA.positionAng, rA) : rA) - RefMath.Transform(bodyB.positionLin, bodyB.positionAng, rB);

                // Stabilization
                if (float.IsInfinity(stiffnessLin))
                    C -= C0Lin * alpha;

                // Compute force
                float3 F = K * C + lambdaLin;

                // Choose jacobian depending on input body
                Mat3 jLin = body == bodyA ? Mat3.Identity : -Mat3.Identity;
                Mat3 jAng = body == bodyA ? RefMath.Skew(-RefMath.Rotate(bodyA.positionAng, rA)) : RefMath.Skew(RefMath.Rotate(bodyB.positionAng, rB));

                // Stamp into LHS
                Mat3 jLinT = RefMath.Transpose(jLin);
                Mat3 jAngT = RefMath.Transpose(jAng);
                Mat3 jAngTk = jAngT * K;

                lhsLin += jLinT * K * jLin;
                lhsAng += jAngTk * jAng;
                lhsCross += jAngTk * jLin;

                // Diagonal approximation for higher order terms
                float3 r = body == bodyA ? RefMath.Rotate(bodyA.positionAng, rA) : -RefMath.Rotate(bodyB.positionAng, rB);
                Mat3 H =
                    GeometricStiffnessBallSocket(0, r) * F[0] +
                    GeometricStiffnessBallSocket(1, r) * F[1] +
                    GeometricStiffnessBallSocket(2, r) * F[2];
                lhsAng += RefMath.Diagonalize(H);

                // Stamp into RHS
                rhsLin += jLinT * F;
                rhsAng += jAngT * F;
            }

            // Angular constraint
            if (math.lengthsq(penaltyAng) > 0)
            {
                // Compute constraint and jacobians
                Mat3 K = RefMath.Diagonal(penaltyAng.x, penaltyAng.y, penaltyAng.z);
                float3 C = ((bodyA != null ? bodyA.positionAng : Quat.Identity) - bodyB.positionAng) * torqueArm;

                // Stabilization
                if (float.IsInfinity(stiffnessAng))
                    C -= C0Ang * alpha;

                // Compute force
                float3 F = K * C + lambdaAng;

                // Choose jacobian depending on input body
                Mat3 jAng = (body == bodyA ? Mat3.Identity : -Mat3.Identity) * torqueArm;

                // Stamp into LHS
                lhsAng += RefMath.Transpose(jAng) * K * jAng;

                // Stamp into RHS
                rhsAng += RefMath.Transpose(jAng) * F;
            }
        }

        public override void UpdateDual(float alpha)
        {
            // Linear constraint
            if (math.lengthsq(penaltyLin) > 0)
            {
                // Compute constraint and jacobians
                Mat3 K = RefMath.Diagonal(penaltyLin.x, penaltyLin.y, penaltyLin.z);
                float3 C = (bodyA != null ? RefMath.Transform(bodyA.positionLin, bodyA.positionAng, rA) : rA) - RefMath.Transform(bodyB.positionLin, bodyB.positionAng, rB);

                if (float.IsInfinity(stiffnessLin))
                {
                    // Stabilization
                    C -= C0Lin * alpha;

                    // Compute force
                    float3 F = K * C + lambdaLin;

                    // Store updated force
                    lambdaLin = F;
                }

                // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
                penaltyLin = RefMath.Min(penaltyLin + math.abs(C) * solver.betaLin, math.min(stiffnessLin, RefConstants.PenaltyMax));
            }

            // Angular constraint
            if (math.lengthsq(penaltyAng) > 0)
            {
                // Compute constraint and jacobians
                Mat3 K = RefMath.Diagonal(penaltyAng.x, penaltyAng.y, penaltyAng.z);
                float3 C = ((bodyA != null ? bodyA.positionAng : Quat.Identity) - bodyB.positionAng) * torqueArm;

                if (float.IsInfinity(stiffnessAng))
                {
                    // Stabilization
                    C -= C0Ang * alpha;

                    // Compute force
                    float3 F = K * C + lambdaAng;

                    // Store updated force
                    lambdaAng = F;
                }

                // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
                penaltyAng = RefMath.Min(penaltyAng + math.abs(C) * solver.betaAng, math.min(stiffnessAng, RefConstants.PenaltyMax));
            }

            // Fracture test
            if (math.lengthsq(lambdaAng) > fracture * fracture)
            {
                penaltyLin = float3.zero;
                penaltyAng = float3.zero;
                lambdaLin = float3.zero;
                lambdaAng = float3.zero;
                broken = true;
            }
        }
    }

    /// <summary>Standard spring force.</summary>
    public sealed class Spring : Force
    {
        public float3 rA, rB;
        public float rest;
        public float stiffness;

        public Spring(Solver solver, Rigid bodyA, Rigid bodyB, float3 rA, float3 rB, float stiffness, float rest = -1)
            : base(solver, bodyA, bodyB)
        {
            this.rA = rA;
            this.rB = rB;
            this.rest = rest;
            this.stiffness = stiffness;
            if (this.rest < 0.0f)
            {
                float3 pA = RefMath.Transform(bodyA.positionLin, bodyA.positionAng, this.rA);
                float3 pB = RefMath.Transform(bodyB.positionLin, bodyB.positionAng, this.rB);
                this.rest = math.length(pA - pB);
            }
        }

        public override bool Initialize() => true;

        public override void UpdatePrimal(Rigid body, float alpha, ref Mat3 lhsLin, ref Mat3 lhsAng, ref Mat3 lhsCross, ref float3 rhsLin, ref float3 rhsAng)
        {
            float3 pA = RefMath.Transform(bodyA.positionLin, bodyA.positionAng, rA);
            float3 pB = RefMath.Transform(bodyB.positionLin, bodyB.positionAng, rB);
            float3 d = pA - pB;
            float dLen = math.length(d);
            if (dLen <= 1.0e-6f)
                return;

            float3 n = d / dLen;
            float C = dLen - rest;
            float f = stiffness * C;

            float3 rWorld;
            float3 jLin;
            float3 jAng;
            if (body == bodyA)
            {
                rWorld = RefMath.Rotate(bodyA.positionAng, rA);
                jLin = n;
                jAng = math.cross(rWorld, n);
            }
            else
            {
                rWorld = RefMath.Rotate(bodyB.positionAng, rB);
                jLin = -n;
                jAng = -math.cross(rWorld, n);
            }

            float3 F = jLin * f;
            float3 Tau = jAng * f;
            Mat3 Kll = RefMath.Outer(jLin, jLin) * stiffness;
            Mat3 Kla = RefMath.Outer(jAng, jLin) * stiffness;
            Mat3 Kaa = RefMath.Outer(jAng, jAng) * stiffness;

            lhsLin += Kll;
            lhsAng += Kaa;
            lhsCross += Kla;
            rhsLin += F;
            rhsAng += Tau;
        }

        public override void UpdateDual(float alpha) { }
    }
}
