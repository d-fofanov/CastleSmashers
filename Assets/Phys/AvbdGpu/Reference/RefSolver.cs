// Port of solver.cpp from avbd-demo3d (https://github.com/savant117/avbd-demo3d).
//
// Copyright (c) 2026 Chris Giles
//
// Permission to use, copy, modify, distribute and sell this software
// and its documentation for any purpose is hereby granted without fee,
// provided that the above copyright notice appear in all copies.
// Chris Giles makes no representations about the suitability
// of this software for any purpose.
// It is provided "as is" without express or implied warranty.
//
// Deviation from the C++: gravity is a vector (the reference is Z-up with a scalar gravity along Z; Unity is Y-up).

using Unity.Mathematics;

namespace Phys.AvbdRef
{
    /// <summary>Core solver class which holds all the rigid bodies and forces, and has logic to step the simulation forward in time.
    /// Single-threaded CPU reference used as the oracle for the GPU kernels.</summary>
    public sealed class Solver
    {
        public float dt;          // Timestep
        public float3 gravity;    // Gravity vector
        public int iterations;    // Solver iterations

        public float alpha;       // Stabilization parameter
        public float betaLin;     // Penalty ramping parameter for linear constraints
        public float betaAng;     // Penalty ramping parameter for angular constraints
        public float gamma;       // Warmstarting decay parameter

        public Rigid bodies;
        public Force forces;

        public Solver()
        {
            DefaultParams();
        }

        /// <summary>Ray-cast against each dynamic OBB; returns the closest body and the hit point in its local space.</summary>
        public Rigid Pick(float3 origin, float3 dir, out float3 local)
        {
            const float epsilon = 1.0e-6f;
            float bestT = float.PositiveInfinity;
            Rigid bestBody = null;
            float3 bestLocal = float3.zero;

            for (Rigid body = bodies; body != null; body = body.next)
            {
                if (body.mass <= 0.0f)
                    continue;

                Quat invRot = RefMath.Conjugate(body.positionAng);
                float3 o = RefMath.Rotate(invRot, origin - body.positionLin);
                float3 d = RefMath.Rotate(invRot, dir);
                float3 half = body.size * 0.5f;

                float tEnter = 0.0f;
                float tExit = float.PositiveInfinity;
                bool hit = true;

                for (int i = 0; i < 3; ++i)
                {
                    if (math.abs(d[i]) < epsilon)
                    {
                        if (o[i] < -half[i] || o[i] > half[i]) { hit = false; break; }
                        continue;
                    }

                    float invD = 1.0f / d[i];
                    float t0 = (-half[i] - o[i]) * invD;
                    float t1 = (half[i] - o[i]) * invD;
                    if (t0 > t1) { float tmp = t0; t0 = t1; t1 = tmp; }

                    tEnter = math.max(tEnter, t0);
                    tExit = math.min(tExit, t1);
                    if (tEnter > tExit) { hit = false; break; }
                }

                if (!hit)
                    continue;

                float tHit = tEnter >= 0.0f ? tEnter : tExit;
                if (tHit < 0.0f)
                    continue;

                if (tHit < bestT)
                {
                    bestT = tHit;
                    bestBody = body;
                    bestLocal = o + d * tHit;
                }
            }

            local = bestLocal;
            return bestBody;
        }

        public void Clear()
        {
            while (forces != null)
                forces.Destroy();
            while (bodies != null)
                bodies.Destroy();
        }

        public void DefaultParams()
        {
            dt = 1.0f / 60.0f;
            gravity = new float3(0, -10.0f, 0);
            iterations = 10;

            // Note: in the paper, beta is suggested to be [1, 1000]. Technically, the best choice will
            // depend on the length, mass, and constraint function scales (ie units) of your simulation,
            // along with your strategy for incrementing the penalty parameters.
            // A minor upgrade from the paper is using separate betas for constraints of different units (eg linear vs angular).
            betaLin = 10000.0f;
            betaAng = 100.0f;

            // Alpha controls how much stabilization is applied. Higher values give slower and smoother
            // error correction, and lower values are more responsive and energetic.
            alpha = 0.99f;

            // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
            gamma = 0.999f;
        }

        public int BodyCount { get { int n = 0; for (Rigid b = bodies; b != null; b = b.next) n++; return n; } }
        public int ForceCount { get { int n = 0; for (Force f = forces; f != null; f = f.next) n++; return n; } }

        public void Step()
        {
            // Perform broadphase collision detection
            // This is a naive O(n^2) approach, but it is sufficient for small numbers of bodies in this sample.
            for (Rigid bodyA = bodies; bodyA != null; bodyA = bodyA.next)
            {
                for (Rigid bodyB = bodyA.next; bodyB != null; bodyB = bodyB.next)
                {
                    float3 dp = bodyA.positionLin - bodyB.positionLin;
                    float r = bodyA.radius + bodyB.radius;
                    if (math.dot(dp, dp) <= r * r && !bodyA.ConstrainedTo(bodyB))
                        new Manifold(this, bodyA, bodyB);
                }
            }

            // Initialize and warmstart forces
            for (Force force = forces; force != null;)
            {
                // Initialization can include caching anything that is constant over the step
                if (!force.Initialize())
                {
                    // Force has returned false meaning it is inactive, so remove it from the solver
                    Force next = force.next;
                    force.Destroy();
                    force = next;
                }
                else
                    force = force.next;
            }

            float gravityMag = math.length(gravity);
            float3 gravityDir = gravityMag > 0 ? gravity / gravityMag : float3.zero;

            // Initialize and warmstart bodies (ie primal variables)
            for (Rigid body = bodies; body != null; body = body.next)
            {
                // Compute inertial position (Eq 2)
                body.inertialLin = body.positionLin + body.velocityLin * dt;
                if (body.mass > 0)
                    body.inertialLin += gravity * (dt * dt);
                body.inertialAng = body.positionAng + body.velocityAng * dt;

                // Adaptive warmstart (See original VBD paper)
                float3 accel = (body.velocityLin - body.prevVelocityLin) / dt;
                float accelExt = math.dot(accel, gravityDir);
                float accelWeight = RefMath.Clamp(accelExt / gravityMag, 0.0f, 1.0f);
                if (!math.isfinite(accelWeight))
                    accelWeight = 0.0f;

                // Save initial position (x-) and compute warmstarted position (See original VBD paper)
                body.initialLin = body.positionLin;
                body.initialAng = body.positionAng;
                if (body.mass > 0)
                {
                    body.positionLin = body.positionLin + body.velocityLin * dt + gravity * (accelWeight * dt * dt);
                    body.positionAng = body.positionAng + body.velocityAng * dt;
                }
            }

            // Main solver loop
            for (int it = 0; it < iterations; it++)
            {
                // Primal update
                for (Rigid body = bodies; body != null; body = body.next)
                {
                    // Skip static / kinematic bodies
                    if (body.mass <= 0)
                        continue;

                    // Initialize left and right hand sides of the linear system (Eqs. 5, 6)
                    Mat3 MLin = RefMath.Diagonal(body.mass, body.mass, body.mass);
                    Mat3 MAng = RefMath.Diagonal(body.moment.x, body.moment.y, body.moment.z);

                    Mat3 lhsLin = MLin / (dt * dt);
                    Mat3 lhsAng = MAng / (dt * dt);
                    Mat3 lhsCross = Mat3.Zero;

                    float3 rhsLin = MLin / (dt * dt) * (body.positionLin - body.inertialLin);
                    float3 rhsAng = MAng / (dt * dt) * (body.positionAng - body.inertialAng);

                    // Iterate over all forces acting on the body
                    for (Force force = body.forces; force != null; force = (force.bodyA == body) ? force.nextA : force.nextB)
                    {
                        // Stamp the force and hessian into the linear system
                        force.UpdatePrimal(body, alpha, ref lhsLin, ref lhsAng, ref lhsCross, ref rhsLin, ref rhsAng);
                    }

                    // Solve the SPD linear system using LDL and apply the update (Eq. 4)
                    RefMath.Solve(lhsLin, lhsAng, lhsCross, -rhsLin, -rhsAng, out float3 dxLin, out float3 dxAng);
                    body.positionLin = body.positionLin + dxLin;
                    body.positionAng = body.positionAng + dxAng;
                }

                // Dual update
                for (Force force = forces; force != null; force = force.next)
                {
                    force.UpdateDual(alpha);
                }
            }

            // Compute velocities (BDF1) after the final iteration
            for (Rigid body = bodies; body != null; body = body.next)
            {
                body.prevVelocityLin = body.velocityLin;
                if (body.mass > 0)
                {
                    body.velocityLin = (body.positionLin - body.initialLin) / dt;
                    body.velocityAng = (body.positionAng - body.initialAng) / dt;
                }
            }
        }
    }
}
