// Port of collide.cpp from avbd-demo3d (https://github.com/savant117/avbd-demo3d).
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
    /// <summary>OBB vs OBB contact generation: SAT over 15 axes, face clipping (up to 8 points) or edge-edge closest points.</summary>
    public static class RefCollide
    {
        const int MAX_CONTACTS = 8;
        const int MAX_POLY_VERTS = 16;
        const float SAT_AXIS_EPSILON = 1.0e-6f;
        const float PLANE_EPSILON = 1.0e-5f;
        const float CONTACT_MERGE_DIST_SQ = 1.0e-6f;

        public const int AXIS_FACE_A = 0;
        public const int AXIS_FACE_B = 1;
        public const int AXIS_EDGE = 2;

        struct OBB
        {
            public float3 center;
            public Quat rotation;
            public float3 half;
            public float3 axis0, axis1, axis2;

            public float3 Axis(int i) => i == 0 ? axis0 : i == 1 ? axis1 : axis2;
        }

        struct SatAxis
        {
            public int type;
            public int indexA;
            public int indexB;
            public float separation;
            public float3 normalAB;
            public bool valid;
        }

        struct FaceFrame
        {
            public int axisIndex;
            public float3 normal;
            public float3 center;
            public float3 u;
            public float3 v;
            public float extentU;
            public float extentV;
        }

        static OBB MakeOBB(Rigid body)
        {
            OBB box;
            box.center = body.positionLin;
            box.rotation = body.positionAng;
            box.half = body.size * 0.5f;
            box.axis0 = RefMath.Rotate(body.positionAng, new float3(1.0f, 0.0f, 0.0f));
            box.axis1 = RefMath.Rotate(body.positionAng, new float3(0.0f, 1.0f, 0.0f));
            box.axis2 = RefMath.Rotate(body.positionAng, new float3(0.0f, 0.0f, 1.0f));
            return box;
        }

        static float AbsDot(float3 a, float3 b) => math.abs(math.dot(a, b));

        static float3 SupportPoint(in OBB box, float3 dir)
        {
            float sx = math.dot(dir, box.axis0) >= 0.0f ? 1.0f : -1.0f;
            float sy = math.dot(dir, box.axis1) >= 0.0f ? 1.0f : -1.0f;
            float sz = math.dot(dir, box.axis2) >= 0.0f ? 1.0f : -1.0f;

            return box.center
                + box.axis0 * (box.half.x * sx)
                + box.axis1 * (box.half.y * sy)
                + box.axis2 * (box.half.z * sz);
        }

        static void GetFaceAxes(in OBB box, int axisIndex, out float3 u, out float3 v, out float extentU, out float extentV)
        {
            if (axisIndex == 0)
            {
                u = box.axis1; v = box.axis2; extentU = box.half.y; extentV = box.half.z;
            }
            else if (axisIndex == 1)
            {
                u = box.axis0; v = box.axis2; extentU = box.half.x; extentV = box.half.z;
            }
            else
            {
                u = box.axis0; v = box.axis1; extentU = box.half.x; extentV = box.half.y;
            }
        }

        static void BuildFaceFrame(in OBB box, int axisIndex, float3 outwardNormal, out FaceFrame frame)
        {
            float sign = math.dot(outwardNormal, box.Axis(axisIndex)) >= 0.0f ? 1.0f : -1.0f;
            frame.axisIndex = axisIndex;
            frame.normal = box.Axis(axisIndex) * sign;
            frame.center = box.center + frame.normal * box.half[axisIndex];
            GetFaceAxes(box, axisIndex, out frame.u, out frame.v, out frame.extentU, out frame.extentV);
        }

        static int ChooseIncidentFaceAxis(in OBB box, float3 referenceNormal)
        {
            int axis = 0;
            float best = -float.MaxValue;
            for (int i = 0; i < 3; ++i)
            {
                float d = AbsDot(box.Axis(i), referenceNormal);
                if (d > best)
                {
                    best = d;
                    axis = i;
                }
            }
            return axis;
        }

        static void BuildIncidentFace(in OBB box, int axisIndex, float3 referenceNormal, float3[] outVerts)
        {
            float sign = math.dot(box.Axis(axisIndex), referenceNormal) > 0.0f ? -1.0f : 1.0f;
            float3 faceNormal = box.Axis(axisIndex) * sign;
            float3 faceCenter = box.center + faceNormal * box.half[axisIndex];

            GetFaceAxes(box, axisIndex, out float3 u, out float3 v, out float extentU, out float extentV);

            outVerts[0] = faceCenter + u * extentU + v * extentV;
            outVerts[1] = faceCenter - u * extentU + v * extentV;
            outVerts[2] = faceCenter - u * extentU - v * extentV;
            outVerts[3] = faceCenter + u * extentU - v * extentV;
        }

        static int ClipPolygonAgainstPlane(float3[] inVerts, int inCount, float3 planeNormal, float planeOffset, float3[] outVerts)
        {
            if (inCount <= 0)
                return 0;

            int outCount = 0;
            float3 a = inVerts[inCount - 1];
            float da = math.dot(planeNormal, a) - planeOffset;

            for (int i = 0; i < inCount; ++i)
            {
                float3 b = inVerts[i];
                float db = math.dot(planeNormal, b) - planeOffset;

                bool aInside = da <= PLANE_EPSILON;
                bool bInside = db <= PLANE_EPSILON;

                if (aInside != bInside)
                {
                    float t = 0.0f;
                    float denom = da - db;
                    if (math.abs(denom) > SAT_AXIS_EPSILON)
                        t = RefMath.Clamp(da / denom, 0.0f, 1.0f);

                    if (outCount < MAX_POLY_VERTS)
                        outVerts[outCount++] = a + (b - a) * t;
                }

                if (bInside && outCount < MAX_POLY_VERTS)
                    outVerts[outCount++] = b;

                a = b;
                da = db;
            }

            return outCount;
        }

        static bool AddContact(Rigid bodyA, Rigid bodyB, Manifold.Contact[] contacts, ref int contactCount, float3[] contactMidpoints, float3 xA, float3 xB, int featureKey)
        {
            float3 midpoint = (xA + xB) * 0.5f;

            for (int i = 0; i < contactCount; ++i)
            {
                float3 d = midpoint - contactMidpoints[i];
                if (math.lengthsq(d) < CONTACT_MERGE_DIST_SQ)
                    return false;
            }

            if (contactCount >= MAX_CONTACTS)
                return false;

            Manifold.Contact c = default;
            c.feature = featureKey;
            c.rA = RefMath.Rotate(RefMath.Conjugate(bodyA.positionAng), xA - bodyA.positionLin);
            c.rB = RefMath.Rotate(RefMath.Conjugate(bodyB.positionAng), xB - bodyB.positionLin);
            contacts[contactCount] = c;
            contactMidpoints[contactCount] = midpoint;
            ++contactCount;

            return true;
        }

        static bool TestAxis(in OBB boxA, in OBB boxB, float3 delta, float3 axis, int type, int indexA, int indexB, ref SatAxis best)
        {
            float lenSq = math.lengthsq(axis);
            if (lenSq < SAT_AXIS_EPSILON)
                return true;

            float invLen = 1.0f / math.sqrt(lenSq);
            float3 n = axis * invLen;
            if (math.dot(n, delta) < 0.0f)
                n = -n;

            float distance = math.abs(math.dot(delta, n));

            float rA =
                boxA.half.x * AbsDot(n, boxA.axis0) +
                boxA.half.y * AbsDot(n, boxA.axis1) +
                boxA.half.z * AbsDot(n, boxA.axis2);

            float rB =
                boxB.half.x * AbsDot(n, boxB.axis0) +
                boxB.half.y * AbsDot(n, boxB.axis1) +
                boxB.half.z * AbsDot(n, boxB.axis2);

            float separation = distance - (rA + rB);
            if (separation > 0.0f)
                return false;

            if (!best.valid || separation > best.separation)
            {
                best.valid = true;
                best.type = type;
                best.indexA = indexA;
                best.indexB = indexB;
                best.separation = separation;
                best.normalAB = n;
            }

            return true;
        }

        static void SupportEdge(in OBB box, int axisIndex, float3 dir, out float3 edgeA, out float3 edgeB)
        {
            int axis1 = (axisIndex + 1) % 3;
            int axis2 = (axisIndex + 2) % 3;

            float sign1 = math.dot(dir, box.Axis(axis1)) >= 0.0f ? 1.0f : -1.0f;
            float sign2 = math.dot(dir, box.Axis(axis2)) >= 0.0f ? 1.0f : -1.0f;

            float3 edgeCenter = box.center
                + box.Axis(axis1) * (box.half[axis1] * sign1)
                + box.Axis(axis2) * (box.half[axis2] * sign2);

            edgeA = edgeCenter - box.Axis(axisIndex) * box.half[axisIndex];
            edgeB = edgeCenter + box.Axis(axisIndex) * box.half[axisIndex];
        }

        static void ClosestPointsOnSegments(float3 p0, float3 p1, float3 q0, float3 q1, out float3 c0, out float3 c1)
        {
            float3 d1 = p1 - p0;
            float3 d2 = q1 - q0;
            float3 r = p0 - q0;
            float a = math.dot(d1, d1);
            float e = math.dot(d2, d2);
            float f = math.dot(d2, r);

            float s = 0.0f;
            float t = 0.0f;

            if (a <= SAT_AXIS_EPSILON && e <= SAT_AXIS_EPSILON)
            {
                c0 = p0;
                c1 = q0;
                return;
            }

            if (a <= SAT_AXIS_EPSILON)
            {
                t = RefMath.Clamp(f / e, 0.0f, 1.0f);
            }
            else
            {
                float c = math.dot(d1, r);
                if (e <= SAT_AXIS_EPSILON)
                {
                    s = RefMath.Clamp(-c / a, 0.0f, 1.0f);
                }
                else
                {
                    float b = math.dot(d1, d2);
                    float denom = a * e - b * b;

                    if (math.abs(denom) > SAT_AXIS_EPSILON)
                        s = RefMath.Clamp((b * f - c * e) / denom, 0.0f, 1.0f);

                    t = (b * s + f) / e;

                    if (t < 0.0f)
                    {
                        t = 0.0f;
                        s = RefMath.Clamp(-c / a, 0.0f, 1.0f);
                    }
                    else if (t > 1.0f)
                    {
                        t = 1.0f;
                        s = RefMath.Clamp((b - c) / a, 0.0f, 1.0f);
                    }
                }
            }

            c0 = p0 + d1 * s;
            c1 = q0 + d2 * t;
        }

        [System.ThreadStatic] static float3[] s_Clip0;
        [System.ThreadStatic] static float3[] s_Clip1;
        [System.ThreadStatic] static float3[] s_Midpoints;

        static int BuildFaceManifold(Rigid bodyA, Rigid bodyB, in OBB boxA, in OBB boxB, bool referenceIsA, int referenceAxis, float3 normalAB, Manifold.Contact[] contacts)
        {
            OBB referenceBox = referenceIsA ? boxA : boxB;
            OBB incidentBox = referenceIsA ? boxB : boxA;
            float3 referenceOutward = referenceIsA ? normalAB : -normalAB;

            BuildFaceFrame(referenceBox, referenceAxis, referenceOutward, out FaceFrame referenceFace);

            int incidentAxis = ChooseIncidentFaceAxis(incidentBox, referenceFace.normal);

            float3[] clip0 = s_Clip0 ??= new float3[MAX_POLY_VERTS];
            float3[] clip1 = s_Clip1 ??= new float3[MAX_POLY_VERTS];
            BuildIncidentFace(incidentBox, incidentAxis, referenceFace.normal, clip0);
            int count = 4;

            float3 n0 = referenceFace.u;
            float o0 = math.dot(n0, referenceFace.center) + referenceFace.extentU;
            count = ClipPolygonAgainstPlane(clip0, count, n0, o0, clip1);
            if (count == 0)
                return 0;

            float3 n1 = -referenceFace.u;
            float o1 = math.dot(n1, referenceFace.center) + referenceFace.extentU;
            count = ClipPolygonAgainstPlane(clip1, count, n1, o1, clip0);
            if (count == 0)
                return 0;

            float3 n2 = referenceFace.v;
            float o2 = math.dot(n2, referenceFace.center) + referenceFace.extentV;
            count = ClipPolygonAgainstPlane(clip0, count, n2, o2, clip1);
            if (count == 0)
                return 0;

            float3 n3 = -referenceFace.v;
            float o3 = math.dot(n3, referenceFace.center) + referenceFace.extentV;
            count = ClipPolygonAgainstPlane(clip1, count, n3, o3, clip0);
            if (count == 0)
                return 0;

            int contactCount = 0;
            float3[] contactMidpoints = s_Midpoints ??= new float3[MAX_CONTACTS];
            int featurePrefix = (referenceIsA ? AXIS_FACE_A : AXIS_FACE_B) << 24;
            featurePrefix |= (referenceAxis & 0xFF) << 16;
            featurePrefix |= (incidentAxis & 0xFF) << 8;

            for (int i = 0; i < count && contactCount < MAX_CONTACTS; ++i)
            {
                float3 pIncident = clip0[i];
                float distance = math.dot(pIncident - referenceFace.center, referenceFace.normal);
                if (distance > PLANE_EPSILON)
                    continue;

                float3 pReference = pIncident - referenceFace.normal * distance;
                float3 xA = referenceIsA ? pReference : pIncident;
                float3 xB = referenceIsA ? pIncident : pReference;

                AddContact(bodyA, bodyB, contacts, ref contactCount, contactMidpoints, xA, xB, featurePrefix | (i & 0xFF));
            }

            if (contactCount == 0)
            {
                float3 xA = SupportPoint(boxA, normalAB);
                float3 xB = SupportPoint(boxB, -normalAB);
                AddContact(bodyA, bodyB, contacts, ref contactCount, contactMidpoints, xA, xB, featurePrefix);
            }

            return contactCount;
        }

        static int BuildEdgeContact(Rigid bodyA, Rigid bodyB, in OBB boxA, in OBB boxB, int axisA, int axisB, float3 normalAB, Manifold.Contact[] contacts)
        {
            SupportEdge(boxA, axisA, normalAB, out float3 a0, out float3 a1);
            SupportEdge(boxB, axisB, -normalAB, out float3 b0, out float3 b1);

            ClosestPointsOnSegments(a0, a1, b0, b1, out float3 xA, out float3 xB);

            int contactCount = 0;
            float3[] contactMidpoints = s_Midpoints ??= new float3[MAX_CONTACTS];
            int featureKey = (AXIS_EDGE << 24) | ((axisA & 0xFF) << 8) | (axisB & 0xFF);
            AddContact(bodyA, bodyB, contacts, ref contactCount, contactMidpoints, xA, xB, featureKey);

            if (contactCount == 0)
            {
                xA = SupportPoint(boxA, normalAB);
                xB = SupportPoint(boxB, -normalAB);
                AddContact(bodyA, bodyB, contacts, ref contactCount, contactMidpoints, xA, xB, featureKey);
            }

            return contactCount;
        }

        /// <summary>Contacts between two boxes; the basis has the normal (pointing from B to A) in row 0 and the tangents in rows 1 and 2.</summary>
        public static int Collide(Rigid bodyA, Rigid bodyB, Manifold.Contact[] contacts, out Mat3 basisOut)
        {
            OBB boxA = MakeOBB(bodyA);
            OBB boxB = MakeOBB(bodyB);
            float3 delta = boxB.center - boxA.center;

            SatAxis bestFace = default;
            bestFace.separation = -float.MaxValue;
            bestFace.valid = false;

            SatAxis bestEdge = default;
            bestEdge.separation = -float.MaxValue;
            bestEdge.valid = false;

            basisOut = default;

            for (int i = 0; i < 3; ++i)
            {
                if (!TestAxis(boxA, boxB, delta, boxA.Axis(i), AXIS_FACE_A, i, -1, ref bestFace))
                    return 0;
            }

            for (int i = 0; i < 3; ++i)
            {
                if (!TestAxis(boxA, boxB, delta, boxB.Axis(i), AXIS_FACE_B, -1, i, ref bestFace))
                    return 0;
            }

            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    float3 axis = math.cross(boxA.Axis(i), boxB.Axis(j));
                    if (!TestAxis(boxA, boxB, delta, axis, AXIS_EDGE, i, j, ref bestEdge))
                        return 0;
                }
            }

            if (!bestFace.valid)
                return 0;

            SatAxis best = bestFace;
            if (bestEdge.valid)
            {
                const float edgeRelTol = 0.95f;
                const float edgeAbsTol = 0.01f;
                if (edgeRelTol * bestEdge.separation > bestFace.separation + edgeAbsTol)
                    best = bestEdge;
            }

            basisOut = RefMath.Orthonormal(-best.normalAB);

            if (best.type == AXIS_EDGE)
                return BuildEdgeContact(bodyA, bodyB, boxA, boxB, best.indexA, best.indexB, best.normalAB, contacts);

            if (best.type == AXIS_FACE_A)
                return BuildFaceManifold(bodyA, bodyB, boxA, boxB, true, best.indexA, best.normalAB, contacts);

            return BuildFaceManifold(bodyA, bodyB, boxA, boxB, false, best.indexB, best.normalAB, contacts);
        }
    }
}
