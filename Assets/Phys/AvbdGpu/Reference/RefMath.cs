// Port of maths.h from avbd-demo3d (https://github.com/savant117/avbd-demo3d).
//
// Copyright (c) 2026 Chris Giles
//
// Permission to use, copy, modify, distribute and sell this software
// and its documentation for any purpose is hereby granted without fee,
// provided that the above copyright notice appear in all copies.
// Chris Giles makes no representations about the suitability
// of this software for any purpose.
// It is provided "as is" without express or implied warranty.

using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Phys.AvbdRef
{
    /// <summary>Row-major 3x3 matrix with the reference's operator set (Unity.Mathematics.float3x3 is column-major,
    /// which makes a line-for-line port error prone).</summary>
    public struct Mat3
    {
        public float3 r0, r1, r2;

        public Mat3(float3 row0, float3 row1, float3 row2) { r0 = row0; r1 = row1; r2 = row2; }

        public Mat3(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
        {
            r0 = new float3(m00, m01, m02); r1 = new float3(m10, m11, m12); r2 = new float3(m20, m21, m22);
        }

        public float3 this[int i]
        {
            get => i == 0 ? r0 : i == 1 ? r1 : r2;
            set { if (i == 0) r0 = value; else if (i == 1) r1 = value; else r2 = value; }
        }

        public float this[int i, int j]
        {
            get => this[i][j];
            set { float3 r = this[i]; r[j] = value; this[i] = r; }
        }

        public float3 Col(int i) => new float3(r0[i], r1[i], r2[i]);

        public static readonly Mat3 Identity = new Mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        public static readonly Mat3 Zero = default;

        public static float3 operator *(Mat3 a, float3 b) => new float3(math.dot(a.r0, b), math.dot(a.r1, b), math.dot(a.r2, b));
        public static Mat3 operator +(Mat3 a, Mat3 b) => new Mat3(a.r0 + b.r0, a.r1 + b.r1, a.r2 + b.r2);
        public static Mat3 operator -(Mat3 a, Mat3 b) => new Mat3(a.r0 - b.r0, a.r1 - b.r1, a.r2 - b.r2);
        public static Mat3 operator -(Mat3 a) => new Mat3(-a.r0, -a.r1, -a.r2);
        public static Mat3 operator *(Mat3 a, float b) => new Mat3(a.r0 * b, a.r1 * b, a.r2 * b);
        public static Mat3 operator /(Mat3 a, float b) => new Mat3(a.r0 / b, a.r1 / b, a.r2 / b);

        public static Mat3 operator *(Mat3 a, Mat3 b) => new Mat3(
            new float3(math.dot(a.r0, b.Col(0)), math.dot(a.r0, b.Col(1)), math.dot(a.r0, b.Col(2))),
            new float3(math.dot(a.r1, b.Col(0)), math.dot(a.r1, b.Col(1)), math.dot(a.r1, b.Col(2))),
            new float3(math.dot(a.r2, b.Col(0)), math.dot(a.r2, b.Col(1)), math.dot(a.r2, b.Col(2))));
    }

    /// <summary>Quaternion (x, y, z, w) with the reference's operators: q - q' is a rotation vector, q + w integrates one.</summary>
    public struct Quat
    {
        public float x, y, z, w;

        public Quat(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }

        public static readonly Quat Identity = new Quat(0, 0, 0, 1);

        public float3 Vec => new float3(x, y, z);

        public float this[int i]
        {
            get => i == 0 ? x : i == 1 ? y : i == 2 ? z : w;
            set { if (i == 0) x = value; else if (i == 1) y = value; else if (i == 2) z = value; else w = value; }
        }

        public static Quat operator *(Quat a, float b) => new Quat(a.x * b, a.y * b, a.z * b, a.w * b);
        public static Quat operator /(Quat a, float b) => new Quat(a.x / b, a.y / b, a.z / b, a.w / b);

        public static Quat operator *(Quat a, Quat b) => new Quat(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);

        public static Quat operator +(Quat a, Quat b) => new Quat(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);

        /// <summary>Rotation vector taking b to a (world frame): 2 * (a * b^-1).vec.</summary>
        public static float3 operator -(Quat a, Quat b) => (a * RefMath.Inverse(b)).Vec * 2.0f;

        /// <summary>Integrates a world-frame rotation vector: normalize(a + 0.5 * (b, 0) * a).</summary>
        public static Quat operator +(Quat a, float3 b) => RefMath.Normalize(a + new Quat(b.x, b.y, b.z, 0) * a * 0.5f);

        public quaternion ToUnity() => new quaternion(x, y, z, w);
        public static Quat FromUnity(quaternion q) => new Quat(q.value.x, q.value.y, q.value.z, q.value.w);
    }

    public static class RefMath
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Rad(float deg) => deg * 0.01745329251994329577f;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Sign(float x) => x < 0 ? -1.0f : x > 0 ? 1.0f : 0.0f;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Clamp(float x, float a, float b) => math.max(a, math.min(b, x));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 Clamp(float3 v, float a, float b) => new float3(Clamp(v.x, a, b), Clamp(v.y, a, b), Clamp(v.z, a, b));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 Min(float3 a, float b) => new float3(math.min(a.x, b), math.min(a.y, b), math.min(a.z, b));

        public static float LengthSq(Quat q) => math.lengthsq(q.Vec) + q.w * q.w;
        public static float Length(Quat q) => math.sqrt(LengthSq(q));
        public static Quat Normalize(Quat q) => q / Length(q);
        public static Quat Conjugate(Quat q) => new Quat(-q.x, -q.y, -q.z, q.w);
        public static Quat Inverse(Quat q) => Conjugate(q) / LengthSq(q);

        public static Mat3 Skew(float3 r) => new Mat3(
            0, -r.z, r.y,
            r.z, 0, -r.x,
            -r.y, r.x, 0);

        public static Mat3 Outer(float3 a, float3 b) => new Mat3(b * a.x, b * a.y, b * a.z);

        public static Mat3 Transpose(Mat3 a) => new Mat3(a.Col(0), a.Col(1), a.Col(2));

        public static Mat3 Diagonal(float m00, float m11, float m22) => new Mat3(m00, 0, 0, 0, m11, 0, 0, 0, m22);

        public static float3 Rotate(Quat angle, float3 v)
        {
            float3 u = new float3(angle.x, angle.y, angle.z);
            float3 t = math.cross(u, v) * 2.0f;
            return v + t * angle.w + math.cross(u, t);
        }

        public static float3 Transform(float3 qLin, Quat qAng, float3 v) => Rotate(qAng, v) + qLin;

        /// <summary>Row-major rotation matrix of q (rows are the rotated basis vectors' components as in the reference).</summary>
        public static Mat3 Rotation(Quat q)
        {
            float x = q.x, y = q.y, z = q.z, w = q.w;
            float xx = x * x, yy = y * y, zz = z * z;
            float xy = x * y, xz = x * z, yz = y * z;
            float wx = w * x, wy = w * y, wz = w * z;
            return new Mat3(
                1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy),
                2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx),
                2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy));
        }

        /// <summary>Normal in the first row, two tangents in the second and third.</summary>
        public static Mat3 Orthonormal(float3 normal)
        {
            float3 t1 = math.abs(normal.x) > math.abs(normal.z) ? new float3(-normal.y, normal.x, 0) : new float3(0, -normal.z, normal.y);
            t1 = math.normalize(t1);
            float3 t2 = math.cross(normal, t1);
            return new Mat3(normal, t1, t2);
        }

        public static Mat3 Diagonalize(Mat3 m) => Diagonal(math.length(m.Col(0)), math.length(m.Col(1)), math.length(m.Col(2)));

        /// <summary>6x6 LDL^T solve of the SPD block system [aLin aCross^T; aCross aAng] x = b.</summary>
        public static void Solve(Mat3 aLin, Mat3 aAng, Mat3 aCross, float3 bLin, float3 bAng, out float3 xLin, out float3 xAng)
        {
            // Extract elements from lower triangle storage
            float A11 = aLin[0][0];
            float A21 = aLin[1][0], A22 = aLin[1][1];
            float A31 = aLin[2][0], A32 = aLin[2][1], A33 = aLin[2][2];
            float A41 = aCross[0][0], A42 = aCross[0][1], A43 = aCross[0][2], A44 = aAng[0][0];
            float A51 = aCross[1][0], A52 = aCross[1][1], A53 = aCross[1][2], A54 = aAng[1][0], A55 = aAng[1][1];
            float A61 = aCross[2][0], A62 = aCross[2][1], A63 = aCross[2][2], A64 = aAng[2][0], A65 = aAng[2][1], A66 = aAng[2][2];

            // Step 1: LDL^T decomposition
            float L21 = A21 / A11;
            float L31 = A31 / A11;
            float L41 = A41 / A11;
            float L51 = A51 / A11;
            float L61 = A61 / A11;

            float D1 = A11;

            float D2 = A22 - L21 * L21 * D1;

            float L32 = (A32 - L21 * L31 * D1) / D2;
            float L42 = (A42 - L21 * L41 * D1) / D2;
            float L52 = (A52 - L21 * L51 * D1) / D2;
            float L62 = (A62 - L21 * L61 * D1) / D2;

            float D3 = A33 - (L31 * L31 * D1 + L32 * L32 * D2);

            float L43 = (A43 - L31 * L41 * D1 - L32 * L42 * D2) / D3;
            float L53 = (A53 - L31 * L51 * D1 - L32 * L52 * D2) / D3;
            float L63 = (A63 - L31 * L61 * D1 - L32 * L62 * D2) / D3;

            float D4 = A44 - (L41 * L41 * D1 + L42 * L42 * D2 + L43 * L43 * D3);

            float L54 = (A54 - L41 * L51 * D1 - L42 * L52 * D2 - L43 * L53 * D3) / D4;
            float L64 = (A64 - L41 * L61 * D1 - L42 * L62 * D2 - L43 * L63 * D3) / D4;

            float D5 = A55 - (L51 * L51 * D1 + L52 * L52 * D2 + L53 * L53 * D3 + L54 * L54 * D4);

            float L65 = (A65 - L51 * L61 * D1 - L52 * L62 * D2 - L53 * L63 * D3 - L54 * L64 * D4) / D5;

            float D6 = A66 - (L61 * L61 * D1 + L62 * L62 * D2 + L63 * L63 * D3 + L64 * L64 * D4 + L65 * L65 * D5);

            // Step 2: Forward substitution: Solve Ly = b
            float y1 = bLin[0];
            float y2 = bLin[1] - L21 * y1;
            float y3 = bLin[2] - L31 * y1 - L32 * y2;
            float y4 = bAng[0] - L41 * y1 - L42 * y2 - L43 * y3;
            float y5 = bAng[1] - L51 * y1 - L52 * y2 - L53 * y3 - L54 * y4;
            float y6 = bAng[2] - L61 * y1 - L62 * y2 - L63 * y3 - L64 * y4 - L65 * y5;

            // Step 3: Diagonal solve: Solve Dz = y
            float z1 = y1 / D1;
            float z2 = y2 / D2;
            float z3 = y3 / D3;
            float z4 = y4 / D4;
            float z5 = y5 / D5;
            float z6 = y6 / D6;

            // Step 4: Backward substitution: Solve L^T x = z
            xAng = default;
            xLin = default;
            xAng[2] = z6;
            xAng[1] = z5 - L65 * xAng[2];
            xAng[0] = z4 - L54 * xAng[1] - L64 * xAng[2];
            xLin[2] = z3 - L43 * xAng[0] - L53 * xAng[1] - L63 * xAng[2];
            xLin[1] = z2 - L32 * xLin[2] - L42 * xAng[0] - L52 * xAng[1] - L62 * xAng[2];
            xLin[0] = z1 - L21 * xLin[1] - L31 * xLin[2] - L41 * xAng[0] - L51 * xAng[1] - L61 * xAng[2];
        }
    }
}
