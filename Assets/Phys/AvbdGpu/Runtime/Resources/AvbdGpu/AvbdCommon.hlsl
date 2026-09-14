// Shared declarations of the AVBD GPU solver: buffer layouts (mirrored by AvbdGpuTypes.cs), the parameter constant
// buffer, quaternion algebra, the 6x6 LDL^T solve and hashing helpers.
//
// Algorithm and numerics follow avbd-demo3d (https://github.com/savant117/avbd-demo3d, Copyright (c) 2026 Chris Giles,
// permission granted to use, copy, modify, distribute and sell provided the copyright notice appears in all copies).
#ifndef AVBD_COMMON_INCLUDED
#define AVBD_COMMON_INCLUDED

#define AVBD_THREADS 64
#define PENALTY_MIN 1.0
#define PENALTY_MAX 10000000000.0
#define COLLISION_MARGIN 0.01
#define STICK_THRESH 0.00001
#define HARD_STIFFNESS 1e30
#define MAX_CONTACTS 8
#define MAX_COLORS 32
#define UNCOLORED 0xFFFFFFFFu
#define STATIC_COLOR 0xFFFFFFFEu

// Constraint reference packing (per-body CSR lists): type in the top 2 bits, index below.
#define CONS_MANIFOLD 0u
#define CONS_JOINT 1u
#define CONS_SPRING 2u
#define CONS_IGNORE 3u
#define CONS_TYPE(r) ((r) >> 30)
#define CONS_INDEX(r) ((r) & 0x3FFFFFFFu)
#define CONS_REF(t, i) (((t) << 30) | (i))

// Counter slots (_Counters) and stats slots (_Stats copies them at the end of the step).
#define CNT_PAIRS 0
#define CNT_MANIFOLDS 1
#define CNT_CONTACTS 2
#define CNT_LARGE 3
#define CNT_OVERFLOW 4          // bit mask: 1 pairs, 2 manifolds, 4 contacts, 8 cell entries, 16 large bodies, 32 unsorted cells
#define CNT_OVERFLOW_BODIES 5   // bodies that ended in the Jacobi overflow colour
#define CNT_COLORS_USED 6
#define CNT_CONSTRAINTS 7       // manifolds + joints + springs (dual / CSR index space)
#define CNT_ACTIVE_JOINTS 8
#define CNT_COUNT 16

// Indirect dispatch argument slots (uint3 each, byte offset = slot * 12).
#define ARG_BODIES 0
#define ARG_PAIRS 1
#define ARG_MANIFOLDS 2
#define ARG_CONSTRAINTS 3
#define ARG_JOINTS 4
#define ARG_COLOR0 8            // 33 slots: colours 0..31 and the overflow group at _ActiveColors
#define ARG_COUNT (ARG_COLOR0 + MAX_COLORS + 1)

struct BodyDef
{
    float3 size;        // full widths
    float mass;         // <= 0: static
    float3 moment;      // body-frame diagonal inertia
    float friction;
    float radius;       // bounding sphere
    uint flags;         // bit 0: static
    float pad0, pad1;
};

struct Manifold
{
    uint bodyA, bodyB, numContacts, contactStart;
    float friction;
    float3 normal;      // basis row 0: normal pointing from B to A
    float3 tangent1;    // basis row 1
    float3 tangent2;    // basis row 2
    float2 pad;
};

struct Contact
{
    float3 rA;          // contact offset in A's local space
    float3 rB;          // contact offset in B's local space
    float3 C0;          // constraint error at x- (basis * (xA - xB) + (margin, 0, 0))
    float3 penalty;
    float3 lambda;
    uint feature;       // feature key, bit 31 = stick
};

struct JointDef
{
    int bodyA, bodyB;   // bodyA = -1: rA is a world anchor
    float3 rA;
    float3 rB;
    float stiffnessLin, stiffnessAng, fracture, torqueArm;
    // Snap fracture (each HARD_STIFFNESS = off): the linear multiplier's pull along the snap axis (tension), its part across
    // the axis (shear) and the anchor separation each break the joint past their limit. snapAxis: +-1 x, +-2 y, +-3 z of
    // body B's frame, the direction in which B separates from A (0 = +y).
    float fractureLateral, fractureTension, breakDistance, snapAxis;
};

struct JointState
{
    float3 C0Lin, C0Ang;
    float3 penaltyLin, penaltyAng;
    float3 lambdaLin, lambdaAng;
    uint broken;
    float pad;
};

struct SpringDef
{
    int bodyA, bodyB;
    float3 rA;
    float3 rB;
    float stiffness, rest;
    float2 pad;
};

cbuffer AvbdParams
{
    float _Dt;
    float _Alpha;               // stabilisation parameter used by the warm start decay
    float _BetaLin;
    float _BetaAng;
    float _Gamma;
    float _LambdaDecay;         // alpha * gamma, or 1 with post-stabilisation
    float _CellSize;
    float _InvCellSize;
    float3 _Gravity;
    float _GravityMag;
    float3 _GravityDir;
    uint _BodyCount;
    uint _JointCount;
    uint _SpringCount;
    uint _ActiveColors;
    uint _RotatedInertia;
    uint _HashMask;             // hash table size - 1
    uint _CellMask;             // grid cell count - 1
    uint _MaxPairs;
    uint _MaxManifolds;
    uint _MaxContacts;
    uint _MaxCellEntries;
    uint _MaxLargeBodies;
    uint _LargeBodyCells;       // bodies spanning more cells than this go to the large-body list
    uint _ColorRounds;
    uint _Substep;
    uint _Pad0, _Pad1;
};

// ------------------------------------------------------------------------------------------------ quaternions

float4 qmul(float4 a, float4 b)
{
    return float4(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz), a.w * b.w - dot(a.xyz, b.xyz));
}

float4 qconj(float4 q) { return float4(-q.xyz, q.w); }
float4 qinv(float4 q) { return qconj(q) / dot(q, q); }

// Rotation vector taking b to a (world frame): 2 * (a * b^-1).xyz
float3 qsub(float4 a, float4 b) { return qmul(a, qinv(b)).xyz * 2.0; }

// Integrates a world-frame rotation vector: normalize(a + 0.5 * (w, 0) * a)
float4 qadd(float4 a, float3 w) { return normalize(a + qmul(float4(w, 0), a) * 0.5); }

float3 qrotate(float4 q, float3 v)
{
    float3 t = cross(q.xyz, v) * 2.0;
    return v + t * q.w + cross(q.xyz, t);
}

float3 qtransform(float3 p, float4 q, float3 v) { return qrotate(q, v) + p; }

// Rotation matrix R with mul(R, v) == qrotate(q, v): the columns are the rotated body axes, row i holds the i-th
// world components of those axes (so the world AABB extent is dot(abs(R[i]), half)).
float3x3 qmatrix(float4 q)
{
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;
    return float3x3(
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
}

// ------------------------------------------------------------------------------------------------ matrices

float3x3 skew3(float3 r)
{
    return float3x3(
        0, -r.z, r.y,
        r.z, 0, -r.x,
        -r.y, r.x, 0);
}

float3x3 outer3(float3 a, float3 b) { return float3x3(b * a.x, b * a.y, b * a.z); }

float3x3 diag3(float3 d) { return float3x3(d.x, 0, 0, 0, d.y, 0, 0, 0, d.z); }

float3 col3(float3x3 m, int i) { return float3(m[0][i], m[1][i], m[2][i]); }

float3x3 diagonalize3(float3x3 m) { return diag3(float3(length(col3(m, 0)), length(col3(m, 1)), length(col3(m, 2)))); }

// Normal in row 0, tangents in rows 1 and 2
float3x3 orthonormal3(float3 normal)
{
    float3 t1 = abs(normal.x) > abs(normal.z) ? float3(-normal.y, normal.x, 0) : float3(0, -normal.z, normal.y);
    t1 = normalize(t1);
    float3 t2 = cross(normal, t1);
    return float3x3(normal, t1, t2);
}

// 6x6 LDL^T solve of the SPD block system [aLin aCross^T; aCross aAng] x = b (reference maths.h solve())
void solve6(float3x3 aLin, float3x3 aAng, float3x3 aCross, float3 bLin, float3 bAng, out float3 xLin, out float3 xAng)
{
    float A11 = aLin[0][0];
    float A21 = aLin[1][0], A22 = aLin[1][1];
    float A31 = aLin[2][0], A32 = aLin[2][1], A33 = aLin[2][2];
    float A41 = aCross[0][0], A42 = aCross[0][1], A43 = aCross[0][2], A44 = aAng[0][0];
    float A51 = aCross[1][0], A52 = aCross[1][1], A53 = aCross[1][2], A54 = aAng[1][0], A55 = aAng[1][1];
    float A61 = aCross[2][0], A62 = aCross[2][1], A63 = aCross[2][2], A64 = aAng[2][0], A65 = aAng[2][1], A66 = aAng[2][2];

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

    float y1 = bLin[0];
    float y2 = bLin[1] - L21 * y1;
    float y3 = bLin[2] - L31 * y1 - L32 * y2;
    float y4 = bAng[0] - L41 * y1 - L42 * y2 - L43 * y3;
    float y5 = bAng[1] - L51 * y1 - L52 * y2 - L53 * y3 - L54 * y4;
    float y6 = bAng[2] - L61 * y1 - L62 * y2 - L63 * y3 - L64 * y4 - L65 * y5;

    float z1 = y1 / D1;
    float z2 = y2 / D2;
    float z3 = y3 / D3;
    float z4 = y4 / D4;
    float z5 = y5 / D5;
    float z6 = y6 / D6;

    xAng.z = z6;
    xAng.y = z5 - L65 * xAng.z;
    xAng.x = z4 - L54 * xAng.y - L64 * xAng.z;
    xLin.z = z3 - L43 * xAng.x - L53 * xAng.y - L63 * xAng.z;
    xLin.y = z2 - L32 * xLin.z - L42 * xAng.x - L52 * xAng.y - L62 * xAng.z;
    xLin.x = z1 - L21 * xLin.y - L31 * xLin.z - L41 * xAng.x - L51 * xAng.y - L61 * xAng.z;
}

// ------------------------------------------------------------------------------------------------ hashing

uint wangHash(uint s)
{
    s = (s ^ 61u) ^ (s >> 16);
    s *= 9u;
    s = s ^ (s >> 4);
    s *= 0x27d4eb2du;
    s = s ^ (s >> 15);
    return s;
}

uint cellHash(int3 c)
{
    uint h = (uint)c.x * 73856093u ^ (uint)c.y * 19349663u ^ (uint)c.z * 83492791u;
    return h & _CellMask;
}

uint pairHash(uint a, uint b)
{
    return wangHash(a * 0x9E3779B1u ^ wangHash(b)) & _HashMask;
}

int3 cellCoord(float3 p) { return (int3)floor(p * _InvCellSize); }

// Colouring priority: hashed index, ties broken by the index
bool higherPriority(uint i, uint j)
{
    uint hi = wangHash(i), hj = wangHash(j);
    return hi != hj ? hi > hj : i > j;
}

bool isStatic(BodyDef d) { return d.mass <= 0.0; }

#endif
