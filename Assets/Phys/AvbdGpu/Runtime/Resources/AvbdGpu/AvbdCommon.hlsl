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
#define ALIGN_MIN_SPEED 1.0     // below this speed an align-to-velocity body keeps its orientation (a stuck arrow)
#define HIT_MIN_SPEED 2.0       // a projectile slower than this does not count as a hit in the event words

// Sleep word (_BodySleep, GPU-owned): bit 31 = asleep, bit 30 = woken by a touch this step (its manifolds were carried, not
// recomputed, so it may not fall asleep again before the next step), bits 0..29 = steps at rest within the rest anchor.
#define SLEEP_ASLEEP 0x80000000u
#define SLEEP_WOKEN 0x40000000u
#define SLEEP_COUNTER_MASK 0x3FFFFFFFu
#define SLEEP_COUNTER_MAX 0x20000000u

// Wake list entries (_WakeList.y, uploaded by the CPU): wake the body; wake the bodies of its previous constraint list
// (a retired support); a spawn (sleep word and island label reset).
#define WAKE_SELF 0u
#define WAKE_NEIGHBOURS 1u
#define WAKE_SPAWN 2u

// Body flags (BodyDef.flags, mirrored by GpuBodyDef).
#define FLAG_STATIC 1u
#define FLAG_LOCK_ROTATION 2u   // the angular degrees of freedom are frozen (3x3 primal solve, no angular velocity)
#define FLAG_HEADING 4u         // locked rotation set to the drive's yaw about +y at the start of every step
#define FLAG_ALIGN_VELOCITY 8u  // locked rotation set to point the body's +z along its velocity (arrows)
#define FLAG_DEAD 16u           // retired slot: no collisions, no update, not drawn; reused by a later spawn
#define FLAG_REPORT_EVENTS 32u  // the narrowphase records the kinds of bodies this one touches in _BodyEvents
#define FLAG_DRIVEN 64u         // the body has a BodyDrive record (external force or motor)
#define FLAG_TERRAIN 128u       // the terrain slot: static, identity pose, in no grid cell, the partner of every terrain manifold
#define KIND_SHIFT 8            // bits 8-9: 0 plain, 1 unit, 2 projectile (event attribution only)
#define KIND_MASK (3u << KIND_SHIFT)
#define KIND_UNIT 1u
#define KIND_PROJECTILE 2u
#define bodyKind(f) (((f) & KIND_MASK) >> KIND_SHIFT)

// Drive modes (BodyDrive.mode).
#define DRIVE_NONE 0u
#define DRIVE_FORCE 1u          // constant world-space force
#define DRIVE_TO_POINT 2u       // force of constant magnitude towards a fixed point
#define DRIVE_MOTOR 3u          // force towards a target velocity on the masked axes, capped

// Event bits (_BodyEvents, sticky until the slot is respawned): kinds touched, then the number of new manifolds with projectiles.
#define EVENT_TOUCH_STATIC 1u
#define EVENT_TOUCH_BODY 2u
#define EVENT_TOUCH_UNIT 4u
#define EVENT_TOUCH_PROJECTILE 8u
#define EVENT_HITS_SHIFT 8

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
#define CNT_TERRAIN 9           // manifolds against the terrain (a subset of CNT_MANIFOLDS)
#define CNT_SLEEPING 10         // bodies asleep at the end of the step
#define CNT_CARRIED 11          // manifolds of sleeping bodies carried over unchanged (a subset of CNT_MANIFOLDS)
#define CNT_WOKEN 12            // sleeping bodies woken by a touch this step
#define CNT_PREV_MANIFOLDS 13   // last step's manifold count (CarrySleeping runs over last step's list)
#define CNT_COUNT 16

// Indirect dispatch argument slots (uint3 each, byte offset = slot * 12).
#define ARG_BODIES 0
#define ARG_PAIRS 1
#define ARG_MANIFOLDS 2
#define ARG_CONSTRAINTS 3
#define ARG_JOINTS 4
#define ARG_PREV_MANIFOLDS 5    // last step's manifold count (CarrySleeping)
#define ARG_WAKE_LIST 6         // the CPU wake list
#define ARG_COLOR0 8            // 33 slots: colours 0..31 and the overflow group at _ActiveColors
#define ARG_COUNT (ARG_COLOR0 + MAX_COLORS + 1)

struct BodyDef
{
    float3 size;        // full widths
    float mass;         // <= 0: static
    float3 moment;      // body-frame diagonal inertia
    float friction;
    float radius;       // bounding sphere
    uint flags;         // FLAG_*, KIND_*
    float pad0, pad1;
};

// External drive of a body (FLAG_DRIVEN), applied as an acceleration of the inertial pose in Predict.
struct BodyDrive
{
    float3 target;      // FORCE: force (N); TO_POINT: the point; MOTOR: target velocity (m/s)
    uint mode;          // DRIVE_*
    float3 mask;        // MOTOR: axes the motor acts on (1 / 0)
    float limit;        // TO_POINT: force magnitude (N); MOTOR: force cap (N)
    float yaw;          // FLAG_HEADING: heading about +y (rad), applied kinematically at the start of the step
    float pad0, pad1, pad2;
};

// A body spawned into a retired slot: the GPU-owned state is written by the SpawnBodies kernel (the definition and the
// drive are uploaded by the CPU).
struct SpawnRecord
{
    uint slot;
    uint pad0, pad1, pad2;
    float4 pos;
    float4 rot;
    float4 vel;
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
    uint _SleepSteps;           // steps at rest before an island sleeps; 0: sleeping off
    uint _WakeCount;            // entries of the CPU wake list this step
    // the terrain (AvbdTerrain.hlsl): the heightfield's sample (0, 0), spacing, resolution, max-mip size, body slot
    float2 _TerrainOrigin;
    float2 _TerrainCell;
    float2 _TerrainInvCell;
    uint _TerrainResX;
    uint _TerrainResZ;
    uint _TerrainMipX;
    uint _TerrainMipZ;
    uint _TerrainSlot;          // 0xFFFFFFFF: no terrain
    float _TerrainMaxHeight;
    // sleeping: rest thresholds (squared), the relabel step flag, the speed above which a touch wakes a whole island
    float _SleepDistSq;
    float _SleepAngleSq;
    uint _Relabel;              // 1: awake bodies restart their island labels from their own index this step
    float _WakeSpeedSq;
    uint _WakeHoldSteps;        // an island woken by a fast touch stays awake at least this long
    uint _Pad2, _Pad3, _Pad4;
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

// 3x3 LDL^T solve of the SPD system a x = b: the linear block alone, for bodies with locked rotation (the angular inertia is
// infinite, so the angular update vanishes and the cross terms drop out).
float3 solve3(float3x3 a, float3 b)
{
    float D1 = a[0][0];
    float L21 = a[1][0] / D1;
    float L31 = a[2][0] / D1;
    float D2 = a[1][1] - L21 * L21 * D1;
    float L32 = (a[2][1] - L21 * L31 * D1) / D2;
    float D3 = a[2][2] - (L31 * L31 * D1 + L32 * L32 * D2);
    float y1 = b.x;
    float y2 = b.y - L21 * y1;
    float y3 = b.z - L31 * y1 - L32 * y2;
    float3 x;
    x.z = y3 / D3;
    x.y = y2 / D2 - L32 * x.z;
    x.x = y1 / D1 - L21 * x.y - L31 * x.z;
    return x;
}

// Rotation about +y (heading).
float4 qyaw(float yaw) { return float4(0, sin(yaw * 0.5), 0, cos(yaw * 0.5)); }

// Orientation whose +z is forward and whose +y is as close to up as possible (Unity's LookRotation): the rotation matrix
// with columns (right, up, forward) converted to a quaternion.
float4 qlook(float3 forward, float3 up)
{
    float3 f = normalize(forward);
    float3 r = cross(up, f);
    float rl = length(r);
    if (rl < 1.0e-4) { up = abs(f.y) < 0.9 ? float3(0, 1, 0) : float3(0, 0, 1); r = cross(up, f); rl = length(r); }
    r /= rl;
    float3 u = cross(f, r);
    // matrix elements m[row][col], columns r, u, f
    float m00 = r.x, m01 = u.x, m02 = f.x;
    float m10 = r.y, m11 = u.y, m12 = f.y;
    float m20 = r.z, m21 = u.z, m22 = f.z;
    float trace = m00 + m11 + m22;
    float4 q;
    if (trace > 0.0)
    {
        float s = sqrt(trace + 1.0) * 2.0;
        q = float4((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s);
    }
    else if (m00 > m11 && m00 > m22)
    {
        float s = sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q = float4(0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s);
    }
    else if (m11 > m22)
    {
        float s = sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q = float4((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s);
    }
    else
    {
        float s = sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q = float4((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s);
    }
    return normalize(q);
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

// Static bodies and retired (dead) slots take no part in the solve.
bool isStatic(BodyDef d) { return d.mass <= 0.0 || (d.flags & FLAG_DEAD) != 0; }
bool isDead(BodyDef d) { return (d.flags & FLAG_DEAD) != 0; }
bool isTerrain(BodyDef d) { return (d.flags & FLAG_TERRAIN) != 0; }
bool lockedRotation(BodyDef d) { return (d.flags & (FLAG_LOCK_ROTATION | FLAG_HEADING | FLAG_ALIGN_VELOCITY)) != 0; }

// Sleeping (the sleep word of _BodySleep): an asleep body is frozen and, like a static one, takes no part in the solve;
// an active body is dynamic, alive and awake. With sleeping off no word counts as asleep.
bool asleepWord(uint w) { return _SleepSteps != 0 && (w & SLEEP_ASLEEP) != 0; }
bool activeBody(BodyDef d, uint sleepWord) { return !isStatic(d) && !asleepWord(sleepWord); }

#endif
