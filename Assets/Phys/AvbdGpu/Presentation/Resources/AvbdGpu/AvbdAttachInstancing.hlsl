// Attachments: meshes drawn either relative to a body's pose from the solver buffers (a weapon in a figure's hand, a plate
// under its feet, a spell on an arrow's tip) or in world space (an explosion), one instance per record of a structured
// buffer (AttachmentRenderer). The record's rotation and scale apply in the parent frame, then the parent's pose.
#ifndef AVBD_ATTACH_INSTANCING
#define AVBD_ATTACH_INSTANCING

struct BodyDef
{
    float3 size;
    float mass;
    float3 moment;
    float friction;
    float radius;
    uint flags;
    float pad0, pad1;
};

struct Attachment
{
    float3 pos;        // in the parent's frame (body space, or the world)
    float unlit;       // 0 lit like the bodies, 1 the flat colour
    float4 rot;        // quaternion in the parent's frame
    float3 scale;      // per axis
    uint body;         // the parent body, 0xFFFFFFFF = world space
    uint color;        // RGBA8 (alpha = opacity for the transparent material)
    uint pad0, pad1, pad2;
};

StructuredBuffer<Attachment> _Attachments;
StructuredBuffer<float4> _BodyPos;
StructuredBuffer<float4> _BodyRot;
StructuredBuffer<BodyDef> _BodyDef;

float3 attachRotate(float4 q, float3 v)
{
    float3 t = cross(q.xyz, v) * 2.0;
    return v + t * q.w + cross(q.xyz, t);
}

void attachVertex(uint instanceID, float3 positionOS, float3 normalOS, out float3 positionWS, out float3 normalWS, out float4 color, out float unlit)
{
    Attachment a = _Attachments[instanceID];
    float3 local = attachRotate(a.rot, positionOS * a.scale) + a.pos;
    float3 n = attachRotate(a.rot, normalOS);
    if (a.body != 0xFFFFFFFFu)
    {
        BodyDef d = _BodyDef[a.body];
        if (d.flags & (16u | 128u)) local = 0;   // a retired (dead) or terrain parent: collapse like the body itself
        float4 q = _BodyRot[a.body];
        positionWS = attachRotate(q, local) + _BodyPos[a.body].xyz;
        normalWS = attachRotate(q, n);
    }
    else
    {
        positionWS = local;
        normalWS = n;
    }
    color = float4(a.color & 255u, (a.color >> 8) & 255u, (a.color >> 16) & 255u, a.color >> 24) / 255.0;
    unlit = a.unlit;
}

#endif
