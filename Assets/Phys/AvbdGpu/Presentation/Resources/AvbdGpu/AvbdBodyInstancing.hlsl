// Shared by the passes of AvbdBox.shader: solver buffers, instance decoding and body colouring.
// Every draw is Graphics.RenderMeshPrimitives with one instance per body; the instance id plus _InstanceOffset is the body index.
// _MeshScale 0: the unit cube is scaled by the body's box size (the collision shape).
// _MeshScale > 0: the mesh is drawn as is, scaled by _MeshScale and shifted by _MeshOffset (body space), e.g. the brick model.
#ifndef AVBD_BODY_INSTANCING_INCLUDED
#define AVBD_BODY_INSTANCING_INCLUDED

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

StructuredBuffer<float4> _BodyPos;
StructuredBuffer<float4> _BodyRot;
StructuredBuffer<BodyDef> _BodyDef;
StructuredBuffer<uint> _BodyColor;    // graph colour (solver output)
StructuredBuffer<uint> _BodyTint;     // RGBA8 per body set by the application; alpha 0 = untinted (hash palette)

CBUFFER_START(UnityPerMaterial)
float4 _BaseColor;
float4 _StaticColor;
float _ColorMode;
CBUFFER_END

// Per-draw (MaterialPropertyBlock)
uint _InstanceOffset;
float _MeshScale;
float3 _MeshOffset;

float3 qrotate(float4 q, float3 v)
{
    float3 t = cross(q.xyz, v) * 2.0;
    return v + t * q.w + cross(q.xyz, t);
}

uint wangHash(uint s)
{
    s = (s ^ 61u) ^ (s >> 16);
    s *= 9u;
    s = s ^ (s >> 4);
    s *= 0x27d4eb2du;
    s = s ^ (s >> 15);
    return s;
}

float3 hsv(float h, float s, float v)
{
    float3 k = float3(1.0, 2.0 / 3.0, 1.0 / 3.0);
    float3 p = abs(frac(h + k) * 6.0 - 3.0);
    return v * lerp(1.0, saturate(p - 1.0), s);
}

float3 bodyColor(uint id, BodyDef d)
{
    int mode = (int)_ColorMode;
    if (mode == 1)
    {
        uint c = _BodyColor[id];
        if (c > 32u) return _StaticColor.rgb;
        if (c == 32u) return float3(1, 0, 1);              // overflow (Jacobi) group
        return hsv(frac(c * 0.618034), 0.65, 0.95);
    }
    uint tint = _BodyTint[id];
    if ((tint >> 24) != 0u)
        return float3(tint & 255u, (tint >> 8) & 255u, (tint >> 16) & 255u) / 255.0;
    if (d.mass <= 0.0) return _StaticColor.rgb;
    if (mode == 2) return _BaseColor.rgb;
    uint h = wangHash(id);
    return hsv((h & 1023u) / 1024.0, 0.35 + ((h >> 10) & 255u) / 255.0 * 0.25, 0.8 + ((h >> 18) & 255u) / 255.0 * 0.2);
}

// World position and normal of a mesh vertex of the given instance.
void bodyVertex(uint instanceID, float3 positionOS, float3 normalOS, out float3 positionWS, out float3 normalWS, out uint id)
{
    id = instanceID + _InstanceOffset;
    BodyDef d = _BodyDef[id];
    float4 q = _BodyRot[id];
    float3 local = _MeshScale > 0.0 ? positionOS * _MeshScale + _MeshOffset : positionOS * d.size;
    positionWS = qrotate(q, local) + _BodyPos[id].xyz;
    normalWS = qrotate(q, normalOS);
}

#endif
