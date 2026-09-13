// Instanced box rendering straight from the solver buffers (Graphics.RenderMeshPrimitives, one draw for every body).
// Simple lit shading (main light + ambient); colour modes: 0 palette by body, 1 graph colour group, 2 uniform.
Shader "Phys/AvbdBox"
{
    Properties
    {
        _BaseColor ("Dynamic colour", Color) = (0.55, 0.62, 0.75, 1)
        _StaticColor ("Static colour", Color) = (0.42, 0.42, 0.42, 1)
        _ColorMode ("Colour mode", Float) = 0
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" }
        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForward" }
            Cull Back
            ZWrite On

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

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
            StructuredBuffer<uint> _BodyColor;

            CBUFFER_START(UnityPerMaterial)
            float4 _BaseColor;
            float4 _StaticColor;
            float _ColorMode;
            CBUFFER_END

            struct Attributes
            {
                float3 positionOS : POSITION;
                float3 normalOS : NORMAL;
                uint instanceID : SV_InstanceID;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 normalWS : TEXCOORD0;
                float3 positionWS : TEXCOORD1;
                float3 color : TEXCOORD2;
            };

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
                if (d.mass <= 0.0) return _StaticColor.rgb;
                int mode = (int)_ColorMode;
                if (mode == 1)
                {
                    uint c = _BodyColor[id];
                    if (c > 32u) return _StaticColor.rgb;
                    if (c == 32u) return float3(1, 0, 1);              // overflow (Jacobi) group
                    return hsv(frac(c * 0.618034), 0.65, 0.95);
                }
                if (mode == 2) return _BaseColor.rgb;
                uint h = wangHash(id);
                return hsv((h & 1023u) / 1024.0, 0.35 + ((h >> 10) & 255u) / 255.0 * 0.25, 0.8 + ((h >> 18) & 255u) / 255.0 * 0.2);
            }

            Varyings vert(Attributes v)
            {
                Varyings o;
                uint id = v.instanceID;
                BodyDef d = _BodyDef[id];
                float4 q = _BodyRot[id];
                float3 p = qrotate(q, v.positionOS * d.size) + _BodyPos[id].xyz;
                o.positionWS = p;
                o.positionCS = TransformWorldToHClip(p);
                o.normalWS = qrotate(q, v.normalOS);
                o.color = bodyColor(id, d);
                return o;
            }

            half4 frag(Varyings i) : SV_Target
            {
                float3 n = normalize(i.normalWS);
                Light light = GetMainLight();
                float ndl = saturate(dot(n, light.direction));
                float3 ambient = SampleSH(n);
                float3 lit = i.color * (light.color * ndl + ambient + 0.12);
                return half4(lit, 1);
            }
            ENDHLSL
        }

        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode" = "DepthOnly" }
            ZWrite On
            ColorMask R

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct BodyDef { float3 size; float mass; float3 moment; float friction; float radius; uint flags; float pad0, pad1; };
            StructuredBuffer<float4> _BodyPos;
            StructuredBuffer<float4> _BodyRot;
            StructuredBuffer<BodyDef> _BodyDef;

            float3 qrotate(float4 q, float3 v)
            {
                float3 t = cross(q.xyz, v) * 2.0;
                return v + t * q.w + cross(q.xyz, t);
            }

            struct Attributes { float3 positionOS : POSITION; uint instanceID : SV_InstanceID; };
            struct Varyings { float4 positionCS : SV_POSITION; };

            Varyings vert(Attributes v)
            {
                Varyings o;
                BodyDef d = _BodyDef[v.instanceID];
                float3 p = qrotate(_BodyRot[v.instanceID], v.positionOS * d.size) + _BodyPos[v.instanceID].xyz;
                o.positionCS = TransformWorldToHClip(p);
                return o;
            }

            half4 frag(Varyings i) : SV_Target { return 0; }
            ENDHLSL
        }
    }
}
