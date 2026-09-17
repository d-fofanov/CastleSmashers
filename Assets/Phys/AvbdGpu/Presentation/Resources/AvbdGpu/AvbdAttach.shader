// Attached and free-standing meshes drawn from a record buffer (AvbdAttachInstancing.hlsl, AttachmentRenderer): weapons,
// highlight plates, spell tips, explosions. The same lit shading as the bodies, blended toward the record's flat colour by
// its unlit weight; the blend state is a material property so that one shader serves an opaque material (with shadows)
// and a transparent one (queue Transparent, no depth write, two-sided).
Shader "Phys/AvbdAttach"
{
    Properties
    {
        _SrcBlend ("Source blend", Float) = 1
        _DstBlend ("Destination blend", Float) = 0
        _ZWrite ("Depth write", Float) = 1
        _Cull ("Cull", Float) = 2
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" }
        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForward" }
            Blend [_SrcBlend] [_DstBlend]
            ZWrite [_ZWrite]
            Cull [_Cull]

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE _MAIN_LIGHT_SHADOWS_SCREEN
            #pragma multi_compile_fragment _ _SHADOWS_SOFT _SHADOWS_SOFT_LOW _SHADOWS_SOFT_MEDIUM _SHADOWS_SOFT_HIGH
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
            #include "AvbdAttachInstancing.hlsl"

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
                float4 color : TEXCOORD2;
                float unlit : TEXCOORD3;
#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
                float4 shadowCoord : TEXCOORD4;
#endif
            };

            Varyings vert(Attributes v)
            {
                Varyings o;
                attachVertex(v.instanceID, v.positionOS, v.normalOS, o.positionWS, o.normalWS, o.color, o.unlit);
                o.positionCS = TransformWorldToHClip(o.positionWS);
#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
                o.shadowCoord = ComputeScreenPos(o.positionCS);
#endif
                return o;
            }

            half4 frag(Varyings i) : SV_Target
            {
                float3 n = normalize(i.normalWS);
#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
                float4 shadowCoord = i.shadowCoord;
#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
                float4 shadowCoord = TransformWorldToShadowCoord(i.positionWS);
#else
                float4 shadowCoord = float4(0, 0, 0, 0);
#endif
                Light light = GetMainLight(shadowCoord);
                float ndl = saturate(dot(n, light.direction)) * light.shadowAttenuation;
                float3 ambient = SampleSH(n);
                float3 lit = i.color.rgb * (light.color * ndl + ambient + 0.12);
                return half4(lerp(lit, i.color.rgb, i.unlit), i.color.a);
            }
            ENDHLSL
        }

        Pass
        {
            Name "ShadowCaster"
            Tags { "LightMode" = "ShadowCaster" }
            ZWrite On
            ZTest LEqual
            ColorMask 0
            Cull Back

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #pragma multi_compile_vertex _ _CASTING_PUNCTUAL_LIGHT_SHADOW
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Shadows.hlsl"
            #include "AvbdAttachInstancing.hlsl"

            float3 _LightDirection;
            float3 _LightPosition;

            struct Attributes { float3 positionOS : POSITION; float3 normalOS : NORMAL; uint instanceID : SV_InstanceID; };
            struct Varyings { float4 positionCS : SV_POSITION; };

            Varyings vert(Attributes v)
            {
                Varyings o;
                float3 positionWS, normalWS; float4 color; float unlit;
                attachVertex(v.instanceID, v.positionOS, v.normalOS, positionWS, normalWS, color, unlit);
#if _CASTING_PUNCTUAL_LIGHT_SHADOW
                float3 lightDirectionWS = normalize(_LightPosition - positionWS);
#else
                float3 lightDirectionWS = _LightDirection;
#endif
                o.positionCS = ApplyShadowClamping(TransformWorldToHClip(ApplyShadowBias(positionWS, normalize(normalWS), lightDirectionWS)));
                return o;
            }

            half4 frag(Varyings i) : SV_Target { return 0; }
            ENDHLSL
        }

        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode" = "DepthOnly" }
            ZWrite On
            ColorMask R
            Cull [_Cull]

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "AvbdAttachInstancing.hlsl"

            struct Attributes { float3 positionOS : POSITION; float3 normalOS : NORMAL; uint instanceID : SV_InstanceID; };
            struct Varyings { float4 positionCS : SV_POSITION; };

            Varyings vert(Attributes v)
            {
                Varyings o;
                float3 positionWS, normalWS; float4 color; float unlit;
                attachVertex(v.instanceID, v.positionOS, v.normalOS, positionWS, normalWS, color, unlit);
                o.positionCS = TransformWorldToHClip(positionWS);
                return o;
            }

            half4 frag(Varyings i) : SV_Target { return 0; }
            ENDHLSL
        }
    }
}
