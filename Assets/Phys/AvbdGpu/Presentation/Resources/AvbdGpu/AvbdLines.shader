// Debug lines (contacts, joints, springs) drawn from a GPU-written vertex buffer with Graphics.RenderPrimitivesIndirect.
Shader "Phys/AvbdLines"
{
    SubShader
    {
        Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry+10" }
        Pass
        {
            Name "Lines"
            Tags { "LightMode" = "UniversalForward" }
            ZTest LEqual
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct DebugVertex { float4 position; float4 color; };
            StructuredBuffer<DebugVertex> _DebugVerts;

            struct Varyings { float4 positionCS : SV_POSITION; float4 color : TEXCOORD0; };

            Varyings vert(uint vertexID : SV_VertexID)
            {
                Varyings o;
                DebugVertex v = _DebugVerts[vertexID];
                o.positionCS = TransformWorldToHClip(v.position.xyz);
                o.color = v.color;
                return o;
            }

            half4 frag(Varyings i) : SV_Target { return half4(i.color.rgb, 1); }
            ENDHLSL
        }
    }
}
