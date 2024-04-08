                       �""D@"���Ȟ���w��C      �      static float4 gl_Position;
static int gl_VertexIndex;
static float2 outUV;

struct SPIRV_Cross_Input
{
    uint gl_VertexIndex : SV_VertexID;
};

struct SPIRV_Cross_Output
{
    float2 outUV : TEXCOORD0;
    float4 gl_Position : SV_Position;
};

void vert_main()
{
    float2 points = float2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2));
    gl_Position = float4((points * 2.0f) - 1.0f.xx, 0.0f, 1.0f);
    outUV = float2(points.x, 1.0f - points.y);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    gl_VertexIndex = int(stage_input.gl_VertexIndex);
    vert_main();
    SPIRV_Cross_Output stage_output;
    stage_output.gl_Position = gl_Position;
    stage_output.outUV = outUV;
    return stage_output;
}
    �      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct main0_out
{
    float2 outUV [[user(locn0)]];
    float4 gl_Position [[position]];
};

vertex main0_out main0(uint gl_VertexIndex [[vertex_id]])
{
    main0_out out = {};
    float2 points = float2(float((int(gl_VertexIndex) << 1) & 2), float(int(gl_VertexIndex) & 2));
    out.gl_Position = float4((points * 2.0) - float2(1.0), 0.0, 1.0);
    out.outUV = float2(points.x, 1.0 - points.y);
    return out;
}

    D      #     7                 GLSL.std.450                      main          )   G        *   H                G        G  )               !                              
                   
   ;           +  
         +  
                                        ;           +  
          +           @+          �?+     "          &            (         ;  (   )      ,     6         6               �     =  
         �  
            �  
            o           �  
            o           P              �              �     !      6   Q     #   !       Q     $   !      P     %   #   $   "      A  &   '         >  '   %   �     2         P     3      2   >  )   3   �  8                                       main                                                     main                                                     main                         postprocess.vert