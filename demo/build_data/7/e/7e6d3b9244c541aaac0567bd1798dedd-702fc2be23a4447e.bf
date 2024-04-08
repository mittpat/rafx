                       �""D@"���Ȟ�
�M}20~      b      SamplerState smp : register(s0, space0);
Texture2D<float4> debug_pip_tex : register(t1, space0);

static float4 gl_Position;
static int gl_VertexIndex;
static float2 out_texcoord;

struct SPIRV_Cross_Input
{
    uint gl_VertexIndex : SV_VertexID;
};

struct SPIRV_Cross_Output
{
    float2 out_texcoord : TEXCOORD0;
    float4 gl_Position : SV_Position;
};

void vert_main()
{
    float2 coord = float2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2));
    gl_Position = float4((coord * 2.0f) - 1.0f.xx, 0.0f, 1.0f);
    out_texcoord = float2(coord.x, 1.0f - coord.y);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    gl_VertexIndex = int(stage_input.gl_VertexIndex);
    vert_main();
    SPIRV_Cross_Output stage_output;
    stage_output.gl_Position = gl_Position;
    stage_output.out_texcoord = out_texcoord;
    return stage_output;
}
    �      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct spvDescriptorSetBuffer0
{
    texture2d<float> debug_pip_tex [[id(1)]];
};

struct main0_out
{
    float2 out_texcoord [[user(locn0)]];
    float4 gl_Position [[position]];
};

vertex main0_out main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint gl_VertexIndex [[vertex_id]])
{
    constexpr sampler smp(filter::linear, mip_filter::linear, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    float2 coord = float2(float((int(gl_VertexIndex) << 1) & 2), float(int(gl_VertexIndex) & 2));
    out.gl_Position = float4((coord * 2.0) - float2(1.0), 0.0, 1.0);
    out.out_texcoord = float2(coord.x, 1.0 - coord.y);
    return out;
}

    �      #     >                 GLSL.std.450                      main          ,   G        *   H                H              H              H              G        G  ,               !                              
                   
   ;           +  
         +  
                                 +                                                   ;           +  
          +            @+     "     �?+     %          )            +         ;  +   ,      ,     =   "   "   6               �     =  
         �  
            �  
            o           �  
            o           P              �     !          �     $   !   =   Q     &   $       Q     '   $      P     (   &   '   %   "   A  )   *         >  *   (   �     3   "      P     4      3   >  ,   4   �  8                                               smp               smp                                    debug_pip_tex              debug_pip_tex                            main                                          smp               smp                                                                          debug_pip_tex              debug_pip_tex                                                                smp               smp                                    debug_pip_tex              debug_pip_tex                            main                                          smp               smp                                                                          debug_pip_tex              debug_pip_tex                                                                smp               smp                                    debug_pip_tex              debug_pip_tex                            main                                          smp               smp                                                                          debug_pip_tex              debug_pip_tex                            debug_pip.vert