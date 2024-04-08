                       �""D@"���Ȟ�Kk��r      0      cbuffer PerFrameUbo : register(b0, space0)
{
    row_major float4x4 per_frame_data_view_proj : packoffset(c0);
};


static float4 out_color;
static float4 in_color;

struct SPIRV_Cross_Input
{
    float4 in_color : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 out_color : SV_Target0;
};

void frag_main()
{
    out_color = in_color;
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_color = stage_input.in_color;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_color = out_color;
    return stage_output;
}
          #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct PerFrameUbo
{
    float4x4 view_proj;
};

struct spvDescriptorSetBuffer0
{
    constant PerFrameUbo* per_frame_data [[id(0)]];
};

struct main0_out
{
    float4 out_color [[color(0)]];
};

struct main0_in
{
    float4 in_color [[user(locn0)]];
};

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    main0_out out = {};
    out.out_color = in.in_color;
    return out;
}

    8      #                      GLSL.std.450                     main    	              G  	          G                 !                                        ;     	         
         ;  
         6               �     =           >  	      �  8                   �                            PerFrameUbo               PerFrameUbo               PerFrameUbo.view_proj                       main              �                            PerFrameUbo               PerFrameUbo               PerFrameUbo.view_proj     @                            �                            PerFrameUbo               PerFrameUbo               PerFrameUbo.view_proj                       main              �                            PerFrameUbo               PerFrameUbo               PerFrameUbo.view_proj     @                            �                            PerFrameUbo               PerFrameUbo               PerFrameUbo.view_proj                       main              �                            PerFrameUbo               PerFrameUbo               PerFrameUbo.view_proj     @                    debug3d.frag