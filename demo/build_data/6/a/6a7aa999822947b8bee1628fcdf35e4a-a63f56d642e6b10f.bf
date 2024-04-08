                       ą®""D@"ÆÉČ±¹šNæ      ž      cbuffer Args : register(b2, space0)
{
    row_major float4x4 uniform_buffer_inverse_projection : packoffset(c0);
    row_major float4x4 uniform_buffer_inverse_view : packoffset(c4);
};

TextureCube<float4> skybox_tex : register(t1, space0);
SamplerState smp : register(s0, space0);

static float4 out_color;
static float3 in_texcoord;

struct SPIRV_Cross_Input
{
    float3 in_texcoord : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 out_color : SV_Target0;
};

void frag_main()
{
    out_color = skybox_tex.Sample(smp, in_texcoord);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_texcoord = stage_input.in_texcoord;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_color = out_color;
    return stage_output;
}
    å      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct Args
{
    float4x4 inverse_projection;
    float4x4 inverse_view;
};

struct spvDescriptorSetBuffer0
{
    texturecube<float> skybox_tex [[id(1)]];
    constant Args* uniform_buffer [[id(2)]];
};

struct main0_out
{
    float4 out_color [[color(0)]];
};

struct main0_in
{
    float3 in_texcoord [[user(locn0)]];
};

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    constexpr sampler smp(filter::linear, mip_filter::linear, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    out.out_color = spvDescriptorSet0.skybox_tex.sample(smp, in.in_texcoord);
    return out;
}

    H      #                      GLSL.std.450                     main    	              G  	          G     "       G     !      G     "       G     !       G                 !                                        ;     	       	 
                                      
   ;                              ;                 
                          ;           6               ų     =  
         =           V              =           W              >  	      ż  8                                               smp               smp                             
       skybox_tex       
       skybox_tex                                    Args              Args               Args.inverse_projection           Args.inverse_view@                      main                                          smp               smp                                                                   
       skybox_tex       
       skybox_tex                                      Args              Args               Args.inverse_projection           Args.inverse_view@                                                            smp               smp                             
       skybox_tex       
       skybox_tex                                    Args              Args               Args.inverse_projection           Args.inverse_view@                      main                                          smp               smp                                                                   
       skybox_tex       
       skybox_tex                                      Args              Args               Args.inverse_projection           Args.inverse_view@                                                            smp               smp                             
       skybox_tex       
       skybox_tex                                    Args              Args               Args.inverse_projection           Args.inverse_view@                      main                                          smp               smp                                                                   
       skybox_tex       
       skybox_tex                                      Args              Args               Args.inverse_projection           Args.inverse_view@                        skybox.frag