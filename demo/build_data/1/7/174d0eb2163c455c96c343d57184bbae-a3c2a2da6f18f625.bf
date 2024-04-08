                       �""D@"���Ȟ�q�<�W�      �      cbuffer Args : register(b0, space0)
{
    row_major float4x4 uniform_buffer_mvp : packoffset(c0);
};

Texture2D<float4> tex : register(t0, space1);
SamplerState smp : register(s1, space0);

static float4 uFragColor;
static float2 o_uv;
static float4 o_color;

struct SPIRV_Cross_Input
{
    float2 o_uv : TEXCOORD0;
    float4 o_color : TEXCOORD1;
};

struct SPIRV_Cross_Output
{
    float4 uFragColor : SV_Target0;
};

void frag_main()
{
    uFragColor = tex.Sample(smp, o_uv) * o_color;
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    o_uv = stage_input.o_uv;
    o_color = stage_input.o_color;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.uFragColor = uFragColor;
    return stage_output;
}
    R      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct Args
{
    float4x4 mvp;
};

struct spvDescriptorSetBuffer0
{
    constant Args* uniform_buffer [[id(0)]];
};

struct spvDescriptorSetBuffer1
{
    texture2d<float> tex [[id(0)]];
};

struct main0_out
{
    float4 uFragColor [[color(0)]];
};

struct main0_in
{
    float2 o_uv [[user(locn0)]];
    float4 o_color [[user(locn1)]];
};

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant spvDescriptorSetBuffer1& spvDescriptorSet1 [[buffer(1)]])
{
    constexpr sampler smp(filter::linear, mip_filter::linear, address::mirrored_repeat, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    out.uFragColor = spvDescriptorSet1.tex.sample(smp, in.o_uv) * in.o_color;
    return out;
}

    �      #                      GLSL.std.450                     main    	                 G  	          G     "      G     !       G     "       G     !      G            G                !                                        ;     	       	 
                                      
   ;                              ;                 
                          ;                       ;           6               �     =  
         =           V              =           W              =           �              >  	      �  8                   �                            Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main              �                            Args               Args               Args.mvp     @                              smp              smp                                                                                    tex              tex                                    �                            Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main              �                            Args               Args               Args.mvp     @                              smp              smp                                                                                    tex              tex                                    �                            Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main              �                            Args               Args               Args.mvp     @                              smp              smp                                                                                    tex              tex                            sprite.frag