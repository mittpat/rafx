                       à®""D@"¯ÉÈIg
óG¤            cbuffer Args : register(b0, space0)
{
    row_major float4x4 uniform_buffer_mvp : packoffset(c0);
};

Texture2D<float4> tex : register(t1, space1);
SamplerState smp : register(s0, space1);

static float2 o_uv;
static float4 uFragColor;

struct SPIRV_Cross_Input
{
    float2 o_uv : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 uFragColor : SV_Target0;
};

void frag_main()
{
    float4 color = tex.Sample(smp, o_uv);
    uFragColor = color;
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    o_uv = stage_input.o_uv;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.uFragColor = uFragColor;
    return stage_output;
}
    A      #include <metal_stdlib>
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
    texture2d<float> tex [[id(1)]];
};

struct main0_out
{
    float4 uFragColor [[color(0)]];
};

struct main0_in
{
    float2 o_uv [[user(locn0)]];
};

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant spvDescriptorSetBuffer1& spvDescriptorSet1 [[buffer(1)]])
{
    constexpr sampler smp(mip_filter::nearest, address::mirrored_repeat, compare_func::never, max_anisotropy(1), lod_clamp(0.0, 0.0));
    main0_out out = {};
    float4 color = spvDescriptorSet1.tex.sample(smp, in.o_uv);
    out.uFragColor = color;
    return out;
}

    H      #                      GLSL.std.450                     main                  G     "      G     !      G     "      G     !       G            G                 !                             	 
                                      
   ;                              ;                 
                          ;                       ;           6               ø     =  
         =           V              =           W              >        ý  8                                               Args               Args               Args.mvp                               smp              smp                                   tilemap_texture             tex                            main                                          Args               Args               Args.mvp     @                                     smp              smp                                                                               tilemap_texture             tex                                                                Args               Args               Args.mvp                               smp              smp                                   tilemap_texture             tex                            main                                          Args               Args               Args.mvp     @                                     smp              smp                                                                               tilemap_texture             tex                                                                Args               Args               Args.mvp                               smp              smp                                   tilemap_texture             tex                            main                                          Args               Args               Args.mvp     @                                     smp              smp                                                                               tilemap_texture             tex                            tile_layer.frag