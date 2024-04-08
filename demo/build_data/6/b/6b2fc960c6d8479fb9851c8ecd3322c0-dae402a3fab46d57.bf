                       �""D@"���Ȟ��[V�[x      H      Texture2D<float4> debug_pip_tex : register(t1, space0);
SamplerState smp : register(s0, space0);

static float4 out_color;
static float2 in_texcoord;

struct SPIRV_Cross_Input
{
    float2 in_texcoord : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 out_color : SV_Target0;
};

void frag_main()
{
    out_color = debug_pip_tex.Sample(smp, in_texcoord);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_texcoord = stage_input.in_texcoord;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_color = out_color;
    return stage_output;
}
    n      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct spvDescriptorSetBuffer0
{
    texture2d<float> debug_pip_tex [[id(1)]];
};

struct main0_out
{
    float4 out_color [[color(0)]];
};

struct main0_in
{
    float2 in_texcoord [[user(locn0)]];
};

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    constexpr sampler smp(filter::linear, mip_filter::linear, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    out.out_color = spvDescriptorSet0.debug_pip_tex.sample(smp, in.in_texcoord);
    return out;
}

    H      #                      GLSL.std.450                     main    	              G  	          G     "       G     !      G     "       G     !       G                 !                                        ;     	       	 
                                      
   ;                              ;                 
                          ;           6               �     =  
         =           V              =           W              >  	      �  8                                               smp               smp                                    debug_pip_tex              debug_pip_tex                            main                                          smp               smp                                                                          debug_pip_tex              debug_pip_tex                                                                smp               smp                                    debug_pip_tex              debug_pip_tex                            main                                          smp               smp                                                                          debug_pip_tex              debug_pip_tex                                                                smp               smp                                    debug_pip_tex              debug_pip_tex                            main                                          smp               smp                                                                          debug_pip_tex              debug_pip_tex                            debug_pip.frag