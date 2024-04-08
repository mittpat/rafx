                       �""D@"���Ȟ���^��4      �      cbuffer Args : register(b0, space0)
{
    row_major float4x4 uniform_buffer_mvp : packoffset(c0);
};

SamplerState smp : register(s1, space0);
Texture2D<float4> tex : register(t0, space1);

static float4 gl_Position;
static float2 o_uv;
static float2 uv;
static float4 o_color;
static float4 color;
static float3 pos;

struct SPIRV_Cross_Input
{
    float3 pos : POSITION;
    float2 uv : TEXCOORD;
    float4 color : COLOR;
};

struct SPIRV_Cross_Output
{
    float2 o_uv : TEXCOORD0;
    float4 o_color : TEXCOORD1;
    float4 gl_Position : SV_Position;
};

void vert_main()
{
    o_uv = uv;
    o_color = color;
    gl_Position = mul(float4(pos, 1.0f), uniform_buffer_mvp);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    uv = stage_input.uv;
    color = stage_input.color;
    pos = stage_input.pos;
    vert_main();
    SPIRV_Cross_Output stage_output;
    stage_output.gl_Position = gl_Position;
    stage_output.o_uv = o_uv;
    stage_output.o_color = o_color;
    return stage_output;
}
    �      #include <metal_stdlib>
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
    float2 o_uv [[user(locn0)]];
    float4 o_color [[user(locn1)]];
    float4 gl_Position [[position]];
};

struct main0_in
{
    float3 pos [[attribute(0)]];
    float2 uv [[attribute(1)]];
    float4 color [[attribute(2)]];
};

vertex main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant spvDescriptorSetBuffer1& spvDescriptorSet1 [[buffer(1)]])
{
    constexpr sampler smp(filter::linear, mip_filter::linear, address::mirrored_repeat, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    out.o_uv = in.uv;
    out.o_color = in.color;
    out.gl_Position = (*spvDescriptorSet0.uniform_buffer).mvp * float4(in.pos, 1.0);
    return out;
}

    l      #     -                 GLSL.std.450                      main    	               $   G  	          G           G           G           H                H              H              H              G        H            H         #       H               G        G     "       G     !       G  $               !                                        ;     	         
         ;  
                                ;                       ;                        +                                                   ;                       +                                           ;                         "            #      "   ;  #   $      +     &     �?6               �     =           >  	      =           >        A               =     !       =  "   %   $   Q     '   %       Q     (   %      Q     )   %      P     *   '   (   )   &   �     +   !   *   A     ,         >  ,   +   �  8                   �                            Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main              �                            Args               Args               Args.mvp     @                              smp              smp                                                                                    tex              tex                         uv       TEXCOORD          color       COLOR          pos       POSITION                     �                            Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main              �                            Args               Args               Args.mvp     @                              smp              smp                                                                                    tex              tex                         uv       TEXCOORD          color       COLOR          pos       POSITION                     �                            Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main              �                            Args               Args               Args.mvp     @                              smp              smp                                                                                    tex              tex                         uv       TEXCOORD          color       COLOR          pos       POSITION             sprite.vert