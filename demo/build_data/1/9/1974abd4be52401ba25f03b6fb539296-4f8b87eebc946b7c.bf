                       à®""D@"¯ÉÈ=_9à      Q      cbuffer Args : register(b0, space0)
{
    row_major float4x4 uniform_buffer_mvp : packoffset(c0);
};

SamplerState smp : register(s1, space0);
Texture2D<float4> tex : register(t0, space1);

static float4 gl_Position;
static float2 uv;
static float2 in_uv;
static float4 color;
static float4 in_color;
static float2 pos;

struct SPIRV_Cross_Input
{
    float2 pos : POSITION;
    float2 in_uv : TEXCOORD;
    float4 in_color : COLOR;
};

struct SPIRV_Cross_Output
{
    float2 uv : TEXCOORD0;
    float4 color : TEXCOORD1;
    float4 gl_Position : SV_Position;
};

float3 srgb_to_linear(float3 srgb)
{
    bool3 cutoff = bool3(srgb.x < 0.040449999272823333740234375f.xxx.x, srgb.y < 0.040449999272823333740234375f.xxx.y, srgb.z < 0.040449999272823333740234375f.xxx.z);
    float3 higher = pow((srgb + 0.054999999701976776123046875f.xxx) / 1.05499994754791259765625f.xxx, 2.400000095367431640625f.xxx);
    float3 lower = srgb / 12.9200000762939453125f.xxx;
    return float3(cutoff.x ? lower.x : higher.x, cutoff.y ? lower.y : higher.y, cutoff.z ? lower.z : higher.z);
}

void vert_main()
{
    uv = in_uv;
    float3 param = float3(in_color.xyz);
    color = float4(srgb_to_linear(param), in_color.w);
    gl_Position = mul(float4(pos.x, pos.y, 0.0f, 1.0f), uniform_buffer_mvp);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_uv = stage_input.in_uv;
    in_color = stage_input.in_color;
    pos = stage_input.pos;
    vert_main();
    SPIRV_Cross_Output stage_output;
    stage_output.gl_Position = gl_Position;
    stage_output.uv = uv;
    stage_output.color = color;
    return stage_output;
}
          #pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
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
    float2 uv [[user(locn0)]];
    float4 color [[user(locn1)]];
    float4 gl_Position [[position]];
};

struct main0_in
{
    float2 pos [[attribute(0)]];
    float2 in_uv [[attribute(1)]];
    float4 in_color [[attribute(2)]];
};

static inline __attribute__((always_inline))
float3 srgb_to_linear(thread const float3& srgb)
{
    bool3 cutoff = srgb < float3(0.040449999272823333740234375);
    float3 higher = pow((srgb + float3(0.054999999701976776123046875)) / float3(1.05499994754791259765625), float3(2.400000095367431640625));
    float3 lower = srgb / float3(12.9200000762939453125);
    return select(higher, lower, cutoff);
}

vertex main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant spvDescriptorSetBuffer1& spvDescriptorSet1 [[buffer(1)]])
{
    constexpr sampler smp(filter::linear, mip_filter::linear, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    out.uv = in.in_uv;
    float3 param = float3(in.in_color.xyz);
    out.color = float4(srgb_to_linear(param), in.in_color.w);
    out.gl_Position = (*spvDescriptorSet0.uniform_buffer).mvp * float4(in.pos.x, in.pos.y, 0.0, 1.0);
    return out;
}

    Ì      #     z                 GLSL.std.450                      main    -   /   3   5   J   T   G  -          G  /         G  3         G  5         H  H              H  H            H  H            H  H            G  H      H  N          H  N       #       H  N             G  N      G  P   "       G  P   !       G  T               !                                            +        æ®%=,                 +        ®Ga=,                 +        @,                   +            ,      +   ;  ,   -         .      +   ;  .   /        1            2      1   ;  2   3         4      1   ;  4   5        =           +  =   >         ?         +  =   F        G      F     H   1      G   G      I      H   ;  I   J        K          +  K   L         M   1        N   M      O      N   ;  O   P         Q      M   ;  .   T      +  =   U       +     Z       +     [     ?+     v   o§r?,     w   v   v   v   +     x   =,     y   x   x   x   6               ø     =  +   0   /   >  -   0   =  1   6   5   Q     7   6       Q     8   6      Q     9   6      P     :   7   8   9   ¸     k   :           m   :           n   m   w        o         n           q   :   y   ©     u   k   q   o   A  ?   @   5   >   =     A   @   Q     B   u       Q     C   u      Q     D   u      P  1   E   B   C   D   A   >  3   E   A  Q   R   P   L   =  M   S   R   A  ?   V   T   U   =     W   V   A  ?   X   T   F   =     Y   X   P  1   \   W   Y   Z   [     1   ]   S   \   A  2   ^   J   L   >  ^   ]   ý  8                                               Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main                                          Args               Args               Args.mvp     @                              smp              smp                                                                                 tex              tex                         in_uv       TEXCOORD          in_color       COLOR          pos       POSITION                                                 Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main                                          Args               Args               Args.mvp     @                              smp              smp                                                                                 tex              tex                         in_uv       TEXCOORD          in_color       COLOR          pos       POSITION                                                 Args               Args               Args.mvp                               smp              smp                                    tex              tex                            main                                          Args               Args               Args.mvp     @                              smp              smp                                                                                 tex              tex                         in_uv       TEXCOORD          in_color       COLOR          pos       POSITION      	       egui.vert