                       �""D@"���Ȟ��l�жrV            cbuffer Config : register(b2, space0)
{
    row_major float4x4 config_proj : packoffset(c0);
    row_major float4x4 config_proj_inv : packoffset(c4);
    float4 config_samples[16] : packoffset(c8);
    float2 config_random_noise_offset : packoffset(c24);
    uint config_frame_index : packoffset(c24.z);
};

Texture2D<float4> depth_tex : register(t0, space0);
SamplerState smp_nearest : register(s3, space0);
Texture2D<float4> noise_tex : register(t1, space0);
SamplerState smp_linear : register(s4, space0);

static float2 inUV;
static float4 out_image;

struct SPIRV_Cross_Input
{
    float2 inUV : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 out_image : SV_Target0;
};

uint2 spvTextureSize(Texture2D<float4> Tex, uint Level, out uint Param)
{
    uint2 ret;
    Tex.GetDimensions(Level, ret.x, ret.y, Param);
    return ret;
}

float depth_vs(float depth)
{
    return (-1.0f) * (config_proj[3].z / (depth - config_proj[2].z));
}

float3 pos_vs(float2 uv, float depth)
{
    float4 pos_cs = float4(((uv * 2.0f) - 1.0f.xx) * float2(1.0f, -1.0f), depth, 1.0f);
    float4 pos_vs_1 = mul(pos_cs, config_proj_inv);
    float3 result = pos_vs_1.xyz / pos_vs_1.w.xxx;
    return result;
}

void frag_main()
{
    uint _93_dummy_parameter;
    float2 depth_texture_size = float2(int2(spvTextureSize(depth_tex, uint(0), _93_dummy_parameter)));
    float2 depth_texel_size = 1.0f.xx / depth_texture_size;
    uint _105_dummy_parameter;
    float2 noise_texture_size = float2(int2(spvTextureSize(noise_tex, uint(0), _105_dummy_parameter)));
    float2 noise_texel_size = 1.0f.xx / noise_texture_size;
    uint2 pixel = uint2(inUV * depth_texture_size);
    uint2 noise_pixel = (pixel + uint2(noise_texture_size * config_random_noise_offset)) % uint2(noise_texture_size);
    float3 noise_value = noise_tex.Sample(smp_nearest, float2(noise_pixel) * noise_texel_size).xyz;
    noise_value = (noise_value * 2.0f) - 1.0f.xxx;
    float d = depth_tex.Sample(smp_nearest, inUV).x;
    float param = d;
    float d_linear_depth = depth_vs(param);
    float2 param_1 = inUV;
    float param_2 = d;
    float3 P0 = pos_vs(param_1, param_2);
    float4 taps;
    taps.x = depth_tex.Sample(smp_nearest, inUV + float2(depth_texel_size.x * (-2.0f), 0.0f)).x;
    taps.y = depth_tex.Sample(smp_nearest, inUV + float2(depth_texel_size.x * (-1.0f), 0.0f)).x;
    taps.z = depth_tex.Sample(smp_nearest, inUV + float2(depth_texel_size.x * 2.0f, 0.0f)).x;
    taps.w = depth_tex.Sample(smp_nearest, inUV + float2(depth_texel_size.x * 1.0f, 0.0f)).x;
    float param_3 = taps.x;
    float4 taps_linear_depth;
    taps_linear_depth.x = depth_vs(param_3);
    float param_4 = taps.y;
    taps_linear_depth.y = depth_vs(param_4);
    float param_5 = taps.z;
    taps_linear_depth.z = depth_vs(param_5);
    float param_6 = taps.w;
    taps_linear_depth.w = depth_vs(param_6);
    float2 extrapolated_d_error = abs(d_linear_depth.xx - (taps_linear_depth.yw + (taps_linear_depth.yw - taps_linear_depth.xz)));
    float3 x_dir;
    if (extrapolated_d_error.x > extrapolated_d_error.y)
    {
        float2 param_7 = inUV + float2(depth_texel_size.x, 0.0f);
        float param_8 = taps.w;
        float3 P1 = pos_vs(param_7, param_8);
        x_dir = P1 - P0;
    }
    else
    {
        float2 param_9 = inUV - float2(depth_texel_size.x, 0.0f);
        float param_10 = taps.y;
        float3 P1_1 = pos_vs(param_9, param_10);
        x_dir = P0 - P1_1;
    }
    taps.x = depth_tex.Sample(smp_nearest, inUV + float2(0.0f, depth_texel_size.y * (-2.0f))).x;
    taps.y = depth_tex.Sample(smp_nearest, inUV + float2(0.0f, depth_texel_size.y * (-1.0f))).x;
    taps.z = depth_tex.Sample(smp_nearest, inUV + float2(0.0f, depth_texel_size.y * 2.0f)).x;
    taps.w = depth_tex.Sample(smp_nearest, inUV + float2(0.0f, depth_texel_size.y * 1.0f)).x;
    float param_11 = taps.x;
    taps_linear_depth.x = depth_vs(param_11);
    float param_12 = taps.y;
    taps_linear_depth.y = depth_vs(param_12);
    float param_13 = taps.z;
    taps_linear_depth.z = depth_vs(param_13);
    float param_14 = taps.w;
    taps_linear_depth.w = depth_vs(param_14);
    extrapolated_d_error = abs(d_linear_depth.xx - (taps_linear_depth.yw + (taps_linear_depth.yw - taps_linear_depth.xz)));
    float3 y_dir;
    if (extrapolated_d_error.x > extrapolated_d_error.y)
    {
        float2 param_15 = inUV + float2(0.0f, depth_texel_size.y);
        float param_16 = taps.w;
        float3 P1_2 = pos_vs(param_15, param_16);
        y_dir = P1_2 - P0;
    }
    else
    {
        float2 param_17 = inUV - float2(0.0f, depth_texel_size.y);
        float param_18 = taps.y;
        float3 P1_3 = pos_vs(param_17, param_18);
        y_dir = P0 - P1_3;
    }
    float3 normal = normalize(cross(x_dir, y_dir));
    normal = -normal;
    float3 tangent = normalize(noise_value - (normal * dot(noise_value, normal)));
    float3 bitangent = cross(normal, tangent);
    float3x3 TBN = float3x3(float3(tangent), float3(bitangent), float3(normal));
    float occlusion = 0.0f;
    int i = 0;
    for (;;)
    {
        if (i < 16)
        {
            float3 sample_pos_vs = mul(config_samples[i].xyz, TBN);
            sample_pos_vs = P0 + (sample_pos_vs * 0.4000000059604644775390625f);
            float4 sample_pos_cs = float4(sample_pos_vs, 1.0f);
            sample_pos_cs = mul(sample_pos_cs, config_proj);
            float _479 = sample_pos_cs.w;
            float4 _480 = sample_pos_cs;
            float3 _483 = _480.xyz / _479.xxx;
            sample_pos_cs.x = _483.x;
            sample_pos_cs.y = _483.y;
            sample_pos_cs.z = _483.z;
            sample_pos_cs.y *= (-1.0f);
            float4 _494 = sample_pos_cs;
            float3 _499 = (_494.xyz * 0.5f) + 0.5f.xxx;
            sample_pos_cs.x = _499.x;
            sample_pos_cs.y = _499.y;
            sample_pos_cs.z = _499.z;
            float param_19 = depth_tex.Sample(smp_linear, sample_pos_cs.xy).x;
            float sample_depth_vs = depth_vs(param_19);
            float range_adjust = smoothstep(0.0f, 1.0f, 0.100000001490116119384765625f / abs(sample_depth_vs - P0.z));
            if (sample_pos_vs.z < sample_depth_vs)
            {
                occlusion += range_adjust;
            }
            i++;
            continue;
        }
        else
        {
            break;
        }
    }
    occlusion /= 16.0f;
    float3 ambient_factor = (1.0f - occlusion).xxx;
    out_image = float4(ambient_factor, 1.0f);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    inUV = stage_input.inUV;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_image = out_image;
    return stage_output;
}
    4      #pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct Config
{
    float4x4 proj;
    float4x4 proj_inv;
    float4 samples[16];
    float2 random_noise_offset;
    uint frame_index;
};

struct spvDescriptorSetBuffer0
{
    texture2d<float> depth_tex [[id(0)]];
    texture2d<float> noise_tex [[id(1)]];
    constant Config* config [[id(2)]];
};

struct main0_out
{
    float4 out_image [[color(0)]];
};

struct main0_in
{
    float2 inUV [[user(locn0)]];
};

static inline __attribute__((always_inline))
float depth_vs(thread const float& depth, constant Config& config)
{
    return (-1.0) * (config.proj[3].z / (depth - config.proj[2].z));
}

static inline __attribute__((always_inline))
float3 pos_vs(thread const float2& uv, thread const float& depth, constant Config& config)
{
    float4 pos_cs = float4(((uv * 2.0) - float2(1.0)) * float2(1.0, -1.0), depth, 1.0);
    float4 pos_vs_1 = config.proj_inv * pos_cs;
    float3 result = pos_vs_1.xyz / float3(pos_vs_1.w);
    return result;
}

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    constexpr sampler smp_nearest(mip_filter::nearest, compare_func::never, max_anisotropy(1), lod_clamp(0.0, 0.0));
    constexpr sampler smp_linear(filter::linear, mip_filter::nearest, compare_func::never, max_anisotropy(1), lod_clamp(0.0, 0.0));
    main0_out out = {};
    float2 depth_texture_size = float2(int2(spvDescriptorSet0.depth_tex.get_width(), spvDescriptorSet0.depth_tex.get_height()));
    float2 depth_texel_size = float2(1.0) / depth_texture_size;
    float2 noise_texture_size = float2(int2(spvDescriptorSet0.noise_tex.get_width(), spvDescriptorSet0.noise_tex.get_height()));
    float2 noise_texel_size = float2(1.0) / noise_texture_size;
    uint2 pixel = uint2(in.inUV * depth_texture_size);
    uint2 noise_pixel = (pixel + uint2(noise_texture_size * (*spvDescriptorSet0.config).random_noise_offset)) % uint2(noise_texture_size);
    float3 noise_value = spvDescriptorSet0.noise_tex.sample(smp_nearest, (float2(noise_pixel) * noise_texel_size)).xyz;
    noise_value = (noise_value * 2.0) - float3(1.0);
    float d = spvDescriptorSet0.depth_tex.sample(smp_nearest, in.inUV).x;
    float param = d;
    float d_linear_depth = depth_vs(param, (*spvDescriptorSet0.config));
    float2 param_1 = in.inUV;
    float param_2 = d;
    float3 P0 = pos_vs(param_1, param_2, (*spvDescriptorSet0.config));
    float4 taps;
    taps.x = spvDescriptorSet0.depth_tex.sample(smp_nearest, (in.inUV + float2(depth_texel_size.x * (-2.0), 0.0))).x;
    taps.y = spvDescriptorSet0.depth_tex.sample(smp_nearest, (in.inUV + float2(depth_texel_size.x * (-1.0), 0.0))).x;
    taps.z = spvDescriptorSet0.depth_tex.sample(smp_nearest, (in.inUV + float2(depth_texel_size.x * 2.0, 0.0))).x;
    taps.w = spvDescriptorSet0.depth_tex.sample(smp_nearest, (in.inUV + float2(depth_texel_size.x * 1.0, 0.0))).x;
    float param_3 = taps.x;
    float4 taps_linear_depth;
    taps_linear_depth.x = depth_vs(param_3, (*spvDescriptorSet0.config));
    float param_4 = taps.y;
    taps_linear_depth.y = depth_vs(param_4, (*spvDescriptorSet0.config));
    float param_5 = taps.z;
    taps_linear_depth.z = depth_vs(param_5, (*spvDescriptorSet0.config));
    float param_6 = taps.w;
    taps_linear_depth.w = depth_vs(param_6, (*spvDescriptorSet0.config));
    float2 extrapolated_d_error = abs(float2(d_linear_depth) - (taps_linear_depth.yw + (taps_linear_depth.yw - taps_linear_depth.xz)));
    float3 x_dir;
    if (extrapolated_d_error.x > extrapolated_d_error.y)
    {
        float2 param_7 = in.inUV + float2(depth_texel_size.x, 0.0);
        float param_8 = taps.w;
        float3 P1 = pos_vs(param_7, param_8, (*spvDescriptorSet0.config));
        x_dir = P1 - P0;
    }
    else
    {
        float2 param_9 = in.inUV - float2(depth_texel_size.x, 0.0);
        float param_10 = taps.y;
        float3 P1_1 = pos_vs(param_9, param_10, (*spvDescriptorSet0.config));
        x_dir = P0 - P1_1;
    }
    taps.x = spvDescriptorSet0.depth_tex.sample(smp_nearest, (in.inUV + float2(0.0, depth_texel_size.y * (-2.0)))).x;
    taps.y = spvDescriptorSet0.depth_tex.sample(smp_nearest, (in.inUV + float2(0.0, depth_texel_size.y * (-1.0)))).x;
    taps.z = spvDescriptorSet0.depth_tex.sample(smp_nearest, (in.inUV + float2(0.0, depth_texel_size.y * 2.0))).x;
    taps.w = spvDescriptorSet0.depth_tex.sample(smp_nearest, (in.inUV + float2(0.0, depth_texel_size.y * 1.0))).x;
    float param_11 = taps.x;
    taps_linear_depth.x = depth_vs(param_11, (*spvDescriptorSet0.config));
    float param_12 = taps.y;
    taps_linear_depth.y = depth_vs(param_12, (*spvDescriptorSet0.config));
    float param_13 = taps.z;
    taps_linear_depth.z = depth_vs(param_13, (*spvDescriptorSet0.config));
    float param_14 = taps.w;
    taps_linear_depth.w = depth_vs(param_14, (*spvDescriptorSet0.config));
    extrapolated_d_error = abs(float2(d_linear_depth) - (taps_linear_depth.yw + (taps_linear_depth.yw - taps_linear_depth.xz)));
    float3 y_dir;
    if (extrapolated_d_error.x > extrapolated_d_error.y)
    {
        float2 param_15 = in.inUV + float2(0.0, depth_texel_size.y);
        float param_16 = taps.w;
        float3 P1_2 = pos_vs(param_15, param_16, (*spvDescriptorSet0.config));
        y_dir = P1_2 - P0;
    }
    else
    {
        float2 param_17 = in.inUV - float2(0.0, depth_texel_size.y);
        float param_18 = taps.y;
        float3 P1_3 = pos_vs(param_17, param_18, (*spvDescriptorSet0.config));
        y_dir = P0 - P1_3;
    }
    float3 normal = normalize(cross(x_dir, y_dir));
    normal = -normal;
    float3 tangent = normalize(noise_value - (normal * dot(noise_value, normal)));
    float3 bitangent = cross(normal, tangent);
    float3x3 TBN = float3x3(float3(tangent), float3(bitangent), float3(normal));
    float occlusion = 0.0;
    int i = 0;
    for (;;)
    {
        if (i < 16)
        {
            float3 sample_pos_vs = TBN * (*spvDescriptorSet0.config).samples[i].xyz;
            sample_pos_vs = P0 + (sample_pos_vs * 0.4000000059604644775390625);
            float4 sample_pos_cs = float4(sample_pos_vs, 1.0);
            sample_pos_cs = (*spvDescriptorSet0.config).proj * sample_pos_cs;
            float _479 = sample_pos_cs.w;
            float4 _480 = sample_pos_cs;
            float3 _483 = _480.xyz / float3(_479);
            sample_pos_cs.x = _483.x;
            sample_pos_cs.y = _483.y;
            sample_pos_cs.z = _483.z;
            sample_pos_cs.y *= (-1.0);
            float4 _494 = sample_pos_cs;
            float3 _499 = (_494.xyz * 0.5) + float3(0.5);
            sample_pos_cs.x = _499.x;
            sample_pos_cs.y = _499.y;
            sample_pos_cs.z = _499.z;
            float param_19 = spvDescriptorSet0.depth_tex.sample(smp_linear, sample_pos_cs.xy).x;
            float sample_depth_vs = depth_vs(param_19, (*spvDescriptorSet0.config));
            float range_adjust = smoothstep(0.0, 1.0, 0.100000001490116119384765625 / abs(sample_depth_vs - P0.z));
            if (sample_pos_vs.z < sample_depth_vs)
            {
                occlusion += range_adjust;
            }
            i++;
            continue;
        }
        else
        {
            break;
        }
    }
    occlusion /= 16.0;
    float3 ambient_factor = float3(1.0 - occlusion);
    out.out_image = float4(ambient_factor, 1.0);
    return out;
}

    @      #     o             2        GLSL.std.450                     main    q             G           H            H         #       H               H           H        #   @   H              H        #   �   H        #   �  H        #   �  G        G     "       G     !      G  Q   "       G  Q   !       G  U   "       G  U   !      G  b   "       G  b   !      G  q          G  �  "       G  �  !      G                !                                       +          ��                                   +                                                      ;                       +            +           +               !         +     %      +     0      @+     2     �?,     5   2      +     <         =          	 O                               P       O   ;  P   Q         S      T       S   ;  T   U         W   O     Z         ;  P   b         m            p         ;  p   q         y         +     �      �+     �         �     �        +     �        �        +     �  ���>+     �     ?;  T   �      +       ���=           ;         ,     c  2   2   ,     d  2   2   2   ,     f  �  �  �  +     k    �=     n  6               �     =  O   R   Q   =  S   V   U   V  W   X   R   V   d  O   Y   X   g  Z   [   Y      o     \   [   �     `   c  \   =  O   c   b   V  W   e   c   V   d  O   f   e   g  Z   g   f      o     h   g   �     l   c  h   =     r   q   �     t   r   \   m  m   u   t   A  y   z         =     {   z   �     |   h   {   m  m   }   |   �  m   ~   u   }   m  m   �   h   �  m   �   ~   �   V  W   �   c   V   p     �   �   �     �   �   l   W     �   �   �   O     �   �   �             �     �   �   0   �     �   �   d  V  W   �   R   V   W     �   �   r   Q     �   �       A  !   '               =     (  '  A  !   *        %       =     +  *  �     ,  �   +  �     -  (  ,  �     .     -  �     5  r   0   �     7  5  c  �     8  7  5   Q     :  8      Q     ;  8     P     <  :  ;  �   2   A  =   =     <   =     >  =  �     @  >  <  O     B  @  @            Q     D  @     P     E  D  D  D  �     F  B  E  V  W   �   R   V   Q     �   `       �     �   �   �   P     �   �   �   �     �   r   �   W     �   �   �   Q     �   �       V  W   �   R   V   �     �   �      P     �   �   �   �     �   r   �   W     �   �   �   Q     �   �       V  W   �   R   V   �     �   �   0   P     �   �   �   �     �   r   �   W     �   �   �   Q     �   �       V  W   �   R   V   P     �   �   �   �     �   r   �   W     �   �   �   Q     �   �       �     O  �   +  �     P  (  O  �     Q     P  �     Y  �   +  �     Z  (  Y  �     [     Z  �     c  �   +  �     d  (  c  �     e     d  �     m  �   +  �     n  (  m  �     o     n  P     h  Q  [  e  o  O     �   h  h        O     �   h  h         �     �   �   �   �     �   �   �   P     �   .  .  �     �   �   �        �         �   Q     �   �       Q     �   �      �  �   �   �   �   �  �       �  �   �     �  �   �     v  �   0   �     x  v  c  �     y  x  5   Q     {  y      Q     |  y     P     }  {  |  �   2   �     �  >  }  O     �  �  �            Q     �  �     P     �  �  �  �  �     �  �  �  �       �  F  �  �   �    �       r   �   �     �    0   �     �  �  c  �     �  �  5   Q     �  �      Q     �  �     P     �  �  �  �   2   �     �  >  �  O     �  �  �            Q     �  �     P     �  �  �  �  �     �  �  �  �       F  �  �  �   �  �   �     ]    �       V  W      R   V   Q     #  `      �     $  #  �   P     %  �   $  �     &  r   %  W     '     &  Q     (  '      V  W   ,  R   V   �     0  #     P     1  �   0  �     2  r   1  W     3  ,  2  Q     4  3      V  W   8  R   V   �     <  #  0   P     =  �   <  �     >  r   =  W     ?  8  >  Q     @  ?      V  W   D  R   V   P     I  �   #  �     J  r   I  W     K  D  J  Q     L  K      �     �  (  +  �     �  (  �  �     �     �  �     �  4  +  �     �  (  �  �     �     �  �     �  @  +  �     �  (  �  �     �     �  �     �  L  +  �     �  (  �  �     �     �  P     j  �  �  �  �  O     d  j  j        O     h  j  j         �     i  d  h  �     j  d  i  �     l  �   j       m        l  Q     o  m      Q     q  m     �  �   r  o  q  �  t      �  r  s  �  �  s  �     �  J  0   �     �  �  c  �     �  �  5   Q     �  �      Q     �  �     P     �  �  �  L  2   �     �  >  �  O     �  �  �            Q     �  �     P     �  �  �  �  �     �  �  �  �     �  �  F  �  t  �  �  �     �  r   I  �     �  �  0   �     �  �  c  �     �  �  5   Q     �  �      Q     �  �     P     �  �  �  4  2   �     �  >  �  O     �  �  �            Q     �  �     P     �  �  �  �  �     �  �  �  �     �  F  �  �  t  �  t  �     ^  �  s  �  �       �     D   ]  ^       �     E   �       �  �  �     �  �   �  �     �  �  �  �     �  �   �       �     E   �       �     D   �  �  P  �  �  �  �  �  �  �  �  �  �     `  �   t  b  �  �     _     t    �  �  �   �  _  �  �  �  �      �  �  �  �  �  �  A  �  �     %   _  =     �  �  O     �  �  �            �     �  �  �  �     �  �  �  �     �  F  �  Q     �  �      Q     �  �     Q     �  �     P     �  �  �  �  2   A  =   �        =     �  �  �     �  �  �  Q     �  �     O     �  �  �            P     �  �  �  �  �     �  �  �  Q     �  �      R     I  �  n      Q     �  �     �     �  �     R     P  �  I     O     �  P  P            �     �  �  �  �     �  �  f  Q     �  �      R     R  �  n      Q     �  �     R     T  �  R     =  S   �  �  V  W   �  R   �  O     �  T  T         W     �  �  �  Q       �      �         +  �       (    �            Q       F          m              2        m       	          �     
    	            1   �   2   
  �  �     �    �        �        �    �       `    �    �    �     b  `  �      �  �  �  �  �       _  <   �  �  �  �       l  `            2   l  k  2   P     $        2   >    $  �  8                                        	       depth_tex        	       depth_tex                             	       noise_tex       	       noise_tex         �                           Config              Config               Config.proj           Config.proj_inv@          Config.samples[0]�          Config.samples[1]�          Config.samples[2]�          Config.samples[3]�          Config.samples[4]�          Config.samples[5]�          Config.samples[6]�          Config.samples[7]�          Config.samples[8]          Config.samples[9]         Config.samples[10]          Config.samples[11]0         Config.samples[12]@         Config.samples[13]P         Config.samples[14]`         Config.samples[15]p         Config.random_noise_offset�         Config.frame_index�                             smp_nearest              smp_nearest                             
       smp_linear       
       smp_linear                            main                                   	       depth_tex        	       depth_tex                               	       noise_tex       	       noise_tex           �                           Config              Config               Config.proj           Config.proj_inv@          Config.samples[0]�          Config.samples[1]�          Config.samples[2]�          Config.samples[3]�          Config.samples[4]�          Config.samples[5]�          Config.samples[6]�          Config.samples[7]�          Config.samples[8]          Config.samples[9]         Config.samples[10]          Config.samples[11]0         Config.samples[12]@         Config.samples[13]P         Config.samples[14]`         Config.samples[15]p         Config.random_noise_offset�         Config.frame_index�   �                             smp_nearest              smp_nearest                                                                      
       smp_linear       
       smp_linear                                                                                              	       depth_tex        	       depth_tex                             	       noise_tex       	       noise_tex         �                           Config              Config               Config.proj           Config.proj_inv@          Config.samples[0]�          Config.samples[1]�          Config.samples[2]�          Config.samples[3]�          Config.samples[4]�          Config.samples[5]�          Config.samples[6]�          Config.samples[7]�          Config.samples[8]          Config.samples[9]         Config.samples[10]          Config.samples[11]0         Config.samples[12]@         Config.samples[13]P         Config.samples[14]`         Config.samples[15]p         Config.random_noise_offset�         Config.frame_index�                             smp_nearest              smp_nearest                             
       smp_linear       
       smp_linear                            main                                   	       depth_tex        	       depth_tex                               	       noise_tex       	       noise_tex           �                           Config              Config               Config.proj           Config.proj_inv@          Config.samples[0]�          Config.samples[1]�          Config.samples[2]�          Config.samples[3]�          Config.samples[4]�          Config.samples[5]�          Config.samples[6]�          Config.samples[7]�          Config.samples[8]          Config.samples[9]         Config.samples[10]          Config.samples[11]0         Config.samples[12]@         Config.samples[13]P         Config.samples[14]`         Config.samples[15]p         Config.random_noise_offset�         Config.frame_index�   �                             smp_nearest              smp_nearest                                                                      
       smp_linear       
       smp_linear                                                                                              	       depth_tex        	       depth_tex                             	       noise_tex       	       noise_tex         �                           Config              Config               Config.proj           Config.proj_inv@          Config.samples[0]�          Config.samples[1]�          Config.samples[2]�          Config.samples[3]�          Config.samples[4]�          Config.samples[5]�          Config.samples[6]�          Config.samples[7]�          Config.samples[8]          Config.samples[9]         Config.samples[10]          Config.samples[11]0         Config.samples[12]@         Config.samples[13]P         Config.samples[14]`         Config.samples[15]p         Config.random_noise_offset�         Config.frame_index�                             smp_nearest              smp_nearest                             
       smp_linear       
       smp_linear                            main                                   	       depth_tex        	       depth_tex                               	       noise_tex       	       noise_tex           �                           Config              Config               Config.proj           Config.proj_inv@          Config.samples[0]�          Config.samples[1]�          Config.samples[2]�          Config.samples[3]�          Config.samples[4]�          Config.samples[5]�          Config.samples[6]�          Config.samples[7]�          Config.samples[8]          Config.samples[9]         Config.samples[10]          Config.samples[11]0         Config.samples[12]@         Config.samples[13]P         Config.samples[14]`         Config.samples[15]p         Config.random_noise_offset�         Config.frame_index�   �                             smp_nearest              smp_nearest                                                                      
       smp_linear       
       smp_linear                                                          	       ssao.frag