                       �""D@"���Ȟ��b��Tn      T      cbuffer Config : register(b3, space0)
{
    int config_tonemapper_type : packoffset(c0);
    int config_output_color_space : packoffset(c0.y);
    float config_max_color_component_value : packoffset(c0.z);
};

Texture2D<float4> in_color : register(t0, space0);
SamplerState smp : register(s2, space0);
Texture2D<float4> in_blur : register(t1, space0);

static float2 inUV;
static float4 out_sdr;

struct SPIRV_Cross_Input
{
    float2 inUV : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 out_sdr : SV_Target0;
};

float3 RRT_and_ODT_fit(float3 v)
{
    float3 a = (v * (v + 0.02457859925925731658935546875f.xxx)) - 9.0537003416102379560470581054688e-05f.xxx;
    float3 b = (v * ((v * 0.98372900485992431640625f) + 0.4329510033130645751953125f.xxx)) + 0.23808099329471588134765625f.xxx;
    return a / b;
}

float3 tonemap_aces_fitted(inout float3 color)
{
    color = mul(color, float3x3(float3(0.59719002246856689453125f, 0.075999997556209564208984375f, 0.0284000001847743988037109375f), float3(0.354579985141754150390625f, 0.908339977264404296875f, 0.13382999598979949951171875f), float3(0.048229999840259552001953125f, 0.0156599991023540496826171875f, 0.837769985198974609375f)));
    float3 param = color;
    color = RRT_and_ODT_fit(param);
    color = mul(color, float3x3(float3(1.60475003719329833984375f, -0.10208000242710113525390625f, -0.00326999998651444911956787109375f), float3(-0.5310800075531005859375f, 1.108129978179931640625f, -0.07276000082492828369140625f), float3(-0.0736699998378753662109375f, -0.00604999996721744537353515625f, 1.0760200023651123046875f)));
    color = clamp(color, 0.0f.xxx, 1.0f.xxx);
    return color;
}

float3 linear_to_srgb(float3 linearRGB)
{
    bool3 cutoff = bool3(linearRGB.x < 0.003130800090730190277099609375f.xxx.x, linearRGB.y < 0.003130800090730190277099609375f.xxx.y, linearRGB.z < 0.003130800090730190277099609375f.xxx.z);
    float3 higher = (1.05499994754791259765625f.xxx * pow(linearRGB, 0.4166666567325592041015625f.xxx)) - 0.054999999701976776123046875f.xxx;
    float3 lower = linearRGB * 12.9200000762939453125f.xxx;
    return float3(cutoff.x ? lower.x : higher.x, cutoff.y ? lower.y : higher.y, cutoff.z ? lower.z : higher.z);
}

float3 tonemap_aces_film_simple(float3 x)
{
    float a = 2.5099999904632568359375f;
    float b = 0.02999999932944774627685546875f;
    float c = 2.4300000667572021484375f;
    float d = 0.589999973773956298828125f;
    float e = 0.14000000059604644775390625f;
    return clamp((x * ((x * a) + b.xxx)) / ((x * ((x * c) + d.xxx)) + e.xxx), 0.0f.xxx, 1.0f.xxx);
}

float3 tonemap_Hejl2015(float3 hdr)
{
    float4 vh = float4(hdr, 1.0f);
    float4 va = (vh * 1.434999942779541015625f) + 0.0500000007450580596923828125f.xxxx;
    float4 vf = (((vh * va) + 0.0040000001899898052215576171875f.xxxx) / ((vh * (va + 0.550000011920928955078125f.xxxx)) + 0.0491000004112720489501953125f.xxxx)) - 0.082099996507167816162109375f.xxxx;
    return vf.xyz / vf.www;
}

float3 hable_function(float3 x)
{
    return (((x * ((x * 4.0f) + 0.60000002384185791015625f.xxx)) + 0.12999999523162841796875f.xxx) / ((x * ((x * 4.0f) + 5.0f.xxx)) + 3.900000095367431640625f.xxx)) - 0.0333333350718021392822265625f.xxx;
}

float3 tonemap_hable(float3 color)
{
    float3 param = color;
    float3 numerator = hable_function(param);
    float3 param_1 = 6.0f.xxx;
    float3 denominator = hable_function(param_1);
    return numerator / denominator;
}

float3 tonemap_filmic_alu(float3 color_in)
{
    float3 color = max(color_in - 0.0040000001899898052215576171875f.xxx, 0.0f.xxx);
    color = (color * ((color * 6.19999980926513671875f) + 0.5f.xxx)) / ((color * ((color * 6.19999980926513671875f) + 1.7000000476837158203125f.xxx)) + 0.0599999986588954925537109375f.xxx);
    return color;
}

float3 visualize_value(float val)
{
    float g = 1.0f - ((0.20000000298023223876953125f * (val - 3.2360498905181884765625f)) * (val - 3.2360498905181884765625f));
    float b = val;
    float r = 1.0f - (1.0f / ((0.5f * val) - 0.5f));
    if (val > 1.0f)
    {
        b = 0.0f;
    }
    if (val < 3.0f)
    {
        r = 0.0f;
    }
    return clamp(float3(r, g, b), 0.0f.xxx, 1.0f.xxx);
}

float rgb_to_luminosity(float3 color)
{
    return dot(color, float3(0.2989999949932098388671875f, 0.58700001239776611328125f, 0.114000000059604644775390625f));
}

float3 tonemap_basic(float3 color, int tonemapper_type)
{
    switch (tonemapper_type)
    {
        case 1:
        {
            float3 param = color;
            float3 _358 = tonemap_aces_fitted(param);
            float3 param_1 = _358;
            return linear_to_srgb(param_1);
        }
        case 2:
        {
            float3 param_2 = color * 0.60000002384185791015625f;
            float3 param_3 = tonemap_aces_film_simple(param_2);
            return linear_to_srgb(param_3);
        }
        case 3:
        {
            float3 param_4 = color;
            return tonemap_Hejl2015(param_4);
        }
        case 4:
        {
            float3 param_5 = color;
            return tonemap_hable(param_5);
        }
        case 5:
        {
            float3 param_6 = color;
            return tonemap_filmic_alu(param_6);
        }
        case 6:
        {
            return color / (color + 1.0f.xxx);
        }
        case 7:
        {
            float max_val = max(color.x, max(color.y, color.z));
            float param_7 = max_val;
            return visualize_value(param_7);
        }
        case 8:
        {
            float3 param_8 = color;
            float l = rgb_to_luminosity(param_8);
            float param_9 = l;
            return visualize_value(param_9);
        }
        default:
        {
            return color;
        }
    }
}

void frag_main()
{
    float4 rgb = in_color.Sample(smp, inUV) + in_blur.Sample(smp, inUV);
    float3 color = rgb.xyz;
    if (all(bool3(color.x == 0.0f.xxx.x, color.y == 0.0f.xxx.y, color.z == 0.0f.xxx.z)))
    {
        out_sdr = float4(color, 1.0f);
        return;
    }
    float3 param = color;
    int param_1 = config_tonemapper_type;
    float3 color_srgb_linear = tonemap_basic(param, param_1);
    switch (config_output_color_space)
    {
        case 0:
        {
            out_sdr = float4(color_srgb_linear, 1.0f);
            break;
        }
        case 1:
        {
            float3 color_linear_p3 = mul(color_srgb_linear, float3x3(float3(0.822488605976104736328125f, 0.033200047910213470458984375f, 0.0170890651643276214599609375f), float3(0.1775114238262176513671875f, 0.9667999744415283203125f, 0.072411514818668365478515625f), float3(5.5511151231257827021181583404541e-17f, -1.7347234759768070944119244813919e-17f, 0.910499393939971923828125f)));
            out_sdr = float4(color_linear_p3, 1.0f);
            break;
        }
    }
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    inUV = stage_input.inUV;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_sdr = out_sdr;
    return stage_output;
}
    �      #pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct Config
{
    int tonemapper_type;
    int output_color_space;
    float max_color_component_value;
};

struct spvDescriptorSetBuffer0
{
    texture2d<float> in_color [[id(0)]];
    texture2d<float> in_blur [[id(1)]];
    constant Config* config [[id(3)]];
};

struct main0_out
{
    float4 out_sdr [[color(0)]];
};

struct main0_in
{
    float2 inUV [[user(locn0)]];
};

static inline __attribute__((always_inline))
float3 RRT_and_ODT_fit(thread const float3& v)
{
    float3 a = (v * (v + float3(0.02457859925925731658935546875))) - float3(9.0537003416102379560470581054688e-05);
    float3 b = (v * ((v * 0.98372900485992431640625) + float3(0.4329510033130645751953125))) + float3(0.23808099329471588134765625);
    return a / b;
}

static inline __attribute__((always_inline))
float3 tonemap_aces_fitted(thread float3& color)
{
    color = float3x3(float3(0.59719002246856689453125, 0.075999997556209564208984375, 0.0284000001847743988037109375), float3(0.354579985141754150390625, 0.908339977264404296875, 0.13382999598979949951171875), float3(0.048229999840259552001953125, 0.0156599991023540496826171875, 0.837769985198974609375)) * color;
    float3 param = color;
    color = RRT_and_ODT_fit(param);
    color = float3x3(float3(1.60475003719329833984375, -0.10208000242710113525390625, -0.00326999998651444911956787109375), float3(-0.5310800075531005859375, 1.108129978179931640625, -0.07276000082492828369140625), float3(-0.0736699998378753662109375, -0.00604999996721744537353515625, 1.0760200023651123046875)) * color;
    color = fast::clamp(color, float3(0.0), float3(1.0));
    return color;
}

static inline __attribute__((always_inline))
float3 linear_to_srgb(thread const float3& linearRGB)
{
    bool3 cutoff = linearRGB < float3(0.003130800090730190277099609375);
    float3 higher = (float3(1.05499994754791259765625) * pow(linearRGB, float3(0.4166666567325592041015625))) - float3(0.054999999701976776123046875);
    float3 lower = linearRGB * float3(12.9200000762939453125);
    return select(higher, lower, cutoff);
}

static inline __attribute__((always_inline))
float3 tonemap_aces_film_simple(thread const float3& x)
{
    float a = 2.5099999904632568359375;
    float b = 0.02999999932944774627685546875;
    float c = 2.4300000667572021484375;
    float d = 0.589999973773956298828125;
    float e = 0.14000000059604644775390625;
    return fast::clamp((x * ((x * a) + float3(b))) / ((x * ((x * c) + float3(d))) + float3(e)), float3(0.0), float3(1.0));
}

static inline __attribute__((always_inline))
float3 tonemap_Hejl2015(thread const float3& hdr)
{
    float4 vh = float4(hdr, 1.0);
    float4 va = (vh * 1.434999942779541015625) + float4(0.0500000007450580596923828125);
    float4 vf = (((vh * va) + float4(0.0040000001899898052215576171875)) / ((vh * (va + float4(0.550000011920928955078125))) + float4(0.0491000004112720489501953125))) - float4(0.082099996507167816162109375);
    return vf.xyz / vf.www;
}

static inline __attribute__((always_inline))
float3 hable_function(thread const float3& x)
{
    return (((x * ((x * 4.0) + float3(0.60000002384185791015625))) + float3(0.12999999523162841796875)) / ((x * ((x * 4.0) + float3(5.0))) + float3(3.900000095367431640625))) - float3(0.0333333350718021392822265625);
}

static inline __attribute__((always_inline))
float3 tonemap_hable(thread const float3& color)
{
    float3 param = color;
    float3 numerator = hable_function(param);
    float3 param_1 = float3(6.0);
    float3 denominator = hable_function(param_1);
    return numerator / denominator;
}

static inline __attribute__((always_inline))
float3 tonemap_filmic_alu(thread const float3& color_in)
{
    float3 color = fast::max(color_in - float3(0.0040000001899898052215576171875), float3(0.0));
    color = (color * ((color * 6.19999980926513671875) + float3(0.5))) / ((color * ((color * 6.19999980926513671875) + float3(1.7000000476837158203125))) + float3(0.0599999986588954925537109375));
    return color;
}

static inline __attribute__((always_inline))
float3 visualize_value(thread const float& val)
{
    float g = 1.0 - ((0.20000000298023223876953125 * (val - 3.2360498905181884765625)) * (val - 3.2360498905181884765625));
    float b = val;
    float r = 1.0 - (1.0 / ((0.5 * val) - 0.5));
    if (val > 1.0)
    {
        b = 0.0;
    }
    if (val < 3.0)
    {
        r = 0.0;
    }
    return fast::clamp(float3(r, g, b), float3(0.0), float3(1.0));
}

static inline __attribute__((always_inline))
float rgb_to_luminosity(thread const float3& color)
{
    return dot(color, float3(0.2989999949932098388671875, 0.58700001239776611328125, 0.114000000059604644775390625));
}

static inline __attribute__((always_inline))
float3 tonemap_basic(thread const float3& color, thread const int& tonemapper_type)
{
    switch (tonemapper_type)
    {
        case 1:
        {
            float3 param = color;
            float3 _358 = tonemap_aces_fitted(param);
            float3 param_1 = _358;
            return linear_to_srgb(param_1);
        }
        case 2:
        {
            float3 param_2 = color * 0.60000002384185791015625;
            float3 param_3 = tonemap_aces_film_simple(param_2);
            return linear_to_srgb(param_3);
        }
        case 3:
        {
            float3 param_4 = color;
            return tonemap_Hejl2015(param_4);
        }
        case 4:
        {
            float3 param_5 = color;
            return tonemap_hable(param_5);
        }
        case 5:
        {
            float3 param_6 = color;
            return tonemap_filmic_alu(param_6);
        }
        case 6:
        {
            return color / (color + float3(1.0));
        }
        case 7:
        {
            float max_val = fast::max(color.x, fast::max(color.y, color.z));
            float param_7 = max_val;
            return visualize_value(param_7);
        }
        case 8:
        {
            float3 param_8 = color;
            float l = rgb_to_luminosity(param_8);
            float param_9 = l;
            return visualize_value(param_9);
        }
        default:
        {
            return color;
        }
    }
}

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    constexpr sampler smp(mip_filter::linear, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    float4 rgb = spvDescriptorSet0.in_color.sample(smp, in.inUV) + spvDescriptorSet0.in_blur.sample(smp, in.inUV);
    float3 color = rgb.xyz;
    if (all(color == float3(0.0)))
    {
        out.out_sdr = float4(color, 1.0);
        return out;
    }
    float3 param = color;
    int param_1 = (*spvDescriptorSet0.config).tonemapper_type;
    float3 color_srgb_linear = tonemap_basic(param, param_1);
    switch ((*spvDescriptorSet0.config).output_color_space)
    {
        case 0:
        {
            out.out_sdr = float4(color_srgb_linear, 1.0);
            break;
        }
        case 1:
        {
            float3 color_linear_p3 = float3x3(float3(0.822488605976104736328125, 0.033200047910213470458984375, 0.0170890651643276214599609375), float3(0.1775114238262176513671875, 0.9667999744415283203125, 0.072411514818668365478515625), float3(5.5511151231257827021181583404541e-17, -1.7347234759768070944119244813919e-17, 0.910499393939971923828125)) * color_srgb_linear;
            out.out_sdr = float4(color_linear_p3, 1.0);
            break;
        }
    }
    return out;
}

    ,      #     �                GLSL.std.450                     main    �  �          G  �  "       G  �  !       G  �  "       G  �  !      G  �         G  �  "       G  �  !      G  �         H  �      #       H  �     #      H  �     #      G  �     G  �  "       G  �  !           !                              +          +     3   ��>+     4   �E?+     5   �x�=,     6   3   4   5   +     =   Y�<+     F   ��{?+     I   ���>+     M   ��s>  U         +     V   r�?+     W   㥛=+     X   ��<,     Y   V   W   X   +     Z   ���>+     [   ��h?+     \   �
	>,     ]   Z   [   \   +     ^   ҌE=+     _   fI�<+     `   xV?,     a   ^   _   `   ,  U   b   Y   ]   a   +     h   sh�?+     i   Rѽ+     j   MV�,     k   h   i   j   +     l   ���+     m   4׍?+     n   2��,     o   l   m   n   +     p   L���+     q   ?ƻ+     r   ��?,     s   p   q   r   ,  U   t   k   o   s   +     x       +     y     �?+     �   ף @+     �   ���<+     �   �@+     �   =
?+     �   )\>+     �   o�;+     �   ff�@+     �      ?+     �   ���?+     �   ��u=  �     �   �      +     �   .M;,     �   �   �   �   +     �   =
�?,     �   �   �   �   +     �   UU�>,     �   �   �   �   +     �   R�NA,     �   �   �   �     �         +     �   ��?+     �   ��L=+     �   I=+     �   $�=+         �@+       ��?+       �>+         �@+       ��y@+       ��=+     3  qO@+     G    @@,       y   y   y     �          +  �  �       	 �                              �      �  ;  �  �        �     �      �  ;  �  �        �  �    �           �     �  ;  �  �     ;  �  �      ,     �  x   x   x      �     �   ;  �  �       �  +   +         �     �  ;  �  �     +  +   �         �     +   +  +   �     +     �  ��R?+     �  ��=+     �  ^��<,     �  �  �  �  +     �  ��5>+     �  4�w?+     �  }L�=,     �  �  �  �  +     �    �$+     �    ��+     �  }i?,     �  �  �  �  ,  U   �  �  �  �  ,     �  =   =   =   ,     �  I   I   I   ,     �  M   M   M   ,     �  �   �   �   ,     �  �   �   �   ,     �  �   �   �   ,  �   �  �   �   �   �   ,  �   �  �   �   �   �   ,  �   �  �   �   �   �   ,  �   �  �   �   �   �   ,     �        ,     �        ,     �        ,     �        ,     �        ,     �  �   �   �   ,     �  �   �   �   ,     �  �   �   �   ,     �  �   �   �   +     �  4��?,     �  �  �  �  ,  �   �          +     �  ��L�+     �     �+     �  �Ga�,     �  �  �  �  +     �  �޽�,     �  �  �  �  6               �     �  >      �  �  ?  �  ?  =  �  �  �  =  �  �  �  V  �  �  �  �  =  �  �  �  W  �   �  �  �  =  �  �  �  V  �  �  �  �  W  �   �  �  �  �  �   �  �  �  O     �  �  �            �  �   �  �  �  �  �   �  �  �  �      �  �  �  �  �  �  Q     �  �      Q     �  �     Q     �  �     P  �   �  �  �  �  y   >  �  �  �  >  �  �  A  �  �  �  �  =  +   �  �  �  �      �  �  ]     _     c     h     k     n     q     v     �  �  �  �     �  �  6   �     �  �  3  �     �  �  �       �     2   �  �  y        �     2   �   �  �  �     �  y   �  �     �  y   �  �  �   �  �  y   �     �  �  x   �  �  �   �  �  G  �     �  �  x   �  P     �  �  �  �       �     +   �  �    �  �  �  v  Q     x  �      Q     z  �     Q     |  �          }     (   z  |       ~     (   x  }  �     e  ~  3  �     �  e  �       j     2   �  e  y        n     2   �   ~  �  �     o  y   n  �     p  y   o  �  �   r  ~  y   �     �  r  x   ~  �  �   v  ~  G  �     �  v  x   p  P     |  �  j  �            +   |  �    �  �  �  q  �     t  �    �     u  �  t  �  �  �  n  �     L  �  �       N     (   L  �  �     Q  N  �   �     S  Q  �  �     T  N  S  �     Y  Q  �       \     2   N  Y  �  �     ]  T  \  �  �  �  k  �     !  �    �     #  !  �       &     2   �  #  �  �     +  !  �       .     2   �  +  �  �     /  &  .  �     1  /  �  �       1  �  �  �  �  h  Q     �  �      Q     �  �     Q     �  �     P  �   �  �  �  �  y   �  �   �  �  �   �  �   �  �  �    �        2   �  �  �  �  �     �  �    �        2   �    �  �  �   	      �  �     	  �  O                     O                    �           �  �  �  c  �     e  �    �     �  e  �   �     �  �  �  �     �  e  �  �     �  e  �   �     �  �  �       �     2   e  �  �  �     �  �  �       �     +   �  �    �  �   �  �  �        �        �  �        �     2   �   �  �  �     �  �  �   �     �  �  �  �  �  �  �  _  �     �  b   �  �     �  �  �       �     2   �  �  �  �     �  �  F   �     �  �  �       �     2   �  �  �  �     �  �  �  �     �  t   �       �     +   �  �    �  �   �  �  �        �        �  �        �     2   �   �  �  �     �  �  �   �     �  �  �  �  �  �  �  ]  �  �  �  �  �     �  �  ]  �  _  �  c    h    k  ]  n  u  q    v  �  �  A  �  �  �  �  =  +   �  �  �  �      �  �  �      �     �  �  �  �     �  �  �  Q     �  �      Q     �  �     Q     �  �     P  �   �  �  �  �  y   >  �  �  �  �  �  �  Q     �  �      Q     �  �     Q     �  �     P  �   �  �  �  �  y   >  �  �  �  �  �  �  �  >  �  >  �  8                                               in_color               in_color                                    in_blur              in_blur                                    smp              smp         �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                      main                                          in_color               in_color                                      in_blur              in_blur                                      smp              smp                                                 �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                                                            in_color               in_color                                    in_blur              in_blur                                    smp              smp         �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                      main                                          in_color               in_color                                      in_blur              in_blur                                      smp              smp                                                 �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                                                            in_color               in_color                                    in_blur              in_blur                                    smp              smp         �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                      main                                          in_color               in_color                                      in_blur              in_blur                                      smp              smp                                                 �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                        bloom_combine_basic.frag