                       �""D@"���Ȟ�0�d�8�      �=      struct HistogramResult
{
    float average_luminosity_interpolated;
    float average_luminosity_this_frame;
    float average_luminosity_last_frame;
    float min_luminosity_interpolated;
    float min_luminosity_this_frame;
    float min_luminosity_last_frame;
    float max_luminosity_interpolated;
    float max_luminosity_this_frame;
    float max_luminosity_last_frame;
    float low_luminosity_interpolated;
    float low_luminosity_this_frame;
    float low_luminosity_last_frame;
    float high_luminosity_interpolated;
    float high_luminosity_this_frame;
    float high_luminosity_last_frame;
    float average_bin_include_zero;
    float average_bin_non_zero;
    uint min_bin;
    uint max_bin;
    uint low_bin;
    uint high_bin;
};

cbuffer Config : register(b3, space0)
{
    int config_tonemapper_type : packoffset(c0);
    int config_output_color_space : packoffset(c0.y);
    float config_max_color_component_value : packoffset(c0.z);
};

RWByteAddressBuffer histogram_result : register(u4, space0);
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

float3 convertRGB2XYZ(float3 _rgb)
{
    float3 xyz;
    xyz.x = dot(float3(0.41245639324188232421875f, 0.3575761020183563232421875f, 0.180437505245208740234375f), _rgb);
    xyz.y = dot(float3(0.21267290413379669189453125f, 0.715152204036712646484375f, 0.072175003588199615478515625f), _rgb);
    xyz.z = dot(float3(0.01933390088379383087158203125f, 0.119191996753215789794921875f, 0.950304090976715087890625f), _rgb);
    return xyz;
}

float3 convertXYZ2Yxy(float3 _xyz)
{
    float inv = 1.0f / dot(_xyz, 1.0f.xxx);
    return float3(_xyz.y, _xyz.x * inv, _xyz.y * inv);
}

float3 convertRGB2Yxy(float3 _rgb)
{
    float3 param = _rgb;
    float3 param_1 = convertRGB2XYZ(param);
    return convertXYZ2Yxy(param_1);
}

float reinhard2(float x, float whitepoint)
{
    return (x * (1.0f + (x / (whitepoint * whitepoint)))) / (1.0f + x);
}

float3 convertYxy2XYZ(float3 _Yxy)
{
    float3 xyz;
    xyz.x = (_Yxy.x * _Yxy.y) / _Yxy.z;
    xyz.y = _Yxy.x;
    xyz.z = (_Yxy.x * ((1.0f - _Yxy.y) - _Yxy.z)) / _Yxy.z;
    return xyz;
}

float3 convertXYZ2RGB(float3 _xyz)
{
    float3 rgb;
    rgb.x = dot(float3(3.240454196929931640625f, -1.537138462066650390625f, -0.498531401157379150390625f), _xyz);
    rgb.y = dot(float3(-0.969265997409820556640625f, 1.87601077556610107421875f, 0.04155600070953369140625f), _xyz);
    rgb.z = dot(float3(0.0556433983147144317626953125f, -0.2040258944034576416015625f, 1.05722522735595703125f), _xyz);
    return rgb;
}

float3 convertYxy2RGB(float3 _Yxy)
{
    float3 param = _Yxy;
    float3 param_1 = convertYxy2XYZ(param);
    return convertXYZ2RGB(param_1);
}

float3 old_autoexposure_tonemapping(float3 in_color_1, float histogram_result_average_luminosity_interpolated)
{
    if (dot(in_color_1, 1.0f.xxx) < 9.9999997473787516355514526367188e-05f)
    {
        return 0.0f.xxx;
    }
    float average_luma = clamp(histogram_result_average_luminosity_interpolated, 0.0005000000237487256526947021484375f, 0.699999988079071044921875f);
    float3 param = in_color_1;
    float3 Yxy = convertRGB2Yxy(param);
    float gray = 0.02999999932944774627685546875f;
    float white_squared = 1.0f;
    float lp = (Yxy.x * gray) / (average_luma + 9.9999997473787516355514526367188e-05f);
    float param_1 = lp;
    float param_2 = white_squared;
    Yxy.x = reinhard2(param_1, param_2);
    float3 param_3 = Yxy;
    return convertYxy2RGB(param_3);
}

float modified_reinhard(float x, float m, float k)
{
    float kx = k * x;
    return (kx * (1.0f + (x / ((k * m) * m)))) / (1.0f + kx);
}

float3 Oklab_lms_to_Oklab(inout float3 lms)
{
    lms = pow(max(lms, 0.0f.xxx), 0.3333333432674407958984375f.xxx);
    return mul(lms, float3x3(float3(0.2104542553424835205078125f, 1.9779984951019287109375f, 0.025990404188632965087890625f), float3(0.793617784976959228515625f, -2.428592205047607421875f, 0.782771766185760498046875f), float3(-0.004072046838700771331787109375f, 0.4505937099456787109375f, -0.8086757659912109375f)));
}

float3 linear_srgb_to_oklab(float3 rgb)
{
    float3 param = mul(rgb, float3x3(float3(0.4122420847415924072265625f, 0.21194292604923248291015625f, 0.08835887908935546875f), float3(0.53626155853271484375f, 0.680702149868011474609375f, 0.2818474471569061279296875f), float3(0.05142803490161895751953125f, 0.10737408697605133056640625f, 0.630129635334014892578125f)));
    float3 _634 = Oklab_lms_to_Oklab(param);
    return _634;
}

float3 Oklab_to_Oklab_lms(float3 oklab)
{
    float3 lms = mul(oklab, float3x3(float3(0.999981462955474853515625f, 1.00000560283660888671875f, 1.00011169910430908203125f), float3(0.3963304460048675537109375f, -0.105559170246124267578125f, -0.089439980685710906982421875f), float3(0.215799748897552490234375f, -0.063852988183498382568359375f, -1.291461467742919921875f)));
    return pow(lms, 3.0f.xxx);
}

float3 oklab_to_linear_srgb(float3 oklab)
{
    float3 param = oklab;
    return mul(Oklab_to_Oklab_lms(param), float3x3(float3(4.076537609100341796875f, -1.268605709075927734375f, -0.004197560250759124755859375f), float3(-3.307096004486083984375f, 2.609747409820556640625f, -0.70356845855712890625f), float3(0.23082244396209716796875f, -0.34116363525390625f, 1.70720565319061279296875f)));
}

float3 tonemap_bergstrom(float3 in_color_1, float max_component_value, float histogram_result_low_luminosity_interpolated, float histogram_result_high_luminosity_interpolated, float histogram_result_max_luminosity_interpolated)
{
    float l_low = histogram_result_low_luminosity_interpolated;
    float l_high = max(histogram_result_high_luminosity_interpolated, l_low + 0.00999999977648258209228515625f);
    float l_max_scale = 5.0f;
    float l_max = max(histogram_result_max_luminosity_interpolated, l_high * l_max_scale);
    float k_max = max_component_value;
    float k_desaturation = lerp(0.21400000154972076416015625f, k_max, 0.5f);
    float v = 0.203999996185302734375f / (l_high - l_low);
    float luminance = dot(in_color_1, float3(0.2125999927520751953125f, 0.715200006961822509765625f, 0.072200000286102294921875f));
    float adjusted_luminance = 0.0f;
    if (luminance < l_low)
    {
        float param = l_low - luminance;
        float param_1 = l_low;
        float param_2 = v / 0.00999999977648258209228515625f;
        adjusted_luminance = 0.00999999977648258209228515625f - (0.00999999977648258209228515625f * modified_reinhard(param, param_1, param_2));
    }
    else
    {
        if (luminance < l_high)
        {
            adjusted_luminance = 0.00999999977648258209228515625f + ((luminance - l_low) * v);
        }
        else
        {
            float param_3 = luminance - l_high;
            float param_4 = l_max;
            float param_5 = v / (k_max - 0.21400000154972076416015625f);
            adjusted_luminance = 0.21400000154972076416015625f + ((k_max - 0.21400000154972076416015625f) * modified_reinhard(param_3, param_4, param_5));
        }
    }
    if (adjusted_luminance < 9.9999997473787516355514526367188e-05f)
    {
        return 0.0f.xxx;
    }
    else
    {
        float3 out_color = in_color_1 * (adjusted_luminance / (luminance + 9.9999999747524270787835121154785e-07f));
        float max_element = max(out_color.x, max(out_color.y, out_color.z));
        if (max_element > 1.0f)
        {
            out_color /= max_element.xxx;
        }
        float3 param_6 = out_color;
        float3 oklab = linear_srgb_to_oklab(param_6);
        float3 _763 = oklab;
        float2 _765 = _763.yz * (1.0f - clamp((adjusted_luminance - k_desaturation) / (k_max - k_desaturation), 0.0f, 1.0f));
        oklab.y = _765.x;
        oklab.z = _765.y;
        float3 param_7 = oklab;
        out_color = oklab_to_linear_srgb(param_7);
        if (max_element > 1.0f)
        {
            out_color *= max_element;
        }
        return out_color;
    }
}

float3 tonemap_adv(float3 color, int tonemapper_type, float max_component_value, float histogram_result_low_luminosity_interpolated, float histogram_result_average_luminosity_interpolated, float histogram_result_high_luminosity_interpolated, float histogram_result_max_luminosity_interpolated)
{
    switch (tonemapper_type)
    {
        case 1:
        {
            float3 param = color;
            float3 _833 = tonemap_aces_fitted(param);
            float3 param_1 = _833;
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
        case 9:
        {
            float3 param_10 = color;
            float param_11 = histogram_result_average_luminosity_interpolated;
            return old_autoexposure_tonemapping(param_10, param_11);
        }
        case 10:
        {
            float3 param_12 = color;
            float param_13 = max_component_value;
            float param_14 = histogram_result_low_luminosity_interpolated;
            float param_15 = histogram_result_high_luminosity_interpolated;
            float param_16 = histogram_result_max_luminosity_interpolated;
            return tonemap_bergstrom(param_12, param_13, param_14, param_15, param_16);
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
    float param_2 = config_max_color_component_value;
    float param_3 = asfloat(histogram_result.Load(36));
    float param_4 = asfloat(histogram_result.Load(0));
    float param_5 = asfloat(histogram_result.Load(48));
    float param_6 = asfloat(histogram_result.Load(24));
    float3 color_srgb_linear = tonemap_adv(param, param_1, param_2, param_3, param_4, param_5, param_6);
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
    �D      #pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct Config
{
    int tonemapper_type;
    int output_color_space;
    float max_color_component_value;
};

struct HistogramResult
{
    float average_luminosity_interpolated;
    float average_luminosity_this_frame;
    float average_luminosity_last_frame;
    float min_luminosity_interpolated;
    float min_luminosity_this_frame;
    float min_luminosity_last_frame;
    float max_luminosity_interpolated;
    float max_luminosity_this_frame;
    float max_luminosity_last_frame;
    float low_luminosity_interpolated;
    float low_luminosity_this_frame;
    float low_luminosity_last_frame;
    float high_luminosity_interpolated;
    float high_luminosity_this_frame;
    float high_luminosity_last_frame;
    float average_bin_include_zero;
    float average_bin_non_zero;
    uint min_bin;
    uint max_bin;
    uint low_bin;
    uint high_bin;
};

struct HistogramResultBuffer
{
    HistogramResult result;
};

struct spvDescriptorSetBuffer0
{
    texture2d<float> in_color [[id(0)]];
    texture2d<float> in_blur [[id(1)]];
    constant Config* config [[id(3)]];
    device HistogramResultBuffer* histogram_result [[id(4)]];
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
float3 convertRGB2XYZ(thread const float3& _rgb)
{
    float3 xyz;
    xyz.x = dot(float3(0.41245639324188232421875, 0.3575761020183563232421875, 0.180437505245208740234375), _rgb);
    xyz.y = dot(float3(0.21267290413379669189453125, 0.715152204036712646484375, 0.072175003588199615478515625), _rgb);
    xyz.z = dot(float3(0.01933390088379383087158203125, 0.119191996753215789794921875, 0.950304090976715087890625), _rgb);
    return xyz;
}

static inline __attribute__((always_inline))
float3 convertXYZ2Yxy(thread const float3& _xyz)
{
    float inv = 1.0 / dot(_xyz, float3(1.0));
    return float3(_xyz.y, _xyz.x * inv, _xyz.y * inv);
}

static inline __attribute__((always_inline))
float3 convertRGB2Yxy(thread const float3& _rgb)
{
    float3 param = _rgb;
    float3 param_1 = convertRGB2XYZ(param);
    return convertXYZ2Yxy(param_1);
}

static inline __attribute__((always_inline))
float reinhard2(thread const float& x, thread const float& whitepoint)
{
    return (x * (1.0 + (x / (whitepoint * whitepoint)))) / (1.0 + x);
}

static inline __attribute__((always_inline))
float3 convertYxy2XYZ(thread const float3& _Yxy)
{
    float3 xyz;
    xyz.x = (_Yxy.x * _Yxy.y) / _Yxy.z;
    xyz.y = _Yxy.x;
    xyz.z = (_Yxy.x * ((1.0 - _Yxy.y) - _Yxy.z)) / _Yxy.z;
    return xyz;
}

static inline __attribute__((always_inline))
float3 convertXYZ2RGB(thread const float3& _xyz)
{
    float3 rgb;
    rgb.x = dot(float3(3.240454196929931640625, -1.537138462066650390625, -0.498531401157379150390625), _xyz);
    rgb.y = dot(float3(-0.969265997409820556640625, 1.87601077556610107421875, 0.04155600070953369140625), _xyz);
    rgb.z = dot(float3(0.0556433983147144317626953125, -0.2040258944034576416015625, 1.05722522735595703125), _xyz);
    return rgb;
}

static inline __attribute__((always_inline))
float3 convertYxy2RGB(thread const float3& _Yxy)
{
    float3 param = _Yxy;
    float3 param_1 = convertYxy2XYZ(param);
    return convertXYZ2RGB(param_1);
}

static inline __attribute__((always_inline))
float3 old_autoexposure_tonemapping(thread const float3& in_color, thread const float& histogram_result_average_luminosity_interpolated)
{
    if (dot(in_color, float3(1.0)) < 9.9999997473787516355514526367188e-05)
    {
        return float3(0.0);
    }
    float average_luma = fast::clamp(histogram_result_average_luminosity_interpolated, 0.0005000000237487256526947021484375, 0.699999988079071044921875);
    float3 param = in_color;
    float3 Yxy = convertRGB2Yxy(param);
    float gray = 0.02999999932944774627685546875;
    float white_squared = 1.0;
    float lp = (Yxy.x * gray) / (average_luma + 9.9999997473787516355514526367188e-05);
    float param_1 = lp;
    float param_2 = white_squared;
    Yxy.x = reinhard2(param_1, param_2);
    float3 param_3 = Yxy;
    return convertYxy2RGB(param_3);
}

static inline __attribute__((always_inline))
float modified_reinhard(thread const float& x, thread const float& m, thread const float& k)
{
    float kx = k * x;
    return (kx * (1.0 + (x / ((k * m) * m)))) / (1.0 + kx);
}

static inline __attribute__((always_inline))
float3 Oklab_lms_to_Oklab(thread float3& lms)
{
    lms = pow(fast::max(lms, float3(0.0)), float3(0.3333333432674407958984375));
    return float3x3(float3(0.2104542553424835205078125, 1.9779984951019287109375, 0.025990404188632965087890625), float3(0.793617784976959228515625, -2.428592205047607421875, 0.782771766185760498046875), float3(-0.004072046838700771331787109375, 0.4505937099456787109375, -0.8086757659912109375)) * lms;
}

static inline __attribute__((always_inline))
float3 linear_srgb_to_oklab(thread const float3& rgb)
{
    float3 param = float3x3(float3(0.4122420847415924072265625, 0.21194292604923248291015625, 0.08835887908935546875), float3(0.53626155853271484375, 0.680702149868011474609375, 0.2818474471569061279296875), float3(0.05142803490161895751953125, 0.10737408697605133056640625, 0.630129635334014892578125)) * rgb;
    float3 _634 = Oklab_lms_to_Oklab(param);
    return _634;
}

static inline __attribute__((always_inline))
float3 Oklab_to_Oklab_lms(thread const float3& oklab)
{
    float3 lms = float3x3(float3(0.999981462955474853515625, 1.00000560283660888671875, 1.00011169910430908203125), float3(0.3963304460048675537109375, -0.105559170246124267578125, -0.089439980685710906982421875), float3(0.215799748897552490234375, -0.063852988183498382568359375, -1.291461467742919921875)) * oklab;
    return pow(lms, float3(3.0));
}

static inline __attribute__((always_inline))
float3 oklab_to_linear_srgb(thread const float3& oklab)
{
    float3 param = oklab;
    return float3x3(float3(4.076537609100341796875, -1.268605709075927734375, -0.004197560250759124755859375), float3(-3.307096004486083984375, 2.609747409820556640625, -0.70356845855712890625), float3(0.23082244396209716796875, -0.34116363525390625, 1.70720565319061279296875)) * Oklab_to_Oklab_lms(param);
}

static inline __attribute__((always_inline))
float3 tonemap_bergstrom(thread const float3& in_color, thread const float& max_component_value, thread const float& histogram_result_low_luminosity_interpolated, thread const float& histogram_result_high_luminosity_interpolated, thread const float& histogram_result_max_luminosity_interpolated)
{
    float l_low = histogram_result_low_luminosity_interpolated;
    float l_high = fast::max(histogram_result_high_luminosity_interpolated, l_low + 0.00999999977648258209228515625);
    float l_max_scale = 5.0;
    float l_max = fast::max(histogram_result_max_luminosity_interpolated, l_high * l_max_scale);
    float k_max = max_component_value;
    float k_desaturation = mix(0.21400000154972076416015625, k_max, 0.5);
    float v = 0.203999996185302734375 / (l_high - l_low);
    float luminance = dot(in_color, float3(0.2125999927520751953125, 0.715200006961822509765625, 0.072200000286102294921875));
    float adjusted_luminance = 0.0;
    if (luminance < l_low)
    {
        float param = l_low - luminance;
        float param_1 = l_low;
        float param_2 = v / 0.00999999977648258209228515625;
        adjusted_luminance = 0.00999999977648258209228515625 - (0.00999999977648258209228515625 * modified_reinhard(param, param_1, param_2));
    }
    else
    {
        if (luminance < l_high)
        {
            adjusted_luminance = 0.00999999977648258209228515625 + ((luminance - l_low) * v);
        }
        else
        {
            float param_3 = luminance - l_high;
            float param_4 = l_max;
            float param_5 = v / (k_max - 0.21400000154972076416015625);
            adjusted_luminance = 0.21400000154972076416015625 + ((k_max - 0.21400000154972076416015625) * modified_reinhard(param_3, param_4, param_5));
        }
    }
    if (adjusted_luminance < 9.9999997473787516355514526367188e-05)
    {
        return float3(0.0);
    }
    else
    {
        float3 out_color = in_color * (adjusted_luminance / (luminance + 9.9999999747524270787835121154785e-07));
        float max_element = fast::max(out_color.x, fast::max(out_color.y, out_color.z));
        if (max_element > 1.0)
        {
            out_color /= float3(max_element);
        }
        float3 param_6 = out_color;
        float3 oklab = linear_srgb_to_oklab(param_6);
        float3 _763 = oklab;
        float2 _765 = _763.yz * (1.0 - fast::clamp((adjusted_luminance - k_desaturation) / (k_max - k_desaturation), 0.0, 1.0));
        oklab.y = _765.x;
        oklab.z = _765.y;
        float3 param_7 = oklab;
        out_color = oklab_to_linear_srgb(param_7);
        if (max_element > 1.0)
        {
            out_color *= max_element;
        }
        return out_color;
    }
}

static inline __attribute__((always_inline))
float3 tonemap_adv(thread const float3& color, thread const int& tonemapper_type, thread const float& max_component_value, thread const float& histogram_result_low_luminosity_interpolated, thread const float& histogram_result_average_luminosity_interpolated, thread const float& histogram_result_high_luminosity_interpolated, thread const float& histogram_result_max_luminosity_interpolated)
{
    switch (tonemapper_type)
    {
        case 1:
        {
            float3 param = color;
            float3 _833 = tonemap_aces_fitted(param);
            float3 param_1 = _833;
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
        case 9:
        {
            float3 param_10 = color;
            float param_11 = histogram_result_average_luminosity_interpolated;
            return old_autoexposure_tonemapping(param_10, param_11);
        }
        case 10:
        {
            float3 param_12 = color;
            float param_13 = max_component_value;
            float param_14 = histogram_result_low_luminosity_interpolated;
            float param_15 = histogram_result_high_luminosity_interpolated;
            float param_16 = histogram_result_max_luminosity_interpolated;
            return tonemap_bergstrom(param_12, param_13, param_14, param_15, param_16);
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
    float param_2 = (*spvDescriptorSet0.config).max_color_component_value;
    float param_3 = (*spvDescriptorSet0.histogram_result).result.low_luminosity_interpolated;
    float param_4 = (*spvDescriptorSet0.histogram_result).result.average_luminosity_interpolated;
    float param_5 = (*spvDescriptorSet0.histogram_result).result.high_luminosity_interpolated;
    float param_6 = (*spvDescriptorSet0.histogram_result).result.max_luminosity_interpolated;
    float3 color_srgb_linear = tonemap_adv(param, param_1, param_2, param_3, param_4, param_5, param_6);
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

    -      #     `                GLSL.std.450                     main    �  �          G  �  "       G  �  !       G  �  "       G  �  !      G  �         G  �  "       G  �  !      G  �         H  �      #       H  �     #      H  �     #      G  �     G  �  "       G  �  !      H  �      #       H  �     #      H  �     #      H  �     #      H  �     #      H  �     #      H  �     #      H  �     #      H  �     #       H  �  	   #   $   H  �  
   #   (   H  �     #   ,   H  �     #   0   H  �     #   4   H  �     #   8   H  �     #   <   H  �     #   @   H  �     #   D   H  �     #   H   H  �     #   L   H  �     #   P   H  �      #       G  �     G  �  "       G  �  !           !                              a          +     n   |-�>+     o   7�>+     p   ��8>,     q   n   o   p     t           +  t   u       +     w   ��Y>+     x   77?+     y   }Г=,     z   w   x   y   +        !b�<+     �   ��=+     �   !Gs?,     �      �   �   +     �   �cO@+     �   ��Ŀ+     �   �?��,     �   �   �   �   +     �   �!x�+     �   !�?+     �   �6*=,     �   �   �   �   +     �   U�c=+     �   *�P�+     �   (S�?,     �   �   �   �   +     �     �?,     �   �   �   �   +     �   ��>+     �   �E?+     �   �x�=,     �   �   �   �   +     �   Y�<+     �   ��{?+     �   ���>+     �   ��s>          +       r�?+       㥛=+       ��<,             +     	  ���>+     
  ��h?+       �
	>,       	  
    +       ҌE=+       fI�<+       xV?,             ,            +       sh�?+       Rѽ+       MV�,             +       ���+       4׍?+       2��,             +       L���+        ?ƻ+     !  ��?,     "       !  ,    #      "  +     '      +     /  ף @+     1  ���<+     3  �@+     5  =
?+     7  )\>+     S  o�;+     Y  ff�@+     \     ?+     c  ���?+     g  ��u=  n    o  n     +     s  .M;,     t  s  s  s  +     w  =
�?,     x  w  w  w  +     z  UU�>,     {  z  z  z  +     �  R�NA,     �  �  �  �    �        +     �  ��?+     �  ��L=+     �  I=+     �  $�=+     �    �@+     �  ��?+     �  �>+     �    �@+     �  ��y@+     �  ��=+     �  ��8,     �  '  '  '  +     �  o:+     �  333?+       ���>,             +       R�W>+       /�?+       ���<,             +       �*K?+       n�+       �cH?,             +       �n��+        8��>+     !  `O�,     "       !  ,    #      "  +     )  ��?+     *  / �?+     +  ��?,     ,  )  *  +  +     -  ���>+     .  h/ؽ+     /  O,��,     0  -  .  /  +     1  ��\>+     2  [ł�+     3  �N��,     4  1  2  3  ,    5  ,  0  4  +     9    @@,     :  9  9  9  +     Q  �r�@+     R  �a��+     S  ����,     T  Q  R  S  +     U  v�S�+     V  '@+     W  4�,     X  U  V  W  +     Y  �\l>+     Z   ���+     [  ���?,     \  Y  Z  [  ,    ]  T  X  \  +     d  e�>+     e  �Y>+     f  ���=,     g  d  e  f  +     h  pH	?+     i  B.?+     j  ON�>,     k  h  i  j  +     l  4�R=+     m  ���=+     n  -P!?,     o  l  m  n  ,    p  g  k  o  +     |  
�#<+     �  �"[>+     �  `�P>+     �  гY>+     �  Y7?+     �  �ݓ=,     �  �  �  �  +     �  �7�5  �        +       qO@ 	 �                              �      �  ;  �  �        �     �      �  ;  �  �        �  �     �     �  ;  �  �     ;  �  �         �     �  ;  �  �       �  a   a         �     �  ;  �  �     +  a   �      +  a   �       �                                                     t   t   t   t     �  �     �     �  ;  �  �     +  a   �  	   +  a   �     +  a   �        �     a      �        +  a   �     +     �  ��R?+     �  ��=+     �  ^��<,     �  �  �  �  +     �  ��5>+     �  4�w?+     �  }L�=,     �  �  �  �  +     �    �$+     �    ��+     �  }i?,     �  �  �  �  ,    �  �  �  �  ,       �   �   �   ,       �   �   �   ,       �   �   �   ,        1  1  1  ,     !  5  5  5  ,     "  7  7  7  ,  �  #  �  �  �  �  ,  �  $  S  S  S  S  ,  �  &  �  �  �  �  ,  �  '  �  �  �  �  ,     (  �  �  �  ,     )  �  �  �  ,     *  �  �  �  ,     +  �  �  �  ,     ,  �  �  �  ,     ?  S  S  S  ,     @  \  \  \  ,     A  c  c  c  ,     B  g  g  g  +     D  33�A+     O  4��?,     P  O  O  O  ,  �  Q  �  �  �  �  +     U  
�#�+     V  ��L�+     W     �+     X  �Ga�,     Y  X  X  X  +     Z  �޽�,     [  Z  Z  Z  6               �     �        �  u     �    =  �  �  �  =  �  �  �  V  �  �  �  �  =  �  �  �  W  �  �  �  �  =  �  �  �  V  �  �  �  �  W  �  �  �  �  �  �  �  �  �  O     �  �  �            �  o  �  �  �  �  n  �  �  �  �      �  �  �  �  �  �  Q     �  �      Q     �  �     Q     �  �     P  �  �  �  �  �  �   >  �  �  �    �  �  A  �  �  �  �  =  a   �  �  A  �  �  �  �  =     �  �  A  �  �  �  �  �  =     �  �  A  �  �  �  �  �  =     �  �  A  �  �  �  �  �  =     �  �  A  �  �  �  �  �  =     �  �  �  i      �  �  5     7     ;     @     C     F     I     N     Y  	   ^  
   b  �  b  �  �      �  u   )  �  )  �     -  �  |       .     (   �  -  �     2  .  �       3     (   �  2       6     .   �  �  \  �     9  .  �  �     :  �  9  �     <  �  �  �  n  ?  <  �  �  d      �  ?  @  J  �  J  �  n  M  <  .  �  c      �  M  N  U  �  U  �     W  �  �  �     Z  <  .  �     ^  :  W  �     �  ^  Z  �     �  ^  3  �     �  �  3  �     �  Z  �  �     �  �   �  �     �  �  �       �     2   ^  Z  �   �     �  �  �       b     2   W  �  �  �  c  �  N  �     Q  <  �       T     2   Q  :  |  �  c  �  c  �     
  T  N  b  U  �  d  �  @  �     C  �  <  �     E  D  9  �     �  E  C  �     �  E  �  �     �  �  �  �     �  C  �  �     �  �   �  �     �  �  �       �     2   E  C  �   �     �  �  �       I     2   U  �  |  �  d  �  d  �     	  I  @  
  c  �  n  f  	  �  �  �      �  f  g  h  �  h  �     l  <  �  �     m  	  l  �     n  �  m  Q     p  n      Q     r  n     Q     t  n          u     (   r  t       v     (   p  u  �  n  x  v  �   �  ~      �  x  y  ~  �  y  P     |  v  v  v  �     }  n  |  �  ~  �  ~  �       n  h  }  y  �     �  p         �     (   �  �       �        �    �     �  #  �  �     �  	  6  �     �  �  6  �     �  �  �       �     +   �  '  �   �     �  �   �  O  �  �  �  �        �  �  �  �  �  Q     �  �      R       �  �     Q     �  �     R       �       �     �  5         �        �  :  �     �  ]  �  �  �      �  x  �  �  �  �  �     �  �  v  �  �  �  �  �       �  ~  �  �  �  �  �  g  �  �  �  �  �  �  �  �       �  g    �  �  i  �  ^  �  �      �  u   �  �  �  �     �  �  �   �  n  �  �  �  �  �      �  �  �  �  �  �  �  �  �  �       �     +   �  �  �  �     �  q   �  �     �  z   �  �     �  �   �  P     F  �  �  �  �     �  F  �   �     �  �   �  �     �  �  �  �     �  �  �  �     �  �  1  �     �  �  �  �     �  �  �  �     �  �   �  �     �  �  �  �     �  �  �  �     �  �  �       G  �       �     2   G  �  �        H  �       �     2   H  �  �  �     �  �  �  �       �  �  P     I  �  �    �       �   I  �       �   I  �       �   I  P     J        �  �  �  �  �       �  �  J  �  �  i  �  Y  �     f  �  �   �     m  f    �     K  m  V       r     2   K  m  �        v     2   \  f  W  �     w  �   v  �     x  �   w  �  n  z  f  �   �     \  z  '  f  �  n  ~  f  9  �     ]  ~  '  x  P     �  ]  r  \       �     +   �  �  �   �  i  �  N  Q     P  �      Q     R  �     Q     T  �          U     (   R  T       V     (   P  U  �     H  V    �     M  H  V       M     2   M  H  �        Q     2   \  V  W  �     R  �   Q  �     S  �   R  �  n  U  V  �   �     ^  U  '  V  �  n  Y  V  9  �     _  Y  '  S  P     _  _  M  ^       b     +   _  �  �   �  i  �  I  �     L  �  �   �     M  �  L  �  i  �  F  �     /  �  ?       1     (   /  �  �     4  1  Y  �     6  4  @  �     7  1  6  �     <  4  A       ?     2   1  <  B  �     @  7  ?  �  i  �  C  �       �  �  �         (       	     2   �    )  �         *            2   �    +  �       	    �         ,  �     �    P  �  i  �  @  Q     �  �      Q     �  �     Q     �  �     P  �  �  �  �  �  �   �  �  �  �  �  �  �  �  �  #    �  �     2   �  �  $  �  �  �  �  Q    �  �     2   �  �  &  �  �  �  �  �  �  �  �  �  '  O     �  �  �            O     �  �  �           �     �  �  �  �  i  �  ;  �     =  �  �  �     �  =  /  �     �  �     �     �  =  �  �     �  =  3  �     �  �  !       �     2   =  �  "  �     �  �  �       �     +   �  �  �   �  o  �  �  t       �        �  {       �     2   x  �  Y  �     �  �  �  �     �  �  �  �  �  i  �  7  �     r    �  �     �  r         �     2   r  �  [  �     �  r  �   �     �  �         �     2   r  �    �     �  �  �  �     v  #  �       z     +   v  �  �   �  o  �  z  t       �        z  {       �     2   x  �  Y  �     �  z  �  �     �  �  �  �  �  i  �  5  �  i  �  i  �       �  5  �  7  �  ;  �  @  �  C  @  F  M  I  b  N  �  Y    �    �  A  �  �  �  �  =  a   �  �  �  �      �  �  �      �     �  �  �  �     �  �    Q     �  �      Q     �  �     Q     �  �     P  �  �  �  �  �  �   >  �  �  �  �  �  �  Q     �        Q     �       Q     �       P  �  �  �  �  �  �   >  �  �  �  �  �  �  �    �    �  8                                               in_color               in_color                                    in_blur              in_blur                                    smp              smp         �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                               HistogramResultBuffer              histogram_result                            main                                          in_color               in_color                                      in_blur              in_blur                                      smp              smp                                                 �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                                   HistogramResultBuffer              histogram_result                                                                in_color               in_color                                    in_blur              in_blur                                    smp              smp         �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                               HistogramResultBuffer              histogram_result                            main                                          in_color               in_color                                      in_blur              in_blur                                      smp              smp                                                 �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                                   HistogramResultBuffer              histogram_result                                                                in_color               in_color                                    in_blur              in_blur                                    smp              smp         �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                               HistogramResultBuffer              histogram_result                            main                                          in_color               in_color                                      in_blur              in_blur                                      smp              smp                                                 �                           Config              Config               Config.tonemapper_type           Config.output_color_space           Config.max_color_component_value                                   HistogramResultBuffer              histogram_result                            bloom_combine_adv.frag