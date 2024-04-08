                       �""D@"���Ȟ��:�0;?1      �.      cbuffer Config : register(b6, space0)
{
    row_major float4x4 config_current_view_proj_inv : packoffset(c0);
    row_major float4x4 config_previous_view_proj : packoffset(c4);
    float2 config_jitter_amount : packoffset(c8);
    uint config_has_history_data : packoffset(c8.z);
    uint config_enable_side_by_side_debug_view : packoffset(c8.w);
    float config_history_weight : packoffset(c9);
    float config_history_weight_velocity_adjust_multiplier : packoffset(c9.y);
    float config_history_weight_velocity_adjust_max : packoffset(c9.z);
    uint config_viewport_width : packoffset(c9.w);
    uint config_viewport_height : packoffset(c10);
};

Texture2D<float4> history_tex : register(t0, space0);
SamplerState smp_bilinear : register(s5, space0);
Texture2D<float4> current_tex : register(t1, space0);
SamplerState smp_nearest : register(s4, space0);
Texture2D<float4> velocity_tex : register(t2, space0);
Texture2D<float4> depth_tex : register(t3, space0);

static float4 gl_FragCoord;
static float4 out_image;
static float2 inUV;

struct SPIRV_Cross_Input
{
    float2 inUV : TEXCOORD0;
    float4 gl_FragCoord : SV_Position;
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

float draw_line(float2 p1, float2 p2, float2 x, inout float width)
{
    float2 p1_to_x = x - p1;
    float2 _line = p2 - p1;
    float2 line_dir = normalize(_line);
    float2 p1_to_nearest_on_line = line_dir * dot(p1_to_x, line_dir);
    float2 nearest_on_line = p1 + p1_to_nearest_on_line;
    float dist_nearest_from_p2 = length(p2 - nearest_on_line);
    float max_dist_from_p = length(_line);
    if (dist_nearest_from_p2 > max_dist_from_p)
    {
        return 0.0f;
    }
    float dist_nearest_from_p1 = length(p1 - nearest_on_line);
    if (dist_nearest_from_p1 > max_dist_from_p)
    {
        return 0.0f;
    }
    width /= 2.0f;
    float width_min = width * 0.949999988079071044921875f;
    float width_max = width * 1.0499999523162841796875f;
    float dist_from_line = length(x - nearest_on_line);
    return clamp(1.0f - ((dist_from_line - width_min) / (width_max - width_min)), 0.0f, 1.0f);
}

float rgb_to_luminosity(float3 color)
{
    return dot(color, float3(0.2989999949932098388671875f, 0.58700001239776611328125f, 0.114000000059604644775390625f));
}

float3 sample_history_catmull_rom(float2 uv, float2 texel_size)
{
    float2 samplePos = uv / texel_size;
    float2 texPos1 = floor(samplePos - 0.5f.xx) + 0.5f.xx;
    float2 f = samplePos - texPos1;
    float2 w0 = f * ((-0.5f).xx + (f * (1.0f.xx - (f * 0.5f))));
    float2 w1 = 1.0f.xx + ((f * f) * ((-2.5f).xx + (f * 1.5f)));
    float2 w2 = f * (0.5f.xx + (f * (2.0f.xx - (f * 1.5f))));
    float2 w3 = (f * f) * ((-0.5f).xx + (f * 0.5f));
    float2 w12 = w1 + w2;
    float2 offset12 = w2 / (w1 + w2);
    float2 texPos0 = texPos1 - 1.0f.xx;
    float2 texPos3 = texPos1 + 2.0f.xx;
    float2 texPos12 = texPos1 + offset12;
    texPos0 *= texel_size;
    texPos3 *= texel_size;
    texPos12 *= texel_size;
    float3 result = 0.0f.xxx;
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos0.x, texPos0.y), 0.0f).xyz * w0.x) * w0.y);
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos12.x, texPos0.y), 0.0f).xyz * w12.x) * w0.y);
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos3.x, texPos0.y), 0.0f).xyz * w3.x) * w0.y);
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos0.x, texPos12.y), 0.0f).xyz * w0.x) * w12.y);
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos12.x, texPos12.y), 0.0f).xyz * w12.x) * w12.y);
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos3.x, texPos12.y), 0.0f).xyz * w3.x) * w12.y);
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos0.x, texPos3.y), 0.0f).xyz * w0.x) * w3.y);
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos12.x, texPos3.y), 0.0f).xyz * w12.x) * w3.y);
    result += ((history_tex.SampleLevel(smp_bilinear, float2(texPos3.x, texPos3.y), 0.0f).xyz * w3.x) * w3.y);
    return max(result, 0.0f.xxx);
}

float3 clip_aabb(float3 aabb_min, float3 aabb_max, float3 history_color, float3 average)
{
    float3 r = history_color - average;
    float3 rmax = aabb_max - average;
    float3 rmin = aabb_min - average;
    if (r.x > (rmax.x + 9.9999999747524270787835121154785e-07f))
    {
        r *= (rmax.x / r.x);
    }
    if (r.y > (rmax.y + 9.9999999747524270787835121154785e-07f))
    {
        r *= (rmax.y / r.y);
    }
    if (r.z > (rmax.z + 9.9999999747524270787835121154785e-07f))
    {
        r *= (rmax.z / r.z);
    }
    if (r.x < (rmin.x - 9.9999999747524270787835121154785e-07f))
    {
        r *= (rmin.x / r.x);
    }
    if (r.y < (rmin.y - 9.9999999747524270787835121154785e-07f))
    {
        r *= (rmin.y / r.y);
    }
    if (r.z < (rmin.z - 9.9999999747524270787835121154785e-07f))
    {
        r *= (rmin.z / r.z);
    }
    return average + r;
}

void frag_main()
{
    out_image = float4(0.0f, 0.0f, 0.0f, 1.0f);
    float2 in_uv = inUV;
    if (config_enable_side_by_side_debug_view != 0u)
    {
        float2 param = float2(0.5f, 0.0f);
        float2 param_1 = float2(0.5f, 1.0f);
        float2 param_2 = inUV;
        float param_3 = 0.00200000009499490261077880859375f;
        float _529 = draw_line(param, param_1, param_2, param_3);
        out_image += _529.xxxx;
        if (in_uv.x < 0.5f)
        {
            in_uv.x += 0.25f;
            float3 current_color = current_tex.Sample(smp_nearest, in_uv).xyz;
            float4 _553 = out_image;
            float3 _555 = _553.xyz + current_color;
            out_image.x = _555.x;
            out_image.y = _555.y;
            out_image.z = _555.z;
            return;
        }
        in_uv.x -= 0.25f;
    }
    if (!(config_has_history_data != 0u))
    {
        float3 current_color_1 = current_tex.Sample(smp_nearest, in_uv).xyz;
        float4 _583 = out_image;
        float3 _585 = _583.xyz + current_color_1;
        out_image.x = _585.x;
        out_image.y = _585.y;
        out_image.z = _585.z;
        return;
    }
    float3 color_sum = 0.0f.xxx;
    float color_weight = 0.0f;
    float3 m1 = 0.0f.xxx;
    float3 m2 = 0.0f.xxx;
    float m_weight = 0.0f;
    uint _605_dummy_parameter;
    float2 texture_size = float2(int2(spvTextureSize(current_tex, uint(0), _605_dummy_parameter)));
    float2 texel_size = 1.0f.xx / texture_size;
    float3 current_color_2;
    int y = -1;
    for (;;)
    {
        if (y <= 1)
        {
            int x = -1;
            for (;;)
            {
                if (x <= 1)
                {
                    float2 sample_uv = clamp(in_uv + (float2(float(x), float(y)) * texel_size), 0.0f.xx, 1.0f.xx);
                    float3 color = current_tex.Sample(smp_nearest, sample_uv).xyz;
                    float3 param_4 = color;
                    float luminance = rgb_to_luminosity(param_4);
                    float weight = 1.0f / (1.0f + luminance);
                    color_sum += (color * weight);
                    color_weight += weight;
                    m1 += color;
                    m2 += (color * color);
                    m_weight += 1.0f;
                    if ((x == 0) && (y == 0))
                    {
                        current_color_2 = color;
                    }
                    x++;
                    continue;
                }
                else
                {
                    break;
                }
            }
            y++;
            continue;
        }
        else
        {
            break;
        }
    }
    float closest_depth = -1.0f;
    float2 closest_velocity_ndc = 0.0f.xx;
    int y_1 = -1;
    for (;;)
    {
        if (y_1 <= 1)
        {
            int x_1 = -1;
            for (;;)
            {
                if (x_1 <= 1)
                {
                    float2 sample_uv_1 = clamp(in_uv + (float2(float(x_1), float(y_1)) * texel_size), 0.0f.xx, 1.0f.xx);
                    float2 v = velocity_tex.Sample(smp_nearest, sample_uv_1).xy;
                    float d = depth_tex.Sample(smp_nearest, sample_uv_1).x;
                    if (d > closest_depth)
                    {
                        closest_depth = d;
                        closest_velocity_ndc = v;
                    }
                    x_1++;
                    continue;
                }
                else
                {
                    break;
                }
            }
            y_1++;
            continue;
        }
        else
        {
            break;
        }
    }
    float depth = closest_depth;
    float2 velocity_ndc = closest_velocity_ndc;
    if (depth <= 0.0f)
    {
        float4 _758 = out_image;
        float3 _760 = _758.xyz + current_color_2;
        out_image.x = _760.x;
        out_image.y = _760.y;
        out_image.z = _760.z;
        return;
    }
    else
    {
        bool _772 = velocity_ndc.x > 9000000.0f;
        bool _778;
        if (_772)
        {
            _778 = velocity_ndc.y > 9000000.0f;
        }
        else
        {
            _778 = _772;
        }
        if (_778)
        {
            float2 viewport_size = float2(float(config_viewport_width), float(config_viewport_height));
            float2 fragcoord_ndc = ((gl_FragCoord.xy / viewport_size) * 2.0f) - 1.0f.xx;
            fragcoord_ndc.y *= (-1.0f);
            float4 new_position_ndc = float4(fragcoord_ndc, depth, 1.0f);
            float4 position_ws = mul(new_position_ndc, config_current_view_proj_inv);
            position_ws /= position_ws.w.xxxx;
            float4 previous_position_ndc = mul(float4(position_ws.xyz, 1.0f), config_previous_view_proj);
            previous_position_ndc /= previous_position_ndc.w.xxxx;
            velocity_ndc = fragcoord_ndc - previous_position_ndc.xy;
        }
    }
    float3 history_color = current_color_2;
    float2 history_sample_uv = inUV - (velocity_ndc * float2(0.5f, -0.5f));
    bool _853 = history_sample_uv.x <= 1.0f;
    bool _859;
    if (_853)
    {
        _859 = history_sample_uv.x >= 0.0f;
    }
    else
    {
        _859 = _853;
    }
    bool _865;
    if (_859)
    {
        _865 = history_sample_uv.y <= 1.0f;
    }
    else
    {
        _865 = _859;
    }
    bool _871;
    if (_865)
    {
        _871 = history_sample_uv.y >= 0.0f;
    }
    else
    {
        _871 = _865;
    }
    if (_871)
    {
        float2 param_5 = history_sample_uv;
        float2 param_6 = texel_size;
        history_color = sample_history_catmull_rom(param_5, param_6);
    }
    float3 mu = m1 / m_weight.xxx;
    float3 sigma = sqrt(abs((m2 / m_weight.xxx) - (mu * mu)));
    float3 min_c = mu - (sigma * 1.0f);
    float3 max_c = mu + (sigma * 1.0f);
    float3 param_7 = min_c;
    float3 param_8 = max_c;
    float3 param_9 = history_color;
    float3 param_10 = mu;
    history_color = clip_aabb(param_7, param_8, param_9, param_10);
    float current_weight = config_history_weight + min(length(velocity_ndc) * config_history_weight_velocity_adjust_multiplier, config_history_weight_velocity_adjust_max);
    float history_weight = 1.0f - current_weight;
    float3 param_11 = current_color_2;
    current_weight *= (1.0f / (1.0f + rgb_to_luminosity(param_11)));
    float3 param_12 = history_color;
    history_weight *= (1.0f / (1.0f + rgb_to_luminosity(param_12)));
    float3 blended_color = ((current_color_2 * current_weight) + (history_color * history_weight)) / (current_weight + history_weight).xxx;
    out_image += float4(blended_color, 1.0f);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    gl_FragCoord = stage_input.gl_FragCoord;
    gl_FragCoord.w = 1.0 / gl_FragCoord.w;
    inUV = stage_input.inUV;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_image = out_image;
    return stage_output;
}
    �0      #pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct Config
{
    float4x4 current_view_proj_inv;
    float4x4 previous_view_proj;
    float2 jitter_amount;
    uint has_history_data;
    uint enable_side_by_side_debug_view;
    float history_weight;
    float history_weight_velocity_adjust_multiplier;
    float history_weight_velocity_adjust_max;
    uint viewport_width;
    uint viewport_height;
};

struct spvDescriptorSetBuffer0
{
    texture2d<float> history_tex [[id(0)]];
    texture2d<float> current_tex [[id(1)]];
    texture2d<float> velocity_tex [[id(2)]];
    texture2d<float> depth_tex [[id(3)]];
    constant Config* config [[id(6)]];
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
float draw_line(thread const float2& p1, thread const float2& p2, thread const float2& x, thread float& width)
{
    float2 p1_to_x = x - p1;
    float2 line = p2 - p1;
    float2 line_dir = normalize(line);
    float2 p1_to_nearest_on_line = line_dir * dot(p1_to_x, line_dir);
    float2 nearest_on_line = p1 + p1_to_nearest_on_line;
    float dist_nearest_from_p2 = length(p2 - nearest_on_line);
    float max_dist_from_p = length(line);
    if (dist_nearest_from_p2 > max_dist_from_p)
    {
        return 0.0;
    }
    float dist_nearest_from_p1 = length(p1 - nearest_on_line);
    if (dist_nearest_from_p1 > max_dist_from_p)
    {
        return 0.0;
    }
    width /= 2.0;
    float width_min = width * 0.949999988079071044921875;
    float width_max = width * 1.0499999523162841796875;
    float dist_from_line = length(x - nearest_on_line);
    return fast::clamp(1.0 - ((dist_from_line - width_min) / (width_max - width_min)), 0.0, 1.0);
}

static inline __attribute__((always_inline))
float rgb_to_luminosity(thread const float3& color)
{
    return dot(color, float3(0.2989999949932098388671875, 0.58700001239776611328125, 0.114000000059604644775390625));
}

static inline __attribute__((always_inline))
float3 sample_history_catmull_rom(thread const float2& uv, thread const float2& texel_size, thread texture2d<float> history_tex, thread sampler smp_bilinear)
{
    float2 samplePos = uv / texel_size;
    float2 texPos1 = floor(samplePos - float2(0.5)) + float2(0.5);
    float2 f = samplePos - texPos1;
    float2 w0 = f * (float2(-0.5) + (f * (float2(1.0) - (f * 0.5))));
    float2 w1 = float2(1.0) + ((f * f) * (float2(-2.5) + (f * 1.5)));
    float2 w2 = f * (float2(0.5) + (f * (float2(2.0) - (f * 1.5))));
    float2 w3 = (f * f) * (float2(-0.5) + (f * 0.5));
    float2 w12 = w1 + w2;
    float2 offset12 = w2 / (w1 + w2);
    float2 texPos0 = texPos1 - float2(1.0);
    float2 texPos3 = texPos1 + float2(2.0);
    float2 texPos12 = texPos1 + offset12;
    texPos0 *= texel_size;
    texPos3 *= texel_size;
    texPos12 *= texel_size;
    float3 result = float3(0.0);
    result += ((history_tex.sample(smp_bilinear, float2(texPos0.x, texPos0.y), level(0.0)).xyz * w0.x) * w0.y);
    result += ((history_tex.sample(smp_bilinear, float2(texPos12.x, texPos0.y), level(0.0)).xyz * w12.x) * w0.y);
    result += ((history_tex.sample(smp_bilinear, float2(texPos3.x, texPos0.y), level(0.0)).xyz * w3.x) * w0.y);
    result += ((history_tex.sample(smp_bilinear, float2(texPos0.x, texPos12.y), level(0.0)).xyz * w0.x) * w12.y);
    result += ((history_tex.sample(smp_bilinear, float2(texPos12.x, texPos12.y), level(0.0)).xyz * w12.x) * w12.y);
    result += ((history_tex.sample(smp_bilinear, float2(texPos3.x, texPos12.y), level(0.0)).xyz * w3.x) * w12.y);
    result += ((history_tex.sample(smp_bilinear, float2(texPos0.x, texPos3.y), level(0.0)).xyz * w0.x) * w3.y);
    result += ((history_tex.sample(smp_bilinear, float2(texPos12.x, texPos3.y), level(0.0)).xyz * w12.x) * w3.y);
    result += ((history_tex.sample(smp_bilinear, float2(texPos3.x, texPos3.y), level(0.0)).xyz * w3.x) * w3.y);
    return fast::max(result, float3(0.0));
}

static inline __attribute__((always_inline))
float3 clip_aabb(thread const float3& aabb_min, thread const float3& aabb_max, thread const float3& history_color, thread const float3& average)
{
    float3 r = history_color - average;
    float3 rmax = aabb_max - average;
    float3 rmin = aabb_min - average;
    if (r.x > (rmax.x + 9.9999999747524270787835121154785e-07))
    {
        r *= (rmax.x / r.x);
    }
    if (r.y > (rmax.y + 9.9999999747524270787835121154785e-07))
    {
        r *= (rmax.y / r.y);
    }
    if (r.z > (rmax.z + 9.9999999747524270787835121154785e-07))
    {
        r *= (rmax.z / r.z);
    }
    if (r.x < (rmin.x - 9.9999999747524270787835121154785e-07))
    {
        r *= (rmin.x / r.x);
    }
    if (r.y < (rmin.y - 9.9999999747524270787835121154785e-07))
    {
        r *= (rmin.y / r.y);
    }
    if (r.z < (rmin.z - 9.9999999747524270787835121154785e-07))
    {
        r *= (rmin.z / r.z);
    }
    return average + r;
}

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], float4 gl_FragCoord [[position]])
{
    constexpr sampler smp_bilinear(filter::linear, mip_filter::nearest, compare_func::never, max_anisotropy(1), lod_clamp(0.0, 0.0));
    constexpr sampler smp_nearest(mip_filter::nearest, compare_func::never, max_anisotropy(1), lod_clamp(0.0, 0.0));
    main0_out out = {};
    out.out_image = float4(0.0, 0.0, 0.0, 1.0);
    float2 in_uv = in.inUV;
    if ((*spvDescriptorSet0.config).enable_side_by_side_debug_view != 0u)
    {
        float2 param = float2(0.5, 0.0);
        float2 param_1 = float2(0.5, 1.0);
        float2 param_2 = in.inUV;
        float param_3 = 0.00200000009499490261077880859375;
        float _529 = draw_line(param, param_1, param_2, param_3);
        out.out_image += float4(_529);
        if (in_uv.x < 0.5)
        {
            in_uv.x += 0.25;
            float3 current_color = spvDescriptorSet0.current_tex.sample(smp_nearest, in_uv).xyz;
            float4 _553 = out.out_image;
            float3 _555 = _553.xyz + current_color;
            out.out_image.x = _555.x;
            out.out_image.y = _555.y;
            out.out_image.z = _555.z;
            return out;
        }
        in_uv.x -= 0.25;
    }
    if (!((*spvDescriptorSet0.config).has_history_data != 0u))
    {
        float3 current_color_1 = spvDescriptorSet0.current_tex.sample(smp_nearest, in_uv).xyz;
        float4 _583 = out.out_image;
        float3 _585 = _583.xyz + current_color_1;
        out.out_image.x = _585.x;
        out.out_image.y = _585.y;
        out.out_image.z = _585.z;
        return out;
    }
    float3 color_sum = float3(0.0);
    float color_weight = 0.0;
    float3 m1 = float3(0.0);
    float3 m2 = float3(0.0);
    float m_weight = 0.0;
    float2 texture_size = float2(int2(spvDescriptorSet0.current_tex.get_width(), spvDescriptorSet0.current_tex.get_height()));
    float2 texel_size = float2(1.0) / texture_size;
    float3 current_color_2;
    int y = -1;
    for (;;)
    {
        if (y <= 1)
        {
            int x = -1;
            for (;;)
            {
                if (x <= 1)
                {
                    float2 sample_uv = fast::clamp(in_uv + (float2(float(x), float(y)) * texel_size), float2(0.0), float2(1.0));
                    float3 color = spvDescriptorSet0.current_tex.sample(smp_nearest, sample_uv).xyz;
                    float3 param_4 = color;
                    float luminance = rgb_to_luminosity(param_4);
                    float weight = 1.0 / (1.0 + luminance);
                    color_sum += (color * weight);
                    color_weight += weight;
                    m1 += color;
                    m2 += (color * color);
                    m_weight += 1.0;
                    if ((x == 0) && (y == 0))
                    {
                        current_color_2 = color;
                    }
                    x++;
                    continue;
                }
                else
                {
                    break;
                }
            }
            y++;
            continue;
        }
        else
        {
            break;
        }
    }
    float closest_depth = -1.0;
    float2 closest_velocity_ndc = float2(0.0);
    int y_1 = -1;
    for (;;)
    {
        if (y_1 <= 1)
        {
            int x_1 = -1;
            for (;;)
            {
                if (x_1 <= 1)
                {
                    float2 sample_uv_1 = fast::clamp(in_uv + (float2(float(x_1), float(y_1)) * texel_size), float2(0.0), float2(1.0));
                    float2 v = spvDescriptorSet0.velocity_tex.sample(smp_nearest, sample_uv_1).xy;
                    float d = spvDescriptorSet0.depth_tex.sample(smp_nearest, sample_uv_1).x;
                    if (d > closest_depth)
                    {
                        closest_depth = d;
                        closest_velocity_ndc = v;
                    }
                    x_1++;
                    continue;
                }
                else
                {
                    break;
                }
            }
            y_1++;
            continue;
        }
        else
        {
            break;
        }
    }
    float depth = closest_depth;
    float2 velocity_ndc = closest_velocity_ndc;
    if (depth <= 0.0)
    {
        float4 _758 = out.out_image;
        float3 _760 = _758.xyz + current_color_2;
        out.out_image.x = _760.x;
        out.out_image.y = _760.y;
        out.out_image.z = _760.z;
        return out;
    }
    else
    {
        bool _772 = velocity_ndc.x > 9000000.0;
        bool _778;
        if (_772)
        {
            _778 = velocity_ndc.y > 9000000.0;
        }
        else
        {
            _778 = _772;
        }
        if (_778)
        {
            float2 viewport_size = float2(float((*spvDescriptorSet0.config).viewport_width), float((*spvDescriptorSet0.config).viewport_height));
            float2 fragcoord_ndc = ((gl_FragCoord.xy / viewport_size) * 2.0) - float2(1.0);
            fragcoord_ndc.y *= (-1.0);
            float4 new_position_ndc = float4(fragcoord_ndc, depth, 1.0);
            float4 position_ws = (*spvDescriptorSet0.config).current_view_proj_inv * new_position_ndc;
            position_ws /= float4(position_ws.w);
            float4 previous_position_ndc = (*spvDescriptorSet0.config).previous_view_proj * float4(position_ws.xyz, 1.0);
            previous_position_ndc /= float4(previous_position_ndc.w);
            velocity_ndc = fragcoord_ndc - previous_position_ndc.xy;
        }
    }
    float3 history_color = current_color_2;
    float2 history_sample_uv = in.inUV - (velocity_ndc * float2(0.5, -0.5));
    bool _853 = history_sample_uv.x <= 1.0;
    bool _859;
    if (_853)
    {
        _859 = history_sample_uv.x >= 0.0;
    }
    else
    {
        _859 = _853;
    }
    bool _865;
    if (_859)
    {
        _865 = history_sample_uv.y <= 1.0;
    }
    else
    {
        _865 = _859;
    }
    bool _871;
    if (_865)
    {
        _871 = history_sample_uv.y >= 0.0;
    }
    else
    {
        _871 = _865;
    }
    if (_871)
    {
        float2 param_5 = history_sample_uv;
        float2 param_6 = texel_size;
        history_color = sample_history_catmull_rom(param_5, param_6, spvDescriptorSet0.history_tex, smp_bilinear);
    }
    float3 mu = m1 / float3(m_weight);
    float3 sigma = sqrt(abs((m2 / float3(m_weight)) - (mu * mu)));
    float3 min_c = mu - (sigma * 1.0);
    float3 max_c = mu + (sigma * 1.0);
    float3 param_7 = min_c;
    float3 param_8 = max_c;
    float3 param_9 = history_color;
    float3 param_10 = mu;
    history_color = clip_aabb(param_7, param_8, param_9, param_10);
    float current_weight = (*spvDescriptorSet0.config).history_weight + fast::min(length(velocity_ndc) * (*spvDescriptorSet0.config).history_weight_velocity_adjust_multiplier, (*spvDescriptorSet0.config).history_weight_velocity_adjust_max);
    float history_weight = 1.0 - current_weight;
    float3 param_11 = current_color_2;
    current_weight *= (1.0 / (1.0 + rgb_to_luminosity(param_11)));
    float3 param_12 = history_color;
    history_weight *= (1.0 / (1.0 + rgb_to_luminosity(param_12)));
    float3 blended_color = ((current_color_2 * current_weight) + (history_color * history_weight)) / float3(current_weight + history_weight);
    out.out_image += float4(blended_color, 1.0);
    return out;
}

    L.      #     a             2        GLSL.std.450                     main    �  �            G  �   "       G  �   !       G  �   "       G  �   !      G  �         G  �         H  �         H  �      #       H  �            H  �        H  �     #   @   H  �           H  �     #   �   H  �     #   �   H  �     #   �   H  �     #   �   H  �     #   �   H  �     #   �   H  �     #   �   H  �  	   #   �   G  �     G  �  "       G  �  !      G    "       G    !      G     "       G     !      G  �  "       G  �  !      G  �  "       G  �  !      G               !                                       +     $   ��>+     %   �E?+     &   �x�=,     '   $   %   &     J   +     N       +     [      @+     k     �?+     }      ?+     �      �+     �      �+     �     �?,     �   N   N   N    	 �                               �       �   ;  �   �         �      �       �   ;  �   �         �   �     �           +  �   �       +  �   �        �         +     �  �7�5+  �   �        �     �   ;  �  �     ,  �   �  N   N   N   k      �        ;  �  �       �  �        �  �  �     �   �            �   �      �     �  ;  �  �       �         +  �                �   ,       }   N   +         �>;  �         ;  �             *        +  �  6     +  �  X        Z  �     +  �  c  ����+  �  j     +     �    ��,     �  N   N   ;  �   �      ;  �   �      +       @T	K+  �       +  �    	           �   ;            +     �  ,     N  }   �   +  �  �        �        +  �  �     +  �  �          �  ,     F  N   k   ,     G  k   k   +     J  l	y:,     P  }   }   ,     Q  �   �   ,     R  �   �   ,     S  [   [   +     T  @F+     W     @,     X  W  W    Z  J        ^  J      6               �     �  �      �  �   �  �  �  >  �  �  =     �  �  A      �     =  �       �  J       �   �        �        �    �        �  �   �  �  �  �     �  �         �     E   F  �     �  �  �  �     �  �  �  �     �    �  �     �  F  �       �     B   �       �     B   F  �  J   �  �  �  �  �      �  �  �  �  �  �  �    �  �       �  �       �     B   �  �  J     �  �  �        �        �    �    �    �       �  �            B     �     U  J              2   U  T  k             +     N   k   �    �    � 	    �  N   �  N         =  �     �  P  �     �  �  �  �  �  �         >  �    Q       �      �  J       }   �        �        �    �           R     �    �      =  �       =  �   !     V  �   "    !  W  �   $  "  �  O     %  $  $            =  �   '  �  O     (  '  '            �     )  (  %  A  *  +  �  �   Q     ,  )      >  +  ,  A  *  -  �  �   Q     .  )     >  -  .  A  *  /  �  �  Q     0  )     >  /  0  �  �  �    �     4      R     �  4  �      �    �    �     (  �  �  �    A    7  �  6  =  �   8  7  �  J   9  8  �   �  J   :  9  �  <      �  :  ;  <  �  ;  =  �   >    =  �   ?     V  �   @  >  ?  W  �   B  @  (  O     C  B  B            =  �   E  �  O     F  E  E            �     G  F  C  A  *  H  �  �   Q     I  G      >  H  I  A  *  J  �  �   Q     K  G     >  J  K  A  *  L  �  �  Q     M  G     >  L  M  �  �  �  <  =  �   U    =  �   V     V  �   W  U  V  d  �   Y  W  g  Z  [  Y  X  o     \  [  �     `  G  \  �  d  �  d  �       �   <  /  g  �     �  N   <  0  g  �     �  �   <  .  g  �     �  �  <  4  g  �  �  �  c  <  �  g  �  J   k  �  j  �  f  g      �  k  e  f  �  e  �  m  �  m  �     0  �  e  �  n  �     /    e  �  n  �     .  �  e  �  n  �  �  +  c  e  �  n  �     4  �  e  \  n  �  J   s  +  j  �  o  n      �  s  n  o  �  n  o     w  +  o     y  �  P     z  w  y       }     2   z  `  (       �     +   }  �  G  V  �   �  U  V  W  �   �  �  �  O     �  �  �            �     �  .  �       �     2   �  �  /  �     �  0  k   �  J   �  +  X  �  J   �  �  X  �  J   �  �  �  P  Z  [  �  �  �  �     \  [  �  4  �  �  �  +  j  �  m  �  o  �  g  �  g  �  �  �  �  j  �  d  �  f  �  �  �  �  �     �  �  f  2  �  �     �  �  f  )  �  �  �  �  c  f  �  �  �  J   �  �  j  �  �  �      �  �  �  �  �  �  �  �  �  �  �     )  �  �  ]  �  �  �  #  c  �  �  �  �     2  �  �  `  �  �  J   �  #  j  �  �  �      �  �  �  �  �  �  o     �  #  o     �  �  P     �  �  �       �     2   �  `  (       �     +   �  �  G  =  �   �  �  V  �   �  �  V  W  �   �  �  �  O     �  �  �         =  �   �  �  V  �   �  �  V  W  �   �  �  �  Q     �  �      �  J   �  �  )  �     ]  �  �  )  P  ^  _  �  �  �     `  _  �  2  �  �  �  #  j  �  �  �  �  �  �  �  �  �  �  �  �  j  �  �  �  �  �  J   �  �  N   �  �      �  �  �  �  �  �  =  �   �  �  O     �  �  �            �     �  �  �  A  *  �  �  �   Q     �  �      >  �  �  A  *  �  �  �   Q     �  �     >  �  �  A  *  �  �  �  Q     �  �     >  �  �  �  �  �  �  Q        �      �  J          �        �        �    Q       �     �  J         �    �    �  J       �      �  
      �    	  
  �  	  A      �    =  �       p         A      �    =  �       p         P           =  �       O                  �           �         [   �         G  Q             �     !     �  R     �  !       Q     '        P  �   )  '  !  �  k   A  +  ,  �  X  =  �  -  ,  �  �   /  -  )  Q     2  /     P  �   4  2  2  2  2  �  �   5  /  4  A  +  7  �  j  =  �  8  7  Q     ;  5      Q     <  5     Q     =  5     P  �   >  ;  <  =  k   �  �   ?  8  >  Q     A  ?     P  �   C  A  A  A  A  �  �   D  ?  C  O     G  D  D         �     H  �  G  �  
  �  
  �     �  �    H  	  �  �  �  �       V  �       P     2   V  N  �  Q     R  P      �  J   S  R  k   �  U      �  S  T  U  �  T  �  J   X  R  N   �  U  �  U  �  J   Y  S  �  X  T  �  [      �  Y  Z  [  �  Z  Q     ]  P     �  J   ^  ]  k   �  [  �  [  �  J   _  Y  U  ^  Z  �  a      �  _  `  a  �  `  Q     c  P     �  J   d  c  N   �  a  �  a  �  J   e  _  [  d  `  �  g      �  e  f  g  �  f  �     .  P  `  �     1  .  P       2        1  �     4  2  P  �     7  .  4  �     ;  7  }   �     =  G  ;       @     2   7  =  Q  �     A  7  @  �     D  7  7  �     F  7  �   �     H  R  F       K     2   D  H  G  �     Q  S  F       T     2   7  Q  P  �     U  7  T  �     \  Q  ;  �     ]  D  \       `     2   7  T  K  �     e  U  `  �     h  2  Q  �     k  2  X  �     n  4  e  �     q  h  `  �     t  k  `  �     w  n  `  =  �   x  �   =  �   y  �   V  �   z  x  y  Q     |  q      Q     ~  q     X  �   �  z  q     N   O     �  �  �            Q     �  A      �     �  �  �  Q     �  A     �     �  �  �  V  �   �  x  y  Q     �  w      P     �  �  ~  X  �   �  �  �     N   O     �  �  �            Q     �  `      �     �  �  �  �     �  �  �  �     �  �  �  V  �   �  x  y  Q     �  t      P     �  �  ~  X  �   �  �  �     N   O     �  �  �            Q     �  ]      �     �  �  �  �     �  �  �  �     �  �  �  V  �   �  x  y  Q     �  w     P     �  |  �  X  �   �  �  �     N   O     �  �  �            �     �  �  �  Q     �  `     �     �  �  �  �     �  �  �  V  �   �  x  y  X  �   �  �  w     N   O     �  �  �            �     �  �  �  �     �  �  �  �     �  �  �  V  �   �  x  y  P     �  �  �  X  �   �  �  �     N   O     �  �  �            �     �  �  �  �     �  �  �  �     �  �  �  V  �   �  x  y  Q     �  t     P     �  |  �  X  �   �  �  �     N   O     �  �  �            �     �  �  �  Q     �  ]     �     �  �  �  �     �  �  �  V  �   �  x  y  P     �  �  �  X  �   �  �  �     N   O     �  �  �            �       �  �  �         �  �       �    V  �   
  x  y  X  �     
  t     N   O                     �         �  �         �  �                     (     �   �  g  �  g  �       �  a    f  P     p  �  �  �  �     q  �  p  �     v    p       Y  q       z     2   Y  q  v       {        z       |        {  �     �  |  k   �     �  q  �  �     �  q  �  �     $    q  �     '  �  q  �     *  �  q  Q     ,  $      Q     .  '      �     /  .  �  �  J   0  ,  /  �  9      �  0  1  9  �  1  �     6  .  ,  �     8  $  6  �  9  �  9  �     	  $  g  8  1  Q     ;  	     Q     =  '     �     >  =  �  �  J   ?  ;  >  �  H      �  ?  @  H  �  @  �     E  =  ;  �     G  	  E  �  H  �  H  �     
  	  9  G  @  Q     J  
     Q     L  '     �     M  L  �  �  J   N  J  M  �  W      �  N  O  W  �  O  �     T  L  J  �     V  
  T  �  W  �  W  �       
  H  V  O  Q     Y        Q     [  *      �     \  [  �  �  J   ]  Y  \  �  f      �  ]  ^  f  �  ^  �     c  [  Y  �     e    c  �  f  �  f  �         W  e  ^  Q     h       Q     j  *     �     k  j  �  �  J   l  h  k  �  u      �  l  m  u  �  m  �     r  j  h  �     t    r  �  u  �  u  �         f  t  m  Q     w       Q     y  *     �     z  y  �  �  J   {  w  z  �  �      �  {  |  �  �  |  �     �  y  w  �     �    �  �  �  �  �  �         u  �  |  �     �  q    A  �  �  �  �  =     �  �       �     B   �  A  �  �  �  �  =     �  �  �     �  �  �  A  �  �  �  �  =     �  �       �     %   �  �  �     �  �  �  �     �  k   �  �     �  �  '   �     �  k   �  �     �  k   �  �     �  �  �  �     �  �  '   �     �  k   �  �     �  k   �  �     �  �  �  �     �  �  �  �     �  �  �  �     �  �  �       �     2   �  �  �  P     �  �  �  �  �     �  �  �  Q     �  �      Q     �  �     Q     �  �     P  �   �  �  �  �  k   =  �   �  �  �  �   �  �  �  >  �  �  �  �  �  �  �  8                                               history_tex               history_tex                                    current_tex              current_tex                                    velocity_tex              velocity_tex                             	       depth_tex       	       depth_tex                                    smp_nearest              smp_nearest                                    smp_bilinear              smp_bilinear         �                           Config              Config 
              Config.current_view_proj_inv           Config.previous_view_proj@          Config.jitter_amount�          Config.has_history_data�   %       Config.enable_side_by_side_debug_view�          Config.history_weight�   0       Config.history_weight_velocity_adjust_multiplier�   )       Config.history_weight_velocity_adjust_max�          Config.viewport_width�          Config.viewport_height�                      main                                          history_tex               history_tex                                      current_tex              current_tex                                      velocity_tex              velocity_tex                               	       depth_tex       	       depth_tex                                      smp_nearest              smp_nearest                                                                             smp_bilinear              smp_bilinear                                                �                           Config              Config 
              Config.current_view_proj_inv           Config.previous_view_proj@          Config.jitter_amount�          Config.has_history_data�   %       Config.enable_side_by_side_debug_view�          Config.history_weight�   0       Config.history_weight_velocity_adjust_multiplier�   )       Config.history_weight_velocity_adjust_max�          Config.viewport_width�          Config.viewport_height�    �                                                        history_tex               history_tex                                    current_tex              current_tex                                    velocity_tex              velocity_tex                             	       depth_tex       	       depth_tex                                    smp_nearest              smp_nearest                                    smp_bilinear              smp_bilinear         �                           Config              Config 
              Config.current_view_proj_inv           Config.previous_view_proj@          Config.jitter_amount�          Config.has_history_data�   %       Config.enable_side_by_side_debug_view�          Config.history_weight�   0       Config.history_weight_velocity_adjust_multiplier�   )       Config.history_weight_velocity_adjust_max�          Config.viewport_width�          Config.viewport_height�                      main                                          history_tex               history_tex                                      current_tex              current_tex                                      velocity_tex              velocity_tex                               	       depth_tex       	       depth_tex                                      smp_nearest              smp_nearest                                                                             smp_bilinear              smp_bilinear                                                �                           Config              Config 
              Config.current_view_proj_inv           Config.previous_view_proj@          Config.jitter_amount�          Config.has_history_data�   %       Config.enable_side_by_side_debug_view�          Config.history_weight�   0       Config.history_weight_velocity_adjust_multiplier�   )       Config.history_weight_velocity_adjust_max�          Config.viewport_width�          Config.viewport_height�    �                                                        history_tex               history_tex                                    current_tex              current_tex                                    velocity_tex              velocity_tex                             	       depth_tex       	       depth_tex                                    smp_nearest              smp_nearest                                    smp_bilinear              smp_bilinear         �                           Config              Config 
              Config.current_view_proj_inv           Config.previous_view_proj@          Config.jitter_amount�          Config.has_history_data�   %       Config.enable_side_by_side_debug_view�          Config.history_weight�   0       Config.history_weight_velocity_adjust_multiplier�   )       Config.history_weight_velocity_adjust_max�          Config.viewport_width�          Config.viewport_height�                      main                                          history_tex               history_tex                                      current_tex              current_tex                                      velocity_tex              velocity_tex                               	       depth_tex       	       depth_tex                                      smp_nearest              smp_nearest                                                                             smp_bilinear              smp_bilinear                                                �                           Config              Config 
              Config.current_view_proj_inv           Config.previous_view_proj@          Config.jitter_amount�          Config.has_history_data�   %       Config.enable_side_by_side_debug_view�          Config.history_weight�   0       Config.history_weight_velocity_adjust_multiplier�   )       Config.history_weight_velocity_adjust_max�          Config.viewport_width�          Config.viewport_height�    �                    taa.frag