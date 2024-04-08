                       �""D@"���ȞE��e����      ��      struct DirectionalLight
{
    float3 direction_ws;
    float intensity;
    float4 color;
    float3 direction_vs;
    int shadow_map;
};

struct ShadowMap2DData
{
    float2 uv_min;
    float2 uv_max;
    row_major float4x4 shadow_map_view_proj;
    float3 shadow_map_light_dir;
};

struct ShadowMapCubeData
{
    float4 uv_min_uv_max[6];
    float cube_map_projection_near_z;
    float cube_map_projection_far_z;
};

struct LightInList
{
    float3 position_ws;
    float range;
    float3 position_vs;
    float intensity;
    float4 color;
    float3 spotlight_direction_ws;
    float spotlight_half_angle;
    float3 spotlight_direction_vs;
    int shadow_map;
};

struct ClusterMeta
{
    uint count;
    uint first_light;
};

struct LightBinningOutput
{
    uint data_write_ptr;
    uint pad0;
    uint pad1;
    uint pad2;
    ClusterMeta offsets[3072];
    uint data[1572864];
};

struct DrawData
{
    uint transform_index;
    uint material_index;
};

struct MaterialDbEntry
{
    float4 base_color_factor;
    float3 emissive_factor;
    float metallic_factor;
    float roughness_factor;
    float normal_texture_scale;
    float alpha_threshold;
    bool enable_alpha_blend;
    bool enable_alpha_clip;
    int color_texture;
    bool base_color_texture_has_alpha_channel;
    int metallic_roughness_texture;
    int normal_texture;
    int emissive_texture;
};

struct MaterialDbEntry_1
{
    float4 base_color_factor;
    float3 emissive_factor;
    float metallic_factor;
    float roughness_factor;
    float normal_texture_scale;
    float alpha_threshold;
    uint enable_alpha_blend;
    uint enable_alpha_clip;
    int color_texture;
    uint base_color_texture_has_alpha_channel;
    int metallic_roughness_texture;
    int normal_texture;
    int emissive_texture;
};

struct Transform
{
    row_major float4x4 model_matrix;
};

cbuffer PerViewData : register(b0, space0)
{
    row_major float4x4 per_view_data_view : packoffset(c0);
    row_major float4x4 per_view_data_view_proj : packoffset(c4);
    float4 per_view_data_ambient_light : packoffset(c8);
    float2 per_view_data_jitter_amount : packoffset(c9);
    uint per_view_data_viewport_width : packoffset(c9.z);
    uint per_view_data_viewport_height : packoffset(c9.w);
    float per_view_data_mip_bias : packoffset(c10);
    float per_view_data_ndf_filter_amount : packoffset(c10.y);
    uint per_view_data_directional_light_count : packoffset(c10.z);
    uint per_view_data_use_clustered_lighting : packoffset(c10.w);
    DirectionalLight per_view_data_directional_lights[8] : packoffset(c11);
    ShadowMap2DData per_view_data_shadow_map_2d_data[96] : packoffset(c35);
    ShadowMapCubeData per_view_data_shadow_map_cube_data[32] : packoffset(c611);
};

RWByteAddressBuffer all_lights : register(u6, space0);
RWByteAddressBuffer light_bin_output : register(u5, space0);
RWByteAddressBuffer all_draw_data : register(u1, space2);
RWByteAddressBuffer all_materials : register(u0, space3);
RWByteAddressBuffer all_transforms : register(u0, space2);
cbuffer PushConstantData : register(b0, space4)
{
    uint push_constants_instance_offset : packoffset(c0);
};

Texture2D<float4> all_material_textures[768] : register(t1, space3);
SamplerState smp : register(s1, space0);
Texture2D<float4> shadow_map_atlas : register(t4, space0);
SamplerComparisonState smp_depth_nearest : register(s3, space0);
SamplerComparisonState smp_depth_linear : register(s2, space0);
Texture2D<float4> ssao_texture : register(t0, space1);

static float4 gl_FragCoord;
static float4 in_position_ws;
static float3 in_position_vs;
static float3 in_normal_vs;
static float3x3 in_model_view;
static uint in_instance_index;
static float2 in_uv;
static float3 in_tangent_vs;
static float3 in_binormal_vs;
static float4 out_color;

struct SPIRV_Cross_Input
{
    float3 in_position_vs : TEXCOORD0;
    float3 in_normal_vs : TEXCOORD1;
    float3 in_tangent_vs : TEXCOORD2;
    float3 in_binormal_vs : TEXCOORD3;
    float2 in_uv : TEXCOORD4;
    float4 in_position_ws : TEXCOORD5;
    float3x3 in_model_view : TEXCOORD6;
    nointerpolation uint in_instance_index : TEXCOORD9;
    float4 gl_FragCoord : SV_Position;
};

struct SPIRV_Cross_Output
{
    float4 out_color : SV_Target0;
};

uint2 spvTextureSize(Texture2D<float4> Tex, uint Level, out uint Param)
{
    uint2 ret;
    Tex.GetDimensions(Level, ret.x, ret.y, Param);
    return ret;
}

uint get_light_cluster_index()
{
    float NEAR_Z = 5.0f;
    float FAR_Z = 10000.0f;
    int X_BINS = 16;
    int Y_BINS = 8;
    int Z_BINS = 24;
    uint cluster_coord_x = min(uint((gl_FragCoord.x / float(per_view_data_viewport_width)) * float(X_BINS)), uint(X_BINS - 1));
    uint cluster_coord_y = min(uint((1.0f - (gl_FragCoord.y / float(per_view_data_viewport_height))) * float(Y_BINS)), uint(Y_BINS - 1));
    float top = float(Z_BINS - 1) * log((-in_position_vs.z) / NEAR_Z);
    float bottom = log(FAR_Z / NEAR_Z);
    uint cluster_coord_z = uint(clamp((top / bottom) + 1.0f, 0.0f, float(Z_BINS - 1)));
    uint linear_index = ((uint(X_BINS * Y_BINS) * cluster_coord_z) + (uint(X_BINS) * cluster_coord_y)) + cluster_coord_x;
    return linear_index;
}

float4 normal_map(int normal_texture, float3x3 tangent_binormal_normal, float2 uv)
{
    float3 normal = all_material_textures[normal_texture].SampleBias(smp, uv, per_view_data_mip_bias).xyz;
    normal = (normal * 2.0f) - 1.0f.xxx;
    normal.z = 0.0f;
    normal.z = sqrt(1.0f - dot(normal, normal));
    normal.x = -normal.x;
    normal.y = -normal.y;
    normal = mul(normal, tangent_binormal_normal);
    return normalize(float4(normal, 0.0f));
}

float DeferredLightingNDFRoughnessFilter(float3 normal, float roughness2, float ndf_filter_amount)
{
    float SIGMA2 = 0.15915493667125701904296875f;
    float KAPPA = 0.180000007152557373046875f;
    float3 dndu = ddx(normal);
    float3 dndv = ddy(normal);
    float kernelRoughness2 = (2.0f * SIGMA2) * (dot(dndu, dndu) + dot(dndv, dndv));
    float clampedKernelRoughness2 = min(kernelRoughness2, KAPPA);
    return clamp(roughness2 + (clampedKernelRoughness2 * ndf_filter_amount), 0.0f, 1.0f);
}

float attenuate_light_for_range(float light_range, float _distance)
{
    return 1.0f - smoothstep(light_range * 0.75f, light_range, _distance);
}

float spotlight_cone_falloff(float3 surface_to_light_dir, float3 spotlight_dir, float spotlight_half_angle)
{
    float cos_angle = dot(-spotlight_dir, surface_to_light_dir);
    float min_cos = cos(spotlight_half_angle);
    float max_cos = lerp(min_cos, 1.0f, 0.5f);
    return smoothstep(min_cos, max_cos, cos_angle);
}

float ndf_ggx(float3 n, float3 h, float roughness_squared)
{
    float a = roughness_squared;
    float a2 = a * a;
    float n_dot_h = max(dot(n, h), 0.0f);
    float bottom_part = ((n_dot_h * n_dot_h) * (a2 - 1.0f)) + 1.0f;
    float bottom = (3.1415927410125732421875f * bottom_part) * bottom_part;
    return a2 / bottom;
}

float geometric_attenuation_schlick_ggx(float dot_product, float k)
{
    float bottom = (dot_product * (1.0f - k)) + k;
    return dot_product / bottom;
}

float geometric_attenuation_smith(float3 n, float3 v, float3 l, float roughness)
{
    float r_plus_1 = roughness + 1.0f;
    float k = (r_plus_1 * r_plus_1) / 8.0f;
    float param = max(dot(n, v), 0.0f);
    float param_1 = k;
    float v_factor = geometric_attenuation_schlick_ggx(param, param_1);
    float param_2 = max(dot(n, l), 0.0f);
    float param_3 = k;
    float l_factor = geometric_attenuation_schlick_ggx(param_2, param_3);
    return v_factor * l_factor;
}

float3 fresnel_schlick(float3 v, float3 h, float3 fresnel_base)
{
    float v_dot_h = max(dot(v, h), 0.0f);
    return fresnel_base + ((1.0f.xxx - fresnel_base) * exp2((((-5.554729938507080078125f) * v_dot_h) - 6.9831600189208984375f) * v_dot_h));
}

float3 shade_pbr(float3 surface_to_light_dir_vs, float3 surface_to_eye_dir_vs, float3 normal_vs, float3 F0, float3 base_color, float roughness, float roughness_ndf_filtered_squared, float metalness, float3 radiance)
{
    float3 halfway_dir_vs = normalize(surface_to_light_dir_vs + surface_to_eye_dir_vs);
    float3 param = normal_vs;
    float3 param_1 = halfway_dir_vs;
    float param_2 = roughness_ndf_filtered_squared;
    float NDF = ndf_ggx(param, param_1, param_2);
    float3 param_3 = normal_vs;
    float3 param_4 = surface_to_eye_dir_vs;
    float3 param_5 = surface_to_light_dir_vs;
    float param_6 = roughness;
    float G = geometric_attenuation_smith(param_3, param_4, param_5, param_6);
    float3 param_7 = surface_to_eye_dir_vs;
    float3 param_8 = halfway_dir_vs;
    float3 param_9 = F0;
    float3 F = fresnel_schlick(param_7, param_8, param_9);
    float3 fresnel_specular = F;
    float3 fresnel_diffuse = 1.0f.xxx - fresnel_specular;
    fresnel_diffuse *= (1.0f - metalness);
    float n_dot_l = max(dot(normal_vs, surface_to_light_dir_vs), 0.0f);
    float n_dot_v = max(dot(normal_vs, surface_to_eye_dir_vs), 0.0f);
    float3 top = F * (NDF * G);
    float bottom = (4.0f * n_dot_v) * n_dot_l;
    float3 specular = top / max(bottom, 0.001000000047497451305389404296875f).xxx;
    return ((((fresnel_diffuse * base_color) / 3.1415927410125732421875f.xxx) + specular) * radiance) * n_dot_l;
}

float3 spot_light_pbr(float3 light_position_vs, float3 light_color, float light_intensity, float3 light_direction_vs, float light_spotlight_half_angle, float3 surface_to_eye_dir_vs, float3 surface_position_vs, float3 normal_vs, float3 F0, float3 base_color, float roughness, float roughness_ndf_filtered_squared, float metalness)
{
    float3 surface_to_light_dir_vs = light_position_vs - surface_position_vs;
    float _distance = length(surface_to_light_dir_vs);
    surface_to_light_dir_vs /= _distance.xxx;
    float attenuation = 1.0f / (0.001000000047497451305389404296875f + (_distance * _distance));
    float3 param = surface_to_light_dir_vs;
    float3 param_1 = light_direction_vs;
    float param_2 = light_spotlight_half_angle;
    float spotlight_direction_intensity = spotlight_cone_falloff(param, param_1, param_2);
    float radiance = (attenuation * light_intensity) * spotlight_direction_intensity;
    if (radiance > 0.0f)
    {
        float3 param_3 = surface_to_light_dir_vs;
        float3 param_4 = surface_to_eye_dir_vs;
        float3 param_5 = normal_vs;
        float3 param_6 = F0;
        float3 param_7 = base_color;
        float param_8 = roughness;
        float param_9 = roughness_ndf_filtered_squared;
        float param_10 = metalness;
        float3 param_11 = light_color * radiance;
        return shade_pbr(param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10, param_11);
    }
    else
    {
        return 0.0f.xxx;
    }
}

float do_calculate_percent_lit(float3 normal_vs, int index, float bias_multiplier)
{
    float4 shadow_map_pos = mul(in_position_ws, per_view_data_shadow_map_2d_data[index].shadow_map_view_proj);
    float3 projected = shadow_map_pos.xyz / shadow_map_pos.w.xxx;
    float2 sample_location_uv = (projected.xy * 0.5f) + 0.5f.xx;
    sample_location_uv.y = 1.0f - sample_location_uv.y;
    float2 uv_min = per_view_data_shadow_map_2d_data[index].uv_min;
    float2 uv_max = per_view_data_shadow_map_2d_data[index].uv_max;
    sample_location_uv = lerp(uv_min, uv_max, sample_location_uv);
    float depth_of_surface = projected.z;
    float3 light_dir_vs = mul(per_view_data_shadow_map_2d_data[index].shadow_map_light_dir, in_model_view);
    float3 surface_to_light_dir_vs = -light_dir_vs;
    float bias_angle_factor = 1.0f - dot(normal_vs, surface_to_light_dir_vs);
    float bias = max(((0.00999999977648258209228515625f * bias_angle_factor) * bias_angle_factor) * bias_angle_factor, 0.0005000000237487256526947021484375f) * bias_multiplier;
    float4 uv_min_max_compare = float4(uv_min, -uv_max);
    float percent_lit = 0.0f;
    uint _629_dummy_parameter;
    float2 texelSize = float2(int2(1, 1) / int2(spvTextureSize(shadow_map_atlas, uint(0), _629_dummy_parameter)));
    int x = -1;
    for (;;)
    {
        if (x <= 1)
        {
            int y = -1;
            for (;;)
            {
                if (y <= 1)
                {
                    float4 uv = float4(sample_location_uv + (float2(float(x), float(y)) * texelSize), 0.0f, 0.0f);
                    float4 _662 = uv;
                    float2 _664 = -_662.xy;
                    uv.z = _664.x;
                    uv.w = _664.y;
                    if (all(bool4(uv.x >= uv_min_max_compare.x, uv.y >= uv_min_max_compare.y, uv.z >= uv_min_max_compare.z, uv.w >= uv_min_max_compare.w)))
                    {
                        float3 _686 = float3(uv.xy, depth_of_surface + bias);
                        percent_lit += shadow_map_atlas.SampleCmp(smp_depth_linear, _686.xy, _686.z);
                    }
                    else
                    {
                        percent_lit += 1.0f;
                    }
                    y++;
                    continue;
                }
                else
                {
                    break;
                }
            }
            x++;
            continue;
        }
        else
        {
            break;
        }
    }
    percent_lit /= 9.0f;
    return percent_lit;
}

float calculate_percent_lit(float3 normal, int index, float bias_multiplier)
{
    if (index == (-1))
    {
        return 1.0f;
    }
    float3 param = normal;
    int param_1 = index;
    float param_2 = bias_multiplier;
    return do_calculate_percent_lit(param, param_1, param_2);
}

float3 point_light_pbr(float3 light_position_vs, float3 light_color, float light_intensity, float3 surface_to_eye_dir_vs, float3 surface_position_vs, float3 normal_vs, float3 F0, float3 base_color, float roughness, float roughness_ndf_filtered_squared, float metalness)
{
    float3 surface_to_light_dir_vs = light_position_vs - surface_position_vs;
    float _distance = length(surface_to_light_dir_vs);
    surface_to_light_dir_vs /= _distance.xxx;
    float attenuation = 1.0f / (0.001000000047497451305389404296875f + (_distance * _distance));
    float3 radiance = (light_color * attenuation) * light_intensity;
    float3 param = surface_to_light_dir_vs;
    float3 param_1 = surface_to_eye_dir_vs;
    float3 param_2 = normal_vs;
    float3 param_3 = F0;
    float3 param_4 = base_color;
    float param_5 = roughness;
    float param_6 = roughness_ndf_filtered_squared;
    float param_7 = metalness;
    float3 param_8 = radiance;
    return shade_pbr(param, param_1, param_2, param_3, param_4, param_5, param_6, param_7, param_8);
}

float calculate_cubemap_equivalent_depth(float3 light_to_surface_ws, float near, float far)
{
    float3 light_to_surface_ws_abs = abs(light_to_surface_ws);
    float face_local_z_depth = max(light_to_surface_ws_abs.x, max(light_to_surface_ws_abs.y, light_to_surface_ws_abs.z));
    float depth_value = ((far + near) / (far - near)) - ((((2.0f * far) * near) / (far - near)) / face_local_z_depth);
    return (depth_value + 1.0f) * 0.5f;
}

float3 cube_sample_to_uv_and_face_index(float3 dir)
{
    float3 dirAbs = abs(dir);
    bool _318 = dirAbs.z >= dirAbs.x;
    bool _326;
    if (_318)
    {
        _326 = dirAbs.z >= dirAbs.y;
    }
    else
    {
        _326 = _318;
    }
    float faceIndex;
    float ma;
    float2 uv;
    if (_326)
    {
        faceIndex = (dir.z < 0.0f) ? 5.0f : 4.0f;
        ma = 0.5f / dirAbs.z;
        float _344;
        if (dir.z < 0.0f)
        {
            _344 = -dir.x;
        }
        else
        {
            _344 = dir.x;
        }
        uv = float2(_344, -dir.y);
    }
    else
    {
        if (dirAbs.y >= dirAbs.x)
        {
            faceIndex = (dir.y < 0.0f) ? 3.0f : 2.0f;
            ma = 0.5f / dirAbs.y;
            float _379;
            if (dir.y < 0.0f)
            {
                _379 = -dir.z;
            }
            else
            {
                _379 = dir.z;
            }
            uv = float2(dir.x, _379);
        }
        else
        {
            faceIndex = float(dir.x < 0.0f);
            ma = 0.5f / dirAbs.x;
            float _401;
            if (dir.x < 0.0f)
            {
                _401 = dir.z;
            }
            else
            {
                _401 = -dir.z;
            }
            uv = float2(_401, -dir.y);
        }
    }
    return float3((uv * ma) + 0.5f.xx, faceIndex);
}

float do_calculate_percent_lit_cube(float3 light_position_ws, float3 light_position_vs, float3 normal_vs, int index, float bias_multiplier)
{
    float near_plane = per_view_data_shadow_map_cube_data[index].cube_map_projection_near_z;
    float far_plane = per_view_data_shadow_map_cube_data[index].cube_map_projection_far_z;
    float3 light_to_surface_ws = in_position_ws.xyz - light_position_ws;
    float3 surface_to_light_dir_vs = normalize(light_position_vs - in_position_vs);
    float bias_angle_factor = 1.0f - max(0.0f, dot(in_normal_vs, surface_to_light_dir_vs));
    bias_angle_factor = pow(bias_angle_factor, 3.0f);
    float bias = 0.000600000028498470783233642578125f + (0.006000000052154064178466796875f * bias_angle_factor);
    float3 param = light_to_surface_ws;
    float param_1 = near_plane;
    float param_2 = far_plane;
    float depth_of_surface = calculate_cubemap_equivalent_depth(param, param_1, param_2);
    float3 param_3 = light_to_surface_ws;
    float3 uv_and_face = cube_sample_to_uv_and_face_index(param_3);
    float4 uv_min_uv_max = per_view_data_shadow_map_cube_data[index].uv_min_uv_max[int(uv_and_face.z)];
    if (uv_min_uv_max.x < 0.0f)
    {
        return 1.0f;
    }
    float2 uv_to_sample = lerp(uv_min_uv_max.xy, uv_min_uv_max.zw, uv_and_face.xy);
    float3 _515 = float3(uv_to_sample, depth_of_surface + bias);
    float shadow = shadow_map_atlas.SampleCmp(smp_depth_nearest, _515.xy, _515.z);
    return shadow;
}

float calculate_percent_lit_cube(float3 light_position_ws, float3 light_position_vs, float3 normal_vs, int index, float bias_multiplier)
{
    if (index == (-1))
    {
        return 1.0f;
    }
    float3 param = light_position_ws;
    float3 param_1 = light_position_vs;
    float3 param_2 = normal_vs;
    int param_3 = index;
    float param_4 = bias_multiplier;
    return do_calculate_percent_lit_cube(param, param_1, param_2, param_3, param_4);
}

float3 iterate_point_and_spot_lights_clustered(float3 surface_to_eye_vs, float4 base_color, float metalness, float roughness, float3 normal_vs, float3 fresnel_base, float roughness_ndf_filtered_squared, uint light_cluster_index)
{
    float3 total_light = 0.0f.xxx;
    uint light_first = light_bin_output.Load(light_cluster_index * 8 + 20);
    uint light_last = light_first + light_bin_output.Load(light_cluster_index * 8 + 16);
    LightInList light;
    uint light_list_index = light_first;
    for (;;)
    {
        if (light_list_index < light_last)
        {
            uint light_index = light_bin_output.Load(light_list_index * 4 + 24592);
            LightInList _1340;
            _1340.position_ws = asfloat(all_lights.Load3(light_index * 80 + 16));
            _1340.range = asfloat(all_lights.Load(light_index * 80 + 28));
            _1340.position_vs = asfloat(all_lights.Load3(light_index * 80 + 32));
            _1340.intensity = asfloat(all_lights.Load(light_index * 80 + 44));
            _1340.color = asfloat(all_lights.Load4(light_index * 80 + 48));
            _1340.spotlight_direction_ws = asfloat(all_lights.Load3(light_index * 80 + 64));
            _1340.spotlight_half_angle = asfloat(all_lights.Load(light_index * 80 + 76));
            _1340.spotlight_direction_vs = asfloat(all_lights.Load3(light_index * 80 + 80));
            _1340.shadow_map = int(all_lights.Load(light_index * 80 + 92));
            light.position_ws = _1340.position_ws;
            light.range = _1340.range;
            light.position_vs = _1340.position_vs;
            light.intensity = _1340.intensity;
            light.color = _1340.color;
            light.spotlight_direction_ws = _1340.spotlight_direction_ws;
            light.spotlight_half_angle = _1340.spotlight_half_angle;
            light.spotlight_direction_vs = _1340.spotlight_direction_vs;
            light.shadow_map = _1340.shadow_map;
            if (dot(light.spotlight_direction_vs, light.spotlight_direction_vs) > 0.00999999977648258209228515625f)
            {
                float light_surface_distance = distance(light.position_ws, in_position_ws.xyz);
                float range = light.range;
                if (light_surface_distance <= range)
                {
                    float param = range;
                    float param_1 = light_surface_distance;
                    float soft_falloff_factor = attenuate_light_for_range(param, param_1);
                    float3 param_2 = light.position_vs;
                    float3 param_3 = light.color.xyz;
                    float param_4 = light.intensity;
                    float3 param_5 = light.spotlight_direction_vs;
                    float param_6 = light.spotlight_half_angle;
                    float3 param_7 = surface_to_eye_vs;
                    float3 param_8 = in_position_vs;
                    float3 param_9 = normal_vs;
                    float3 param_10 = fresnel_base;
                    float3 param_11 = base_color.xyz;
                    float param_12 = roughness;
                    float param_13 = roughness_ndf_filtered_squared;
                    float param_14 = metalness;
                    float3 pbr = spot_light_pbr(param_2, param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10, param_11, param_12, param_13, param_14) * soft_falloff_factor;
                    float percent_lit = 1.0f;
                    if (any(bool3(pbr.x > 0.0f.xxx.x, pbr.y > 0.0f.xxx.y, pbr.z > 0.0f.xxx.z)))
                    {
                        float3 param_15 = normal_vs;
                        int param_16 = light.shadow_map;
                        float param_17 = 1.0f;
                        percent_lit = calculate_percent_lit(param_15, param_16, param_17);
                    }
                    total_light += (pbr * percent_lit);
                }
            }
            else
            {
                float light_surface_distance_1 = distance(light.position_ws, in_position_ws.xyz);
                float range_1 = light.range;
                if (light_surface_distance_1 <= range_1)
                {
                    float param_18 = range_1;
                    float param_19 = light_surface_distance_1;
                    float soft_falloff_factor_1 = attenuate_light_for_range(param_18, param_19);
                    float3 param_20 = light.position_vs;
                    float3 param_21 = light.color.xyz;
                    float param_22 = light.intensity;
                    float3 param_23 = surface_to_eye_vs;
                    float3 param_24 = in_position_vs;
                    float3 param_25 = normal_vs;
                    float3 param_26 = fresnel_base;
                    float3 param_27 = base_color.xyz;
                    float param_28 = roughness;
                    float param_29 = roughness_ndf_filtered_squared;
                    float param_30 = metalness;
                    float3 pbr_1 = point_light_pbr(param_20, param_21, param_22, param_23, param_24, param_25, param_26, param_27, param_28, param_29, param_30) * soft_falloff_factor_1;
                    float percent_lit_1 = 1.0f;
                    if (any(bool3(pbr_1.x > 0.0f.xxx.x, pbr_1.y > 0.0f.xxx.y, pbr_1.z > 0.0f.xxx.z)))
                    {
                        float3 param_31 = light.position_ws;
                        float3 param_32 = light.position_vs;
                        float3 param_33 = normal_vs;
                        int param_34 = light.shadow_map;
                        float param_35 = 1.0f;
                        percent_lit_1 = calculate_percent_lit_cube(param_31, param_32, param_33, param_34, param_35);
                    }
                    total_light += (pbr_1 * percent_lit_1);
                }
            }
            light_list_index++;
            continue;
        }
        else
        {
            break;
        }
    }
    return total_light;
}

float3 iterate_point_and_spot_lights_all(float3 surface_to_eye_vs, float4 base_color, float metalness, float roughness, float3 normal_vs, float3 fresnel_base, float roughness_ndf_filtered_squared, uint light_cluster_index)
{
    float3 total_light = 0.0f.xxx;
    LightInList light;
    uint light_index = 0u;
    for (;;)
    {
        if (light_index < all_lights.Load(0))
        {
            LightInList _1115;
            _1115.position_ws = asfloat(all_lights.Load3(light_index * 80 + 16));
            _1115.range = asfloat(all_lights.Load(light_index * 80 + 28));
            _1115.position_vs = asfloat(all_lights.Load3(light_index * 80 + 32));
            _1115.intensity = asfloat(all_lights.Load(light_index * 80 + 44));
            _1115.color = asfloat(all_lights.Load4(light_index * 80 + 48));
            _1115.spotlight_direction_ws = asfloat(all_lights.Load3(light_index * 80 + 64));
            _1115.spotlight_half_angle = asfloat(all_lights.Load(light_index * 80 + 76));
            _1115.spotlight_direction_vs = asfloat(all_lights.Load3(light_index * 80 + 80));
            _1115.shadow_map = int(all_lights.Load(light_index * 80 + 92));
            light.position_ws = _1115.position_ws;
            light.range = _1115.range;
            light.position_vs = _1115.position_vs;
            light.intensity = _1115.intensity;
            light.color = _1115.color;
            light.spotlight_direction_ws = _1115.spotlight_direction_ws;
            light.spotlight_half_angle = _1115.spotlight_half_angle;
            light.spotlight_direction_vs = _1115.spotlight_direction_vs;
            light.shadow_map = _1115.shadow_map;
            if (dot(light.spotlight_direction_vs, light.spotlight_direction_vs) > 0.00999999977648258209228515625f)
            {
                float light_surface_distance = distance(light.position_ws, in_position_ws.xyz);
                float range = light.range;
                if (light_surface_distance <= range)
                {
                    float param = range;
                    float param_1 = light_surface_distance;
                    float soft_falloff_factor = attenuate_light_for_range(param, param_1);
                    float3 param_2 = light.position_vs;
                    float3 param_3 = light.color.xyz;
                    float param_4 = light.intensity;
                    float3 param_5 = light.spotlight_direction_vs;
                    float param_6 = light.spotlight_half_angle;
                    float3 param_7 = surface_to_eye_vs;
                    float3 param_8 = in_position_vs;
                    float3 param_9 = normal_vs;
                    float3 param_10 = fresnel_base;
                    float3 param_11 = base_color.xyz;
                    float param_12 = roughness;
                    float param_13 = roughness_ndf_filtered_squared;
                    float param_14 = metalness;
                    float3 pbr = spot_light_pbr(param_2, param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10, param_11, param_12, param_13, param_14) * soft_falloff_factor;
                    float percent_lit = 1.0f;
                    if (any(bool3(pbr.x > 0.0f.xxx.x, pbr.y > 0.0f.xxx.y, pbr.z > 0.0f.xxx.z)))
                    {
                        float3 param_15 = normal_vs;
                        int param_16 = light.shadow_map;
                        float param_17 = 1.0f;
                        percent_lit = calculate_percent_lit(param_15, param_16, param_17);
                    }
                    total_light += (pbr * percent_lit);
                }
            }
            else
            {
                float light_surface_distance_1 = distance(light.position_ws, in_position_ws.xyz);
                float range_1 = light.range;
                if (light_surface_distance_1 <= range_1)
                {
                    float param_18 = range_1;
                    float param_19 = light_surface_distance_1;
                    float soft_falloff_factor_1 = attenuate_light_for_range(param_18, param_19);
                    float3 param_20 = light.position_vs;
                    float3 param_21 = light.color.xyz;
                    float param_22 = light.intensity;
                    float3 param_23 = surface_to_eye_vs;
                    float3 param_24 = in_position_vs;
                    float3 param_25 = normal_vs;
                    float3 param_26 = fresnel_base;
                    float3 param_27 = base_color.xyz;
                    float param_28 = roughness;
                    float param_29 = roughness_ndf_filtered_squared;
                    float param_30 = metalness;
                    float3 pbr_1 = point_light_pbr(param_20, param_21, param_22, param_23, param_24, param_25, param_26, param_27, param_28, param_29, param_30) * soft_falloff_factor_1;
                    float percent_lit_1 = 1.0f;
                    if (any(bool3(pbr_1.x > 0.0f.xxx.x, pbr_1.y > 0.0f.xxx.y, pbr_1.z > 0.0f.xxx.z)))
                    {
                        float3 param_31 = light.position_ws;
                        float3 param_32 = light.position_vs;
                        float3 param_33 = normal_vs;
                        int param_34 = light.shadow_map;
                        float param_35 = 1.0f;
                        percent_lit_1 = calculate_percent_lit_cube(param_31, param_32, param_33, param_34, param_35);
                    }
                    total_light += (pbr_1 * percent_lit_1);
                }
            }
            light_index++;
            continue;
        }
        else
        {
            break;
        }
    }
    return total_light;
}

float3 directional_light_pbr(DirectionalLight light, float3 surface_to_eye_dir_vs, float3 surface_position_vs, float3 normal_vs, float3 F0, float3 base_color, float roughness, float roughness_ndf_filtered_squared, float metalness)
{
    float3 surface_to_light_dir_vs = -light.direction_vs;
    float3 radiance = light.color.xyz * light.intensity;
    float3 param = surface_to_light_dir_vs;
    float3 param_1 = surface_to_eye_dir_vs;
    float3 param_2 = normal_vs;
    float3 param_3 = F0;
    float3 param_4 = base_color;
    float param_5 = roughness;
    float param_6 = roughness_ndf_filtered_squared;
    float param_7 = metalness;
    float3 param_8 = radiance;
    return shade_pbr(param, param_1, param_2, param_3, param_4, param_5, param_6, param_7, param_8);
}

float4 pbr_path(float3 surface_to_eye_vs, float4 base_color, float4 emissive_color, float metalness, float roughness, float3 normal_vs, uint light_cluster_index, float ambient_factor)
{
    float3 fresnel_base = 0.039999999105930328369140625f.xxx;
    fresnel_base = lerp(fresnel_base, base_color.xyz, metalness.xxx);
    float3 param = normal_vs;
    float param_1 = roughness * roughness;
    float param_2 = per_view_data_ndf_filter_amount;
    float roughness_ndf_filtered_squared = DeferredLightingNDFRoughnessFilter(param, param_1, param_2);
    float3 total_light = 0.0f.xxx;
    if (per_view_data_use_clustered_lighting != 0u)
    {
        float3 param_3 = surface_to_eye_vs;
        float4 param_4 = base_color;
        float param_5 = metalness;
        float param_6 = roughness;
        float3 param_7 = normal_vs;
        float3 param_8 = fresnel_base;
        float param_9 = roughness_ndf_filtered_squared;
        uint param_10 = light_cluster_index;
        total_light = iterate_point_and_spot_lights_clustered(param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10);
    }
    else
    {
        float3 param_11 = surface_to_eye_vs;
        float4 param_12 = base_color;
        float param_13 = metalness;
        float param_14 = roughness;
        float3 param_15 = normal_vs;
        float3 param_16 = fresnel_base;
        float param_17 = roughness_ndf_filtered_squared;
        uint param_18 = light_cluster_index;
        total_light = iterate_point_and_spot_lights_all(param_11, param_12, param_13, param_14, param_15, param_16, param_17, param_18);
    }
    DirectionalLight param_19;
    uint i = 0u;
    for (;;)
    {
        if (i < per_view_data_directional_light_count)
        {
            param_19.direction_ws = per_view_data_directional_lights[i].direction_ws;
            param_19.intensity = per_view_data_directional_lights[i].intensity;
            param_19.color = per_view_data_directional_lights[i].color;
            param_19.direction_vs = per_view_data_directional_lights[i].direction_vs;
            param_19.shadow_map = per_view_data_directional_lights[i].shadow_map;
            float3 param_20 = surface_to_eye_vs;
            float3 param_21 = in_position_vs;
            float3 param_22 = normal_vs;
            float3 param_23 = fresnel_base;
            float3 param_24 = base_color.xyz;
            float param_25 = roughness;
            float param_26 = roughness_ndf_filtered_squared;
            float param_27 = metalness;
            float3 pbr = directional_light_pbr(param_19, param_20, param_21, param_22, param_23, param_24, param_25, param_26, param_27);
            float percent_lit = 1.0f;
            if (any(bool3(pbr.x > 0.0f.xxx.x, pbr.y > 0.0f.xxx.y, pbr.z > 0.0f.xxx.z)))
            {
                float3 param_28 = normal_vs;
                int param_29 = per_view_data_directional_lights[i].shadow_map;
                float param_30 = 1.0f;
                percent_lit = calculate_percent_lit(param_28, param_29, param_30);
            }
            total_light += (pbr * percent_lit);
            i++;
            continue;
        }
        else
        {
            break;
        }
    }
    float3 ambient = (per_view_data_ambient_light.xyz * base_color.xyz) * ambient_factor;
    uint material_index = all_draw_data.Load(in_instance_index * 8 + 20);
    MaterialDbEntry_1 _1683;
    _1683.base_color_factor = asfloat(all_materials.Load4(material_index * 80 + 0));
    _1683.emissive_factor = asfloat(all_materials.Load3(material_index * 80 + 16));
    _1683.metallic_factor = asfloat(all_materials.Load(material_index * 80 + 28));
    _1683.roughness_factor = asfloat(all_materials.Load(material_index * 80 + 32));
    _1683.normal_texture_scale = asfloat(all_materials.Load(material_index * 80 + 36));
    _1683.alpha_threshold = asfloat(all_materials.Load(material_index * 80 + 40));
    _1683.enable_alpha_blend = all_materials.Load(material_index * 80 + 44);
    _1683.enable_alpha_clip = all_materials.Load(material_index * 80 + 48);
    _1683.color_texture = int(all_materials.Load(material_index * 80 + 52));
    _1683.base_color_texture_has_alpha_channel = all_materials.Load(material_index * 80 + 56);
    _1683.metallic_roughness_texture = int(all_materials.Load(material_index * 80 + 60));
    _1683.normal_texture = int(all_materials.Load(material_index * 80 + 64));
    _1683.emissive_texture = int(all_materials.Load(material_index * 80 + 68));
    MaterialDbEntry per_material_data;
    per_material_data.base_color_factor = _1683.base_color_factor;
    per_material_data.emissive_factor = _1683.emissive_factor;
    per_material_data.metallic_factor = _1683.metallic_factor;
    per_material_data.roughness_factor = _1683.roughness_factor;
    per_material_data.normal_texture_scale = _1683.normal_texture_scale;
    per_material_data.alpha_threshold = _1683.alpha_threshold;
    per_material_data.enable_alpha_blend = _1683.enable_alpha_blend != 0u;
    per_material_data.enable_alpha_clip = _1683.enable_alpha_clip != 0u;
    per_material_data.color_texture = _1683.color_texture;
    per_material_data.base_color_texture_has_alpha_channel = _1683.base_color_texture_has_alpha_channel != 0u;
    per_material_data.metallic_roughness_texture = _1683.metallic_roughness_texture;
    per_material_data.normal_texture = _1683.normal_texture;
    per_material_data.emissive_texture = _1683.emissive_texture;
    float alpha = 1.0f;
    if (per_material_data.enable_alpha_blend)
    {
        alpha = base_color.w;
    }
    else
    {
        bool _1731;
        if (per_material_data.enable_alpha_clip)
        {
            _1731 = base_color.w < per_material_data.alpha_threshold;
        }
        else
        {
            _1731 = per_material_data.enable_alpha_clip;
        }
        if (_1731)
        {
            alpha = 0.0f;
        }
    }
    float3 color = (ambient + total_light) + emissive_color.xyz;
    return float4(color, alpha);
}

float4 pbr_main()
{
    uint material_index = all_draw_data.Load(in_instance_index * 8 + 20);
    MaterialDbEntry_1 _1840;
    _1840.base_color_factor = asfloat(all_materials.Load4(material_index * 80 + 0));
    _1840.emissive_factor = asfloat(all_materials.Load3(material_index * 80 + 16));
    _1840.metallic_factor = asfloat(all_materials.Load(material_index * 80 + 28));
    _1840.roughness_factor = asfloat(all_materials.Load(material_index * 80 + 32));
    _1840.normal_texture_scale = asfloat(all_materials.Load(material_index * 80 + 36));
    _1840.alpha_threshold = asfloat(all_materials.Load(material_index * 80 + 40));
    _1840.enable_alpha_blend = all_materials.Load(material_index * 80 + 44);
    _1840.enable_alpha_clip = all_materials.Load(material_index * 80 + 48);
    _1840.color_texture = int(all_materials.Load(material_index * 80 + 52));
    _1840.base_color_texture_has_alpha_channel = all_materials.Load(material_index * 80 + 56);
    _1840.metallic_roughness_texture = int(all_materials.Load(material_index * 80 + 60));
    _1840.normal_texture = int(all_materials.Load(material_index * 80 + 64));
    _1840.emissive_texture = int(all_materials.Load(material_index * 80 + 68));
    MaterialDbEntry per_material_data;
    per_material_data.base_color_factor = _1840.base_color_factor;
    per_material_data.emissive_factor = _1840.emissive_factor;
    per_material_data.metallic_factor = _1840.metallic_factor;
    per_material_data.roughness_factor = _1840.roughness_factor;
    per_material_data.normal_texture_scale = _1840.normal_texture_scale;
    per_material_data.alpha_threshold = _1840.alpha_threshold;
    per_material_data.enable_alpha_blend = _1840.enable_alpha_blend != 0u;
    per_material_data.enable_alpha_clip = _1840.enable_alpha_clip != 0u;
    per_material_data.color_texture = _1840.color_texture;
    per_material_data.base_color_texture_has_alpha_channel = _1840.base_color_texture_has_alpha_channel != 0u;
    per_material_data.metallic_roughness_texture = _1840.metallic_roughness_texture;
    per_material_data.normal_texture = _1840.normal_texture;
    per_material_data.emissive_texture = _1840.emissive_texture;
    float4 base_color = per_material_data.base_color_factor;
    float ambient_factor = 1.0f;
    uint light_cluster_index = get_light_cluster_index();
    if (per_material_data.color_texture != (-1))
    {
        float4 sampled_color = all_material_textures[per_material_data.color_texture].SampleBias(smp, in_uv, per_view_data_mip_bias);
        if (per_material_data.base_color_texture_has_alpha_channel)
        {
            base_color *= sampled_color;
        }
        else
        {
            base_color = float4(base_color.xyz * sampled_color.xyz, base_color.w);
        }
    }
    float screen_coord_x = gl_FragCoord.x / float(per_view_data_viewport_width);
    float screen_coord_y = gl_FragCoord.y / float(per_view_data_viewport_height);
    ambient_factor = ssao_texture.Sample(smp, float2(screen_coord_x, screen_coord_y)).x;
    float4 emissive_color = float4(per_material_data.emissive_factor, 1.0f);
    if (per_material_data.emissive_texture != (-1))
    {
        emissive_color *= all_material_textures[per_material_data.emissive_texture].SampleBias(smp, in_uv, per_view_data_mip_bias);
    }
    float metalness = per_material_data.metallic_factor;
    float roughness = per_material_data.roughness_factor;
    if (per_material_data.metallic_roughness_texture != (-1))
    {
        float4 sampled = all_material_textures[per_material_data.metallic_roughness_texture].SampleBias(smp, in_uv, per_view_data_mip_bias);
        metalness *= sampled.z;
        roughness *= sampled.y;
    }
    metalness = clamp(metalness, 0.0f, 1.0f);
    roughness = clamp(roughness, 0.0f, 1.0f);
    float3 normal_vs;
    if (per_material_data.normal_texture != (-1))
    {
        float3x3 tbn = float3x3(float3(in_tangent_vs), float3(in_binormal_vs), float3(in_normal_vs));
        int param = per_material_data.normal_texture;
        float3x3 param_1 = tbn;
        float2 param_2 = in_uv;
        normal_vs = normal_map(param, param_1, param_2).xyz;
    }
    else
    {
        normal_vs = normalize(float4(in_normal_vs, 0.0f)).xyz;
    }
    float3 eye_position_vs = 0.0f.xxx;
    float3 surface_to_eye_vs = normalize(eye_position_vs - in_position_vs);
    float3 param_3 = surface_to_eye_vs;
    float4 param_4 = base_color;
    float4 param_5 = emissive_color;
    float param_6 = metalness;
    float param_7 = roughness;
    float3 param_8 = normal_vs;
    uint param_9 = light_cluster_index;
    float param_10 = ambient_factor;
    float4 out_color_1 = pbr_path(param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10);
    return out_color_1;
}

void frag_main()
{
    out_color = pbr_main();
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    gl_FragCoord = stage_input.gl_FragCoord;
    gl_FragCoord.w = 1.0 / gl_FragCoord.w;
    in_position_ws = stage_input.in_position_ws;
    in_position_vs = stage_input.in_position_vs;
    in_normal_vs = stage_input.in_normal_vs;
    in_model_view = stage_input.in_model_view;
    in_instance_index = stage_input.in_instance_index;
    in_uv = stage_input.in_uv;
    in_tangent_vs = stage_input.in_tangent_vs;
    in_binormal_vs = stage_input.in_binormal_vs;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_color = out_color;
    return stage_output;
}
    �      #pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct DirectionalLight
{
    float3 direction_ws;
    float intensity;
    float4 color;
    float3 direction_vs;
    int shadow_map;
};

struct DirectionalLight_1
{
    packed_float3 direction_ws;
    float intensity;
    float4 color;
    packed_float3 direction_vs;
    int shadow_map;
};

struct ShadowMap2DData
{
    float2 uv_min;
    float2 uv_max;
    float4x4 shadow_map_view_proj;
    float3 shadow_map_light_dir;
};

struct ShadowMapCubeData
{
    float4 uv_min_uv_max[6];
    float cube_map_projection_near_z;
    float cube_map_projection_far_z;
    char _m0_final_padding[8];
};

struct PerViewData
{
    float4x4 view;
    float4x4 view_proj;
    float4 ambient_light;
    float2 jitter_amount;
    uint viewport_width;
    uint viewport_height;
    float mip_bias;
    float ndf_filter_amount;
    uint directional_light_count;
    uint use_clustered_lighting;
    DirectionalLight_1 directional_lights[8];
    ShadowMap2DData shadow_map_2d_data[96];
    ShadowMapCubeData shadow_map_cube_data[32];
};

struct LightInList
{
    packed_float3 position_ws;
    float range;
    packed_float3 position_vs;
    float intensity;
    float4 color;
    packed_float3 spotlight_direction_ws;
    float spotlight_half_angle;
    packed_float3 spotlight_direction_vs;
    int shadow_map;
};

struct AllLights
{
    uint light_count;
    LightInList data[512];
};

struct LightInList_1
{
    float3 position_ws;
    float range;
    float3 position_vs;
    float intensity;
    float4 color;
    float3 spotlight_direction_ws;
    float spotlight_half_angle;
    float3 spotlight_direction_vs;
    int shadow_map;
};

struct ClusterMeta
{
    uint count;
    uint first_light;
};

struct LightBinningOutput
{
    uint data_write_ptr;
    uint pad0;
    uint pad1;
    uint pad2;
    ClusterMeta offsets[3072];
    uint data[1572864];
};

struct LightBinOutput
{
    LightBinningOutput data;
};

struct DrawData
{
    uint transform_index;
    uint material_index;
};

struct AllDrawData
{
    uint count;
    uint pad0;
    uint pad1;
    uint pad2;
    DrawData draw_data[1];
};

struct MaterialDbEntry
{
    float4 base_color_factor;
    float3 emissive_factor;
    float metallic_factor;
    float roughness_factor;
    float normal_texture_scale;
    float alpha_threshold;
    bool enable_alpha_blend;
    bool enable_alpha_clip;
    int color_texture;
    bool base_color_texture_has_alpha_channel;
    int metallic_roughness_texture;
    int normal_texture;
    int emissive_texture;
};

struct MaterialDbEntry_1
{
    float4 base_color_factor;
    packed_float3 emissive_factor;
    float metallic_factor;
    float roughness_factor;
    float normal_texture_scale;
    float alpha_threshold;
    uint enable_alpha_blend;
    uint enable_alpha_clip;
    int color_texture;
    uint base_color_texture_has_alpha_channel;
    int metallic_roughness_texture;
    int normal_texture;
    int emissive_texture;
    char _m0_final_padding[8];
};

struct AllMaterials
{
    MaterialDbEntry_1 materials[1];
};

struct Transform
{
    float4x4 model_matrix;
};

struct AllTransforms
{
    Transform transforms[1];
};

struct spvDescriptorSetBuffer0
{
    constant PerViewData* per_view_data [[id(0)]];
    depth2d<float> shadow_map_atlas [[id(4)]];
    device LightBinOutput* light_bin_output [[id(5)]];
    device AllLights* all_lights [[id(6)]];
};

struct spvDescriptorSetBuffer1
{
    texture2d<float> ssao_texture [[id(0)]];
};

struct spvDescriptorSetBuffer2
{
    device AllTransforms* all_transforms [[id(0)]];
    device AllDrawData* all_draw_data [[id(1)]];
};

struct spvDescriptorSetBuffer3
{
    device AllMaterials* all_materials [[id(0)]];
    array<texture2d<float>, 768> all_material_textures [[id(1)]];
};

struct main0_out
{
    float4 out_color [[color(0)]];
};

struct main0_in
{
    float3 in_position_vs [[user(locn0)]];
    float3 in_normal_vs [[user(locn1)]];
    float3 in_tangent_vs [[user(locn2)]];
    float3 in_binormal_vs [[user(locn3)]];
    float2 in_uv [[user(locn4)]];
    float4 in_position_ws [[user(locn5)]];
    float3 in_model_view_0 [[user(locn6)]];
    float3 in_model_view_1 [[user(locn7)]];
    float3 in_model_view_2 [[user(locn8)]];
    uint in_instance_index [[user(locn9)]];
};

static inline __attribute__((always_inline))
uint get_light_cluster_index(constant PerViewData& per_view_data, thread float3& in_position_vs, thread float4& gl_FragCoord)
{
    float NEAR_Z = 5.0;
    float FAR_Z = 10000.0;
    int X_BINS = 16;
    int Y_BINS = 8;
    int Z_BINS = 24;
    uint cluster_coord_x = min(uint((gl_FragCoord.x / float(per_view_data.viewport_width)) * float(X_BINS)), uint(X_BINS - 1));
    uint cluster_coord_y = min(uint((1.0 - (gl_FragCoord.y / float(per_view_data.viewport_height))) * float(Y_BINS)), uint(Y_BINS - 1));
    float top = float(Z_BINS - 1) * log((-in_position_vs.z) / NEAR_Z);
    float bottom = log(FAR_Z / NEAR_Z);
    uint cluster_coord_z = uint(fast::clamp((top / bottom) + 1.0, 0.0, float(Z_BINS - 1)));
    uint linear_index = ((uint(X_BINS * Y_BINS) * cluster_coord_z) + (uint(X_BINS) * cluster_coord_y)) + cluster_coord_x;
    return linear_index;
}

static inline __attribute__((always_inline))
float4 normal_map(constant spvDescriptorSetBuffer3& spvDescriptorSet3, thread const int& normal_texture, thread const float3x3& tangent_binormal_normal, thread const float2& uv, thread sampler smp, constant PerViewData& per_view_data)
{
    float3 normal = spvDescriptorSet3.all_material_textures[normal_texture].sample(smp, uv, bias(per_view_data.mip_bias)).xyz;
    normal = (normal * 2.0) - float3(1.0);
    normal.z = 0.0;
    normal.z = sqrt(1.0 - dot(normal, normal));
    normal.x = -normal.x;
    normal.y = -normal.y;
    normal = tangent_binormal_normal * normal;
    return normalize(float4(normal, 0.0));
}

static inline __attribute__((always_inline))
float DeferredLightingNDFRoughnessFilter(thread const float3& normal, thread const float& roughness2, thread const float& ndf_filter_amount)
{
    float SIGMA2 = 0.15915493667125701904296875;
    float KAPPA = 0.180000007152557373046875;
    float3 dndu = dfdx(normal);
    float3 dndv = dfdy(normal);
    float kernelRoughness2 = (2.0 * SIGMA2) * (dot(dndu, dndu) + dot(dndv, dndv));
    float clampedKernelRoughness2 = fast::min(kernelRoughness2, KAPPA);
    return fast::clamp(roughness2 + (clampedKernelRoughness2 * ndf_filter_amount), 0.0, 1.0);
}

static inline __attribute__((always_inline))
float attenuate_light_for_range(thread const float& light_range, thread const float& _distance)
{
    return 1.0 - smoothstep(light_range * 0.75, light_range, _distance);
}

static inline __attribute__((always_inline))
float spotlight_cone_falloff(thread const float3& surface_to_light_dir, thread const float3& spotlight_dir, thread const float& spotlight_half_angle)
{
    float cos_angle = dot(-spotlight_dir, surface_to_light_dir);
    float min_cos = cos(spotlight_half_angle);
    float max_cos = mix(min_cos, 1.0, 0.5);
    return smoothstep(min_cos, max_cos, cos_angle);
}

static inline __attribute__((always_inline))
float ndf_ggx(thread const float3& n, thread const float3& h, thread const float& roughness_squared)
{
    float a = roughness_squared;
    float a2 = a * a;
    float n_dot_h = fast::max(dot(n, h), 0.0);
    float bottom_part = ((n_dot_h * n_dot_h) * (a2 - 1.0)) + 1.0;
    float bottom = (3.1415927410125732421875 * bottom_part) * bottom_part;
    return a2 / bottom;
}

static inline __attribute__((always_inline))
float geometric_attenuation_schlick_ggx(thread const float& dot_product, thread const float& k)
{
    float bottom = (dot_product * (1.0 - k)) + k;
    return dot_product / bottom;
}

static inline __attribute__((always_inline))
float geometric_attenuation_smith(thread const float3& n, thread const float3& v, thread const float3& l, thread const float& roughness)
{
    float r_plus_1 = roughness + 1.0;
    float k = (r_plus_1 * r_plus_1) / 8.0;
    float param = fast::max(dot(n, v), 0.0);
    float param_1 = k;
    float v_factor = geometric_attenuation_schlick_ggx(param, param_1);
    float param_2 = fast::max(dot(n, l), 0.0);
    float param_3 = k;
    float l_factor = geometric_attenuation_schlick_ggx(param_2, param_3);
    return v_factor * l_factor;
}

static inline __attribute__((always_inline))
float3 fresnel_schlick(thread const float3& v, thread const float3& h, thread const float3& fresnel_base)
{
    float v_dot_h = fast::max(dot(v, h), 0.0);
    return fresnel_base + ((float3(1.0) - fresnel_base) * exp2((((-5.554729938507080078125) * v_dot_h) - 6.9831600189208984375) * v_dot_h));
}

static inline __attribute__((always_inline))
float3 shade_pbr(thread const float3& surface_to_light_dir_vs, thread const float3& surface_to_eye_dir_vs, thread const float3& normal_vs, thread const float3& F0, thread const float3& base_color, thread const float& roughness, thread const float& roughness_ndf_filtered_squared, thread const float& metalness, thread const float3& radiance)
{
    float3 halfway_dir_vs = normalize(surface_to_light_dir_vs + surface_to_eye_dir_vs);
    float3 param = normal_vs;
    float3 param_1 = halfway_dir_vs;
    float param_2 = roughness_ndf_filtered_squared;
    float NDF = ndf_ggx(param, param_1, param_2);
    float3 param_3 = normal_vs;
    float3 param_4 = surface_to_eye_dir_vs;
    float3 param_5 = surface_to_light_dir_vs;
    float param_6 = roughness;
    float G = geometric_attenuation_smith(param_3, param_4, param_5, param_6);
    float3 param_7 = surface_to_eye_dir_vs;
    float3 param_8 = halfway_dir_vs;
    float3 param_9 = F0;
    float3 F = fresnel_schlick(param_7, param_8, param_9);
    float3 fresnel_specular = F;
    float3 fresnel_diffuse = float3(1.0) - fresnel_specular;
    fresnel_diffuse *= (1.0 - metalness);
    float n_dot_l = fast::max(dot(normal_vs, surface_to_light_dir_vs), 0.0);
    float n_dot_v = fast::max(dot(normal_vs, surface_to_eye_dir_vs), 0.0);
    float3 top = F * (NDF * G);
    float bottom = (4.0 * n_dot_v) * n_dot_l;
    float3 specular = top / float3(fast::max(bottom, 0.001000000047497451305389404296875));
    return ((((fresnel_diffuse * base_color) / float3(3.1415927410125732421875)) + specular) * radiance) * n_dot_l;
}

static inline __attribute__((always_inline))
float3 spot_light_pbr(thread const float3& light_position_vs, thread const float3& light_color, thread const float& light_intensity, thread const float3& light_direction_vs, thread const float& light_spotlight_half_angle, thread const float3& surface_to_eye_dir_vs, thread const float3& surface_position_vs, thread const float3& normal_vs, thread const float3& F0, thread const float3& base_color, thread const float& roughness, thread const float& roughness_ndf_filtered_squared, thread const float& metalness)
{
    float3 surface_to_light_dir_vs = light_position_vs - surface_position_vs;
    float _distance = length(surface_to_light_dir_vs);
    surface_to_light_dir_vs /= float3(_distance);
    float attenuation = 1.0 / (0.001000000047497451305389404296875 + (_distance * _distance));
    float3 param = surface_to_light_dir_vs;
    float3 param_1 = light_direction_vs;
    float param_2 = light_spotlight_half_angle;
    float spotlight_direction_intensity = spotlight_cone_falloff(param, param_1, param_2);
    float radiance = (attenuation * light_intensity) * spotlight_direction_intensity;
    if (radiance > 0.0)
    {
        float3 param_3 = surface_to_light_dir_vs;
        float3 param_4 = surface_to_eye_dir_vs;
        float3 param_5 = normal_vs;
        float3 param_6 = F0;
        float3 param_7 = base_color;
        float param_8 = roughness;
        float param_9 = roughness_ndf_filtered_squared;
        float param_10 = metalness;
        float3 param_11 = light_color * radiance;
        return shade_pbr(param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10, param_11);
    }
    else
    {
        return float3(0.0);
    }
}

static inline __attribute__((always_inline))
float do_calculate_percent_lit(thread const float3& normal_vs, thread const int& index, thread const float& bias_multiplier, constant PerViewData& per_view_data, thread float4& in_position_ws, thread depth2d<float> shadow_map_atlas, thread float3x3& in_model_view, thread sampler smp_depth_linear)
{
    float4 shadow_map_pos = per_view_data.shadow_map_2d_data[index].shadow_map_view_proj * in_position_ws;
    float3 projected = shadow_map_pos.xyz / float3(shadow_map_pos.w);
    float2 sample_location_uv = (projected.xy * 0.5) + float2(0.5);
    sample_location_uv.y = 1.0 - sample_location_uv.y;
    float2 uv_min = per_view_data.shadow_map_2d_data[index].uv_min;
    float2 uv_max = per_view_data.shadow_map_2d_data[index].uv_max;
    sample_location_uv = mix(uv_min, uv_max, sample_location_uv);
    float depth_of_surface = projected.z;
    float3 light_dir_vs = in_model_view * per_view_data.shadow_map_2d_data[index].shadow_map_light_dir;
    float3 surface_to_light_dir_vs = -light_dir_vs;
    float bias_angle_factor = 1.0 - dot(normal_vs, surface_to_light_dir_vs);
    float bias0 = fast::max(((0.00999999977648258209228515625 * bias_angle_factor) * bias_angle_factor) * bias_angle_factor, 0.0005000000237487256526947021484375) * bias_multiplier;
    float4 uv_min_max_compare = float4(uv_min, -uv_max);
    float percent_lit = 0.0;
    float2 texelSize = float2(int2(1) / int2(shadow_map_atlas.get_width(), shadow_map_atlas.get_height()));
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            float4 uv = float4(sample_location_uv + (float2(float(x), float(y)) * texelSize), 0.0, 0.0);
            float2 _664 = -uv.xy;
            uv = float4(uv.x, uv.y, _664.x, _664.y);
            if (all(uv >= uv_min_max_compare))
            {
                float3 _684 = float3(uv.xy, depth_of_surface + bias0);
                percent_lit += shadow_map_atlas.sample_compare(smp_depth_linear, _684.xy, _684.z);
            }
            else
            {
                percent_lit += 1.0;
            }
        }
    }
    percent_lit /= 9.0;
    return percent_lit;
}

static inline __attribute__((always_inline))
float calculate_percent_lit(thread const float3& normal, thread const int& index, thread const float& bias_multiplier, constant PerViewData& per_view_data, thread float4& in_position_ws, thread depth2d<float> shadow_map_atlas, thread float3x3& in_model_view, thread sampler smp_depth_linear)
{
    if (index == (-1))
    {
        return 1.0;
    }
    float3 param = normal;
    int param_1 = index;
    float param_2 = bias_multiplier;
    return do_calculate_percent_lit(param, param_1, param_2, per_view_data, in_position_ws, shadow_map_atlas, in_model_view, smp_depth_linear);
}

static inline __attribute__((always_inline))
float3 point_light_pbr(thread const float3& light_position_vs, thread const float3& light_color, thread const float& light_intensity, thread const float3& surface_to_eye_dir_vs, thread const float3& surface_position_vs, thread const float3& normal_vs, thread const float3& F0, thread const float3& base_color, thread const float& roughness, thread const float& roughness_ndf_filtered_squared, thread const float& metalness)
{
    float3 surface_to_light_dir_vs = light_position_vs - surface_position_vs;
    float _distance = length(surface_to_light_dir_vs);
    surface_to_light_dir_vs /= float3(_distance);
    float attenuation = 1.0 / (0.001000000047497451305389404296875 + (_distance * _distance));
    float3 radiance = (light_color * attenuation) * light_intensity;
    float3 param = surface_to_light_dir_vs;
    float3 param_1 = surface_to_eye_dir_vs;
    float3 param_2 = normal_vs;
    float3 param_3 = F0;
    float3 param_4 = base_color;
    float param_5 = roughness;
    float param_6 = roughness_ndf_filtered_squared;
    float param_7 = metalness;
    float3 param_8 = radiance;
    return shade_pbr(param, param_1, param_2, param_3, param_4, param_5, param_6, param_7, param_8);
}

static inline __attribute__((always_inline))
float calculate_cubemap_equivalent_depth(thread const float3& light_to_surface_ws, thread const float& near, thread const float& far)
{
    float3 light_to_surface_ws_abs = abs(light_to_surface_ws);
    float face_local_z_depth = fast::max(light_to_surface_ws_abs.x, fast::max(light_to_surface_ws_abs.y, light_to_surface_ws_abs.z));
    float depth_value = ((far + near) / (far - near)) - ((((2.0 * far) * near) / (far - near)) / face_local_z_depth);
    return (depth_value + 1.0) * 0.5;
}

static inline __attribute__((always_inline))
float3 cube_sample_to_uv_and_face_index(thread const float3& dir)
{
    float3 dirAbs = abs(dir);
    bool _318 = dirAbs.z >= dirAbs.x;
    bool _326;
    if (_318)
    {
        _326 = dirAbs.z >= dirAbs.y;
    }
    else
    {
        _326 = _318;
    }
    float faceIndex;
    float ma;
    float2 uv;
    if (_326)
    {
        faceIndex = (dir.z < 0.0) ? 5.0 : 4.0;
        ma = 0.5 / dirAbs.z;
        float _344;
        if (dir.z < 0.0)
        {
            _344 = -dir.x;
        }
        else
        {
            _344 = dir.x;
        }
        uv = float2(_344, -dir.y);
    }
    else
    {
        if (dirAbs.y >= dirAbs.x)
        {
            faceIndex = (dir.y < 0.0) ? 3.0 : 2.0;
            ma = 0.5 / dirAbs.y;
            float _379;
            if (dir.y < 0.0)
            {
                _379 = -dir.z;
            }
            else
            {
                _379 = dir.z;
            }
            uv = float2(dir.x, _379);
        }
        else
        {
            faceIndex = float(dir.x < 0.0);
            ma = 0.5 / dirAbs.x;
            float _401;
            if (dir.x < 0.0)
            {
                _401 = dir.z;
            }
            else
            {
                _401 = -dir.z;
            }
            uv = float2(_401, -dir.y);
        }
    }
    return float3((uv * ma) + float2(0.5), faceIndex);
}

static inline __attribute__((always_inline))
float do_calculate_percent_lit_cube(thread const float3& light_position_ws, thread const float3& light_position_vs, thread const float3& normal_vs, thread const int& index, thread const float& bias_multiplier, constant PerViewData& per_view_data, thread float4& in_position_ws, thread float3& in_position_vs, thread float3& in_normal_vs, thread depth2d<float> shadow_map_atlas, thread sampler smp_depth_nearest)
{
    float near_plane = per_view_data.shadow_map_cube_data[index].cube_map_projection_near_z;
    float far_plane = per_view_data.shadow_map_cube_data[index].cube_map_projection_far_z;
    float3 light_to_surface_ws = in_position_ws.xyz - light_position_ws;
    float3 surface_to_light_dir_vs = normalize(light_position_vs - in_position_vs);
    float bias_angle_factor = 1.0 - fast::max(0.0, dot(in_normal_vs, surface_to_light_dir_vs));
    bias_angle_factor = pow(bias_angle_factor, 3.0);
    float bias0 = 0.000600000028498470783233642578125 + (0.006000000052154064178466796875 * bias_angle_factor);
    float3 param = light_to_surface_ws;
    float param_1 = near_plane;
    float param_2 = far_plane;
    float depth_of_surface = calculate_cubemap_equivalent_depth(param, param_1, param_2);
    float3 param_3 = light_to_surface_ws;
    float3 uv_and_face = cube_sample_to_uv_and_face_index(param_3);
    float4 uv_min_uv_max = per_view_data.shadow_map_cube_data[index].uv_min_uv_max[int(uv_and_face.z)];
    if (uv_min_uv_max.x < 0.0)
    {
        return 1.0;
    }
    float2 uv_to_sample = mix(uv_min_uv_max.xy, uv_min_uv_max.zw, uv_and_face.xy);
    float3 _515 = float3(uv_to_sample, depth_of_surface + bias0);
    float shadow = shadow_map_atlas.sample_compare(smp_depth_nearest, _515.xy, _515.z);
    return shadow;
}

static inline __attribute__((always_inline))
float calculate_percent_lit_cube(thread const float3& light_position_ws, thread const float3& light_position_vs, thread const float3& normal_vs, thread const int& index, thread const float& bias_multiplier, constant PerViewData& per_view_data, thread float4& in_position_ws, thread float3& in_position_vs, thread float3& in_normal_vs, thread depth2d<float> shadow_map_atlas, thread sampler smp_depth_nearest)
{
    if (index == (-1))
    {
        return 1.0;
    }
    float3 param = light_position_ws;
    float3 param_1 = light_position_vs;
    float3 param_2 = normal_vs;
    int param_3 = index;
    float param_4 = bias_multiplier;
    return do_calculate_percent_lit_cube(param, param_1, param_2, param_3, param_4, per_view_data, in_position_ws, in_position_vs, in_normal_vs, shadow_map_atlas, smp_depth_nearest);
}

static inline __attribute__((always_inline))
float3 iterate_point_and_spot_lights_clustered(thread const float3& surface_to_eye_vs, thread const float4& base_color, thread const float& metalness, thread const float& roughness, thread const float3& normal_vs, thread const float3& fresnel_base, thread const float& roughness_ndf_filtered_squared, thread const uint& light_cluster_index, constant PerViewData& per_view_data, thread float4& in_position_ws, thread float3& in_position_vs, thread float3& in_normal_vs, thread depth2d<float> shadow_map_atlas, thread sampler smp_depth_nearest, thread float3x3& in_model_view, thread sampler smp_depth_linear, device AllLights& all_lights, device LightBinOutput& light_bin_output)
{
    float3 total_light = float3(0.0);
    uint light_first = light_bin_output.data.offsets[light_cluster_index].first_light;
    uint light_last = light_first + light_bin_output.data.offsets[light_cluster_index].count;
    LightInList_1 light;
    for (uint light_list_index = light_first; light_list_index < light_last; light_list_index++)
    {
        uint light_index = light_bin_output.data.data[light_list_index];
        light.position_ws = float3(all_lights.data[light_index].position_ws);
        light.range = all_lights.data[light_index].range;
        light.position_vs = float3(all_lights.data[light_index].position_vs);
        light.intensity = all_lights.data[light_index].intensity;
        light.color = all_lights.data[light_index].color;
        light.spotlight_direction_ws = float3(all_lights.data[light_index].spotlight_direction_ws);
        light.spotlight_half_angle = all_lights.data[light_index].spotlight_half_angle;
        light.spotlight_direction_vs = float3(all_lights.data[light_index].spotlight_direction_vs);
        light.shadow_map = all_lights.data[light_index].shadow_map;
        if (dot(light.spotlight_direction_vs, light.spotlight_direction_vs) > 0.00999999977648258209228515625)
        {
            float light_surface_distance = distance(light.position_ws, in_position_ws.xyz);
            float range = light.range;
            if (light_surface_distance <= range)
            {
                float param = range;
                float param_1 = light_surface_distance;
                float soft_falloff_factor = attenuate_light_for_range(param, param_1);
                float3 param_2 = light.position_vs;
                float3 param_3 = light.color.xyz;
                float param_4 = light.intensity;
                float3 param_5 = light.spotlight_direction_vs;
                float param_6 = light.spotlight_half_angle;
                float3 param_7 = surface_to_eye_vs;
                float3 param_8 = in_position_vs;
                float3 param_9 = normal_vs;
                float3 param_10 = fresnel_base;
                float3 param_11 = base_color.xyz;
                float param_12 = roughness;
                float param_13 = roughness_ndf_filtered_squared;
                float param_14 = metalness;
                float3 pbr = spot_light_pbr(param_2, param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10, param_11, param_12, param_13, param_14) * soft_falloff_factor;
                float percent_lit = 1.0;
                if (any(pbr > float3(0.0)))
                {
                    float3 param_15 = normal_vs;
                    int param_16 = light.shadow_map;
                    float param_17 = 1.0;
                    percent_lit = calculate_percent_lit(param_15, param_16, param_17, per_view_data, in_position_ws, shadow_map_atlas, in_model_view, smp_depth_linear);
                }
                total_light += (pbr * percent_lit);
            }
        }
        else
        {
            float light_surface_distance_1 = distance(light.position_ws, in_position_ws.xyz);
            float range_1 = light.range;
            if (light_surface_distance_1 <= range_1)
            {
                float param_18 = range_1;
                float param_19 = light_surface_distance_1;
                float soft_falloff_factor_1 = attenuate_light_for_range(param_18, param_19);
                float3 param_20 = light.position_vs;
                float3 param_21 = light.color.xyz;
                float param_22 = light.intensity;
                float3 param_23 = surface_to_eye_vs;
                float3 param_24 = in_position_vs;
                float3 param_25 = normal_vs;
                float3 param_26 = fresnel_base;
                float3 param_27 = base_color.xyz;
                float param_28 = roughness;
                float param_29 = roughness_ndf_filtered_squared;
                float param_30 = metalness;
                float3 pbr_1 = point_light_pbr(param_20, param_21, param_22, param_23, param_24, param_25, param_26, param_27, param_28, param_29, param_30) * soft_falloff_factor_1;
                float percent_lit_1 = 1.0;
                if (any(pbr_1 > float3(0.0)))
                {
                    float3 param_31 = light.position_ws;
                    float3 param_32 = light.position_vs;
                    float3 param_33 = normal_vs;
                    int param_34 = light.shadow_map;
                    float param_35 = 1.0;
                    percent_lit_1 = calculate_percent_lit_cube(param_31, param_32, param_33, param_34, param_35, per_view_data, in_position_ws, in_position_vs, in_normal_vs, shadow_map_atlas, smp_depth_nearest);
                }
                total_light += (pbr_1 * percent_lit_1);
            }
        }
    }
    return total_light;
}

static inline __attribute__((always_inline))
float3 iterate_point_and_spot_lights_all(thread const float3& surface_to_eye_vs, thread const float4& base_color, thread const float& metalness, thread const float& roughness, thread const float3& normal_vs, thread const float3& fresnel_base, thread const float& roughness_ndf_filtered_squared, thread const uint& light_cluster_index, constant PerViewData& per_view_data, thread float4& in_position_ws, thread float3& in_position_vs, thread float3& in_normal_vs, thread depth2d<float> shadow_map_atlas, thread sampler smp_depth_nearest, thread float3x3& in_model_view, thread sampler smp_depth_linear, device AllLights& all_lights)
{
    float3 total_light = float3(0.0);
    LightInList_1 light;
    for (uint light_index = 0u; light_index < all_lights.light_count; light_index++)
    {
        light.position_ws = float3(all_lights.data[light_index].position_ws);
        light.range = all_lights.data[light_index].range;
        light.position_vs = float3(all_lights.data[light_index].position_vs);
        light.intensity = all_lights.data[light_index].intensity;
        light.color = all_lights.data[light_index].color;
        light.spotlight_direction_ws = float3(all_lights.data[light_index].spotlight_direction_ws);
        light.spotlight_half_angle = all_lights.data[light_index].spotlight_half_angle;
        light.spotlight_direction_vs = float3(all_lights.data[light_index].spotlight_direction_vs);
        light.shadow_map = all_lights.data[light_index].shadow_map;
        if (dot(light.spotlight_direction_vs, light.spotlight_direction_vs) > 0.00999999977648258209228515625)
        {
            float light_surface_distance = distance(light.position_ws, in_position_ws.xyz);
            float range = light.range;
            if (light_surface_distance <= range)
            {
                float param = range;
                float param_1 = light_surface_distance;
                float soft_falloff_factor = attenuate_light_for_range(param, param_1);
                float3 param_2 = light.position_vs;
                float3 param_3 = light.color.xyz;
                float param_4 = light.intensity;
                float3 param_5 = light.spotlight_direction_vs;
                float param_6 = light.spotlight_half_angle;
                float3 param_7 = surface_to_eye_vs;
                float3 param_8 = in_position_vs;
                float3 param_9 = normal_vs;
                float3 param_10 = fresnel_base;
                float3 param_11 = base_color.xyz;
                float param_12 = roughness;
                float param_13 = roughness_ndf_filtered_squared;
                float param_14 = metalness;
                float3 pbr = spot_light_pbr(param_2, param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10, param_11, param_12, param_13, param_14) * soft_falloff_factor;
                float percent_lit = 1.0;
                if (any(pbr > float3(0.0)))
                {
                    float3 param_15 = normal_vs;
                    int param_16 = light.shadow_map;
                    float param_17 = 1.0;
                    percent_lit = calculate_percent_lit(param_15, param_16, param_17, per_view_data, in_position_ws, shadow_map_atlas, in_model_view, smp_depth_linear);
                }
                total_light += (pbr * percent_lit);
            }
        }
        else
        {
            float light_surface_distance_1 = distance(light.position_ws, in_position_ws.xyz);
            float range_1 = light.range;
            if (light_surface_distance_1 <= range_1)
            {
                float param_18 = range_1;
                float param_19 = light_surface_distance_1;
                float soft_falloff_factor_1 = attenuate_light_for_range(param_18, param_19);
                float3 param_20 = light.position_vs;
                float3 param_21 = light.color.xyz;
                float param_22 = light.intensity;
                float3 param_23 = surface_to_eye_vs;
                float3 param_24 = in_position_vs;
                float3 param_25 = normal_vs;
                float3 param_26 = fresnel_base;
                float3 param_27 = base_color.xyz;
                float param_28 = roughness;
                float param_29 = roughness_ndf_filtered_squared;
                float param_30 = metalness;
                float3 pbr_1 = point_light_pbr(param_20, param_21, param_22, param_23, param_24, param_25, param_26, param_27, param_28, param_29, param_30) * soft_falloff_factor_1;
                float percent_lit_1 = 1.0;
                if (any(pbr_1 > float3(0.0)))
                {
                    float3 param_31 = light.position_ws;
                    float3 param_32 = light.position_vs;
                    float3 param_33 = normal_vs;
                    int param_34 = light.shadow_map;
                    float param_35 = 1.0;
                    percent_lit_1 = calculate_percent_lit_cube(param_31, param_32, param_33, param_34, param_35, per_view_data, in_position_ws, in_position_vs, in_normal_vs, shadow_map_atlas, smp_depth_nearest);
                }
                total_light += (pbr_1 * percent_lit_1);
            }
        }
    }
    return total_light;
}

static inline __attribute__((always_inline))
float3 directional_light_pbr(thread const DirectionalLight& light, thread const float3& surface_to_eye_dir_vs, thread const float3& surface_position_vs, thread const float3& normal_vs, thread const float3& F0, thread const float3& base_color, thread const float& roughness, thread const float& roughness_ndf_filtered_squared, thread const float& metalness)
{
    float3 surface_to_light_dir_vs = -light.direction_vs;
    float3 radiance = light.color.xyz * light.intensity;
    float3 param = surface_to_light_dir_vs;
    float3 param_1 = surface_to_eye_dir_vs;
    float3 param_2 = normal_vs;
    float3 param_3 = F0;
    float3 param_4 = base_color;
    float param_5 = roughness;
    float param_6 = roughness_ndf_filtered_squared;
    float param_7 = metalness;
    float3 param_8 = radiance;
    return shade_pbr(param, param_1, param_2, param_3, param_4, param_5, param_6, param_7, param_8);
}

static inline __attribute__((always_inline))
float4 pbr_path(thread const float3& surface_to_eye_vs, thread const float4& base_color, thread const float4& emissive_color, thread const float& metalness, thread const float& roughness, thread const float3& normal_vs, thread const uint& light_cluster_index, thread const float& ambient_factor, constant PerViewData& per_view_data, thread float4& in_position_ws, thread float3& in_position_vs, thread float3& in_normal_vs, thread depth2d<float> shadow_map_atlas, thread sampler smp_depth_nearest, thread float3x3& in_model_view, thread sampler smp_depth_linear, device AllLights& all_lights, device LightBinOutput& light_bin_output, device AllDrawData& all_draw_data, thread uint& in_instance_index, device AllMaterials& all_materials)
{
    float3 fresnel_base = float3(0.039999999105930328369140625);
    fresnel_base = mix(fresnel_base, base_color.xyz, float3(metalness));
    float3 param = normal_vs;
    float param_1 = roughness * roughness;
    float param_2 = per_view_data.ndf_filter_amount;
    float roughness_ndf_filtered_squared = DeferredLightingNDFRoughnessFilter(param, param_1, param_2);
    float3 total_light = float3(0.0);
    if (per_view_data.use_clustered_lighting != 0u)
    {
        float3 param_3 = surface_to_eye_vs;
        float4 param_4 = base_color;
        float param_5 = metalness;
        float param_6 = roughness;
        float3 param_7 = normal_vs;
        float3 param_8 = fresnel_base;
        float param_9 = roughness_ndf_filtered_squared;
        uint param_10 = light_cluster_index;
        total_light = iterate_point_and_spot_lights_clustered(param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10, per_view_data, in_position_ws, in_position_vs, in_normal_vs, shadow_map_atlas, smp_depth_nearest, in_model_view, smp_depth_linear, all_lights, light_bin_output);
    }
    else
    {
        float3 param_11 = surface_to_eye_vs;
        float4 param_12 = base_color;
        float param_13 = metalness;
        float param_14 = roughness;
        float3 param_15 = normal_vs;
        float3 param_16 = fresnel_base;
        float param_17 = roughness_ndf_filtered_squared;
        uint param_18 = light_cluster_index;
        total_light = iterate_point_and_spot_lights_all(param_11, param_12, param_13, param_14, param_15, param_16, param_17, param_18, per_view_data, in_position_ws, in_position_vs, in_normal_vs, shadow_map_atlas, smp_depth_nearest, in_model_view, smp_depth_linear, all_lights);
    }
    DirectionalLight param_19;
    for (uint i = 0u; i < per_view_data.directional_light_count; i++)
    {
        param_19.direction_ws = float3(per_view_data.directional_lights[i].direction_ws);
        param_19.intensity = per_view_data.directional_lights[i].intensity;
        param_19.color = per_view_data.directional_lights[i].color;
        param_19.direction_vs = float3(per_view_data.directional_lights[i].direction_vs);
        param_19.shadow_map = per_view_data.directional_lights[i].shadow_map;
        float3 param_20 = surface_to_eye_vs;
        float3 param_21 = in_position_vs;
        float3 param_22 = normal_vs;
        float3 param_23 = fresnel_base;
        float3 param_24 = base_color.xyz;
        float param_25 = roughness;
        float param_26 = roughness_ndf_filtered_squared;
        float param_27 = metalness;
        float3 pbr = directional_light_pbr(param_19, param_20, param_21, param_22, param_23, param_24, param_25, param_26, param_27);
        float percent_lit = 1.0;
        if (any(pbr > float3(0.0)))
        {
            float3 param_28 = normal_vs;
            int param_29 = per_view_data.directional_lights[i].shadow_map;
            float param_30 = 1.0;
            percent_lit = calculate_percent_lit(param_28, param_29, param_30, per_view_data, in_position_ws, shadow_map_atlas, in_model_view, smp_depth_linear);
        }
        total_light += (pbr * percent_lit);
    }
    float3 ambient = (per_view_data.ambient_light.xyz * base_color.xyz) * ambient_factor;
    uint material_index = all_draw_data.draw_data[in_instance_index].material_index;
    MaterialDbEntry per_material_data;
    per_material_data.base_color_factor = all_materials.materials[material_index].base_color_factor;
    per_material_data.emissive_factor = float3(all_materials.materials[material_index].emissive_factor);
    per_material_data.metallic_factor = all_materials.materials[material_index].metallic_factor;
    per_material_data.roughness_factor = all_materials.materials[material_index].roughness_factor;
    per_material_data.normal_texture_scale = all_materials.materials[material_index].normal_texture_scale;
    per_material_data.alpha_threshold = all_materials.materials[material_index].alpha_threshold;
    per_material_data.enable_alpha_blend = all_materials.materials[material_index].enable_alpha_blend != 0u;
    per_material_data.enable_alpha_clip = all_materials.materials[material_index].enable_alpha_clip != 0u;
    per_material_data.color_texture = all_materials.materials[material_index].color_texture;
    per_material_data.base_color_texture_has_alpha_channel = all_materials.materials[material_index].base_color_texture_has_alpha_channel != 0u;
    per_material_data.metallic_roughness_texture = all_materials.materials[material_index].metallic_roughness_texture;
    per_material_data.normal_texture = all_materials.materials[material_index].normal_texture;
    per_material_data.emissive_texture = all_materials.materials[material_index].emissive_texture;
    float alpha = 1.0;
    if (per_material_data.enable_alpha_blend)
    {
        alpha = base_color.w;
    }
    else
    {
        bool _1729;
        if (per_material_data.enable_alpha_clip)
        {
            _1729 = base_color.w < per_material_data.alpha_threshold;
        }
        else
        {
            _1729 = per_material_data.enable_alpha_clip;
        }
        if (_1729)
        {
            alpha = 0.0;
        }
    }
    float3 color = (ambient + total_light) + emissive_color.xyz;
    return float4(color, alpha);
}

static inline __attribute__((always_inline))
float4 pbr_main(thread sampler smp, constant PerViewData& per_view_data, thread float4& in_position_ws, thread float3& in_position_vs, thread float3& in_normal_vs, thread depth2d<float> shadow_map_atlas, thread sampler smp_depth_nearest, thread float3x3& in_model_view, thread sampler smp_depth_linear, device AllLights& all_lights, device LightBinOutput& light_bin_output, device AllDrawData& all_draw_data, constant spvDescriptorSetBuffer3& spvDescriptorSet3, thread uint& in_instance_index, thread float4& gl_FragCoord, thread float2& in_uv, thread texture2d<float> ssao_texture, thread float3& in_tangent_vs, thread float3& in_binormal_vs)
{
    uint material_index = all_draw_data.draw_data[in_instance_index].material_index;
    MaterialDbEntry per_material_data;
    per_material_data.base_color_factor = spvDescriptorSet3.all_materials->materials[material_index].base_color_factor;
    per_material_data.emissive_factor = float3(spvDescriptorSet3.all_materials->materials[material_index].emissive_factor);
    per_material_data.metallic_factor = spvDescriptorSet3.all_materials->materials[material_index].metallic_factor;
    per_material_data.roughness_factor = spvDescriptorSet3.all_materials->materials[material_index].roughness_factor;
    per_material_data.normal_texture_scale = spvDescriptorSet3.all_materials->materials[material_index].normal_texture_scale;
    per_material_data.alpha_threshold = spvDescriptorSet3.all_materials->materials[material_index].alpha_threshold;
    per_material_data.enable_alpha_blend = spvDescriptorSet3.all_materials->materials[material_index].enable_alpha_blend != 0u;
    per_material_data.enable_alpha_clip = spvDescriptorSet3.all_materials->materials[material_index].enable_alpha_clip != 0u;
    per_material_data.color_texture = spvDescriptorSet3.all_materials->materials[material_index].color_texture;
    per_material_data.base_color_texture_has_alpha_channel = spvDescriptorSet3.all_materials->materials[material_index].base_color_texture_has_alpha_channel != 0u;
    per_material_data.metallic_roughness_texture = spvDescriptorSet3.all_materials->materials[material_index].metallic_roughness_texture;
    per_material_data.normal_texture = spvDescriptorSet3.all_materials->materials[material_index].normal_texture;
    per_material_data.emissive_texture = spvDescriptorSet3.all_materials->materials[material_index].emissive_texture;
    float4 base_color = per_material_data.base_color_factor;
    float ambient_factor = 1.0;
    uint light_cluster_index = get_light_cluster_index(per_view_data, in_position_vs, gl_FragCoord);
    if (per_material_data.color_texture != (-1))
    {
        float4 sampled_color = spvDescriptorSet3.all_material_textures[per_material_data.color_texture].sample(smp, in_uv, bias(per_view_data.mip_bias));
        if (per_material_data.base_color_texture_has_alpha_channel)
        {
            base_color *= sampled_color;
        }
        else
        {
            base_color = float4(base_color.xyz * sampled_color.xyz, base_color.w);
        }
    }
    float screen_coord_x = gl_FragCoord.x / float(per_view_data.viewport_width);
    float screen_coord_y = gl_FragCoord.y / float(per_view_data.viewport_height);
    ambient_factor = ssao_texture.sample(smp, float2(screen_coord_x, screen_coord_y)).x;
    float4 emissive_color = float4(per_material_data.emissive_factor, 1.0);
    if (per_material_data.emissive_texture != (-1))
    {
        emissive_color *= spvDescriptorSet3.all_material_textures[per_material_data.emissive_texture].sample(smp, in_uv, bias(per_view_data.mip_bias));
    }
    float metalness = per_material_data.metallic_factor;
    float roughness = per_material_data.roughness_factor;
    if (per_material_data.metallic_roughness_texture != (-1))
    {
        float4 sampled = spvDescriptorSet3.all_material_textures[per_material_data.metallic_roughness_texture].sample(smp, in_uv, bias(per_view_data.mip_bias));
        metalness *= sampled.z;
        roughness *= sampled.y;
    }
    metalness = fast::clamp(metalness, 0.0, 1.0);
    roughness = fast::clamp(roughness, 0.0, 1.0);
    float3 normal_vs;
    if (per_material_data.normal_texture != (-1))
    {
        float3x3 tbn = float3x3(float3(in_tangent_vs), float3(in_binormal_vs), float3(in_normal_vs));
        int param = per_material_data.normal_texture;
        float3x3 param_1 = tbn;
        float2 param_2 = in_uv;
        normal_vs = normal_map(spvDescriptorSet3, param, param_1, param_2, smp, per_view_data).xyz;
    }
    else
    {
        normal_vs = normalize(float4(in_normal_vs, 0.0)).xyz;
    }
    float3 eye_position_vs = float3(0.0);
    float3 surface_to_eye_vs = normalize(eye_position_vs - in_position_vs);
    float3 param_3 = surface_to_eye_vs;
    float4 param_4 = base_color;
    float4 param_5 = emissive_color;
    float param_6 = metalness;
    float param_7 = roughness;
    float3 param_8 = normal_vs;
    uint param_9 = light_cluster_index;
    float param_10 = ambient_factor;
    float4 out_color = pbr_path(param_3, param_4, param_5, param_6, param_7, param_8, param_9, param_10, per_view_data, in_position_ws, in_position_vs, in_normal_vs, shadow_map_atlas, smp_depth_nearest, in_model_view, smp_depth_linear, all_lights, light_bin_output, all_draw_data, in_instance_index, *spvDescriptorSet3.all_materials);
    return out_color;
}

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant spvDescriptorSetBuffer1& spvDescriptorSet1 [[buffer(1)]], constant spvDescriptorSetBuffer2& spvDescriptorSet2 [[buffer(2)]], constant spvDescriptorSetBuffer3& spvDescriptorSet3 [[buffer(3)]], float4 gl_FragCoord [[position]])
{
    constexpr sampler smp(filter::linear, mip_filter::linear, address::repeat, compare_func::never, max_anisotropy(16));
    constexpr sampler smp_depth_nearest(mip_filter::nearest, compare_func::greater, max_anisotropy(1), lod_clamp(0.0, 0.0));
    constexpr sampler smp_depth_linear(filter::linear, mip_filter::linear, compare_func::greater, max_anisotropy(1));
    main0_out out = {};
    float3x3 in_model_view = {};
    in_model_view[0] = in.in_model_view_0;
    in_model_view[1] = in.in_model_view_1;
    in_model_view[2] = in.in_model_view_2;
    out.out_color = pbr_main(smp, (*spvDescriptorSet0.per_view_data), in.in_position_ws, in.in_position_vs, in.in_normal_vs, spvDescriptorSet0.shadow_map_atlas, smp_depth_nearest, in_model_view, smp_depth_linear, (*spvDescriptorSet0.all_lights), (*spvDescriptorSet0.light_bin_output), (*spvDescriptorSet2.all_draw_data), spvDescriptorSet3, in.in_instance_index, gl_FragCoord, in.in_uv, spvDescriptorSet1.ssao_texture, in.in_tangent_vs, in.in_binormal_vs);
    return out;
}

    d{      #     ~             2        GLSL.std.450                     main    �  �  �  C    �  \  �  �            G  �   "      G  �   !      G  �   "       G  �   !      H  �       #       H  �      #      H  �      #      H  �      #       H  �      #   ,   G  �      0   H  �       #       H  �      #      H  �         H  �      #      H  �            H  �      #   P   G  �      `   G  �         H  �       #       H  �      #   `   H  �      #   d   G  �      p   H  �          H  �       #       H  �             H  �         H  �      #   @   H  �            H  �      #   �   H  �      #   �   H  �      #   �   H  �      #   �   H  �      #   �   H  �      #   �   H  �      #   �   H  �   	   #   �   H  �   
   #   �   H  �      #   0  H  �      #   0&  G  �      G  �   "       G  �   !       G  �        G  �         G  �        G  �  "       G  �  !      G  �  "       G  �  !      G  C        G  k  "       G  k  !      H  F      #       H  F     #      H  F     #      H  F     #      H  F     #       H  F     #   0   H  F     #   <   H  F     #   @   H  F     #   L   G  H     P   H  I      #       H  I     #      G  I     G  K  "       G  K  !      H        #       H       #      G          G          H        #       H       #      H       #      H       #      H       #      H       #   `  H        #       G       G    "       G    !      H  y      #       H  y     #      G  z        H  {      #       H  {     #      H  {     #      H  {     #      H  {     #      G  {     G  }  "      G  }  !      G       G       	   H  �      #       H  �     #      H  �     #      H  �     #       H  �     #   $   H  �     #   (   H  �     #   ,   H  �     #   0   H  �     #   4   H  �  	   #   8   H  �  
   #   <   H  �     #   @   H  �     #   D   G  �     P   H  �      #       G  �     G  �  "      G  �  !       G  �        G  \        G  �  "      G  �  !       G  �        G  �        G                !                               	           
   	                              �            	 �                            +  �   �        �   �   �      �       �   ;  �   �          �       �     �      �       �   ;  �   �         �   �     �           �   	         	      +  �   �        �   �   �     �         �   	   +  �   �   `     �   �   �   +  �   �        �      �     �   �         +  �   �         �   �   �     �   �   �         �   �         �   �   �   �   �      �      �   ;  �   �      +     �         �         +     �      @+     �     �?+     �       +  �   �      +  �   �       +  �   �      +     -     ?  4  +     H    �@+     I    �@+     l    @@+     �     +     �     +     �        �        ;  �  �        �     	   ;  �  �     ;  �  �     +     �  RI:+     �  ���;+     �         �        ;  �   �      ;  �   �       	 �                            �  �  +       ����+                  �      3           B     
   ;  B  C     +     F        G     	   +     T  
�#<+     [  o:;  �   k        o          �  4     +     �  �Q8>+       �I@+     ,     A+     N  Y���,  	   |  �   �   �   +     �  o�:,  	     �   �   �   +     6    @?  F  	      	         	      	      +  �   G       H  F  G    I  �   H     J     I  ;  J  K        L     �      T     F  +     `     +     c     +     h     +     k       �  4         �   �   +  �                +  �            �         �   �   �   �                       ;         +     �  
�#=,  	   �  �  �  �  +       	   +     7  
      :     �      c          y  �   �     z  y    {  �   �   �   �   z     |     {  ;  |  }        ~     �   ;  ~         �     	               �   �      �              �  �    �  �     �     �  ;  �  �        �     �  ;  �  �        �           [        ;  [  \     ;  �   �      ;  �  �     ;  �  �        
        ;  
       +         �A+         �A+       ���>,       -  -  ,  o    �  �  +  �        +  �        +       ��L�+        E�A@+  �   !  �   +  �   "     +     %     >,  	   (        +     ,  9��=+     6    ��+     7  v��+  �   ?     +  �   D  	   +  �   G  
   +  �   J     +  �   M     +  �   X     +  �   w     6               �     =  �   F    A  L  G  }  `  F  �  =  �   H  G  A  �  J  �  �  H  A  �  9  J  �   =     :  9  A  G  ;  J  �   =  	   <  ;  A  �   =  J  �   =     >  =  A  �   @  J  ?  =     A  @  A  c  B  J  �   =     C  B  A  L  E  J  D  =  �   F  E  A  c  H  J  G  =     I  H  A  c  K  J  J  =     L  K  A  c  N  J  M  =     O  N  �  4  a  F  �   A  �  	  �  �   =     	  	  A  L  	  �   `  =  �   	  	  p     	  	  �     	  	  	  �     	  	    m  �   	  	    �   #	     &   	    A  �  $	  �  �   =     %	  $	  A  L  &	  �   c  =  �   '	  &	  p     (	  '	  �     )	  %	  (	  �     *	  �   )	  �     -	  *	  ,  m  �   .	  -	    �   2	     &   .	    A  �  6	  �  �   =     7	  6	  �     :	  7	         ;	        :	       D	     2   ;	     �        H	     +   D	  �     m  �   I	  H	  �  �   O	  !  I	  �  �   S	  "  2	  �  �   T	  O	  S	  �  �   V	  T	  #	  �  4  n  C    �  �      �  n  o  �  �  o  A  �   r  �   C  =  �   s  r  =  �   t  �   V  �   u  s  t  =     v  \  A  �   w  �   �   =     x  w  W     y  u  v     x  �  �      �  a  |  �  �  �  O  	   �  :  :            O  	   �  y  y            �  	   �  �  �  Q     �  :     Q     �  �      Q     �  �     Q     �  �     P     �  �  �  �  �  �  �  �  |  �       :  y  �  �  �  �  �     ^    |  �  �  �  �  �  �  �     ]  :     ^  �  =  �   �  �  =  �   �  �   V  �   �  �  �  P     �  	  )	  W     �  �  �  Q     �  �      Q     �  <      Q     �  <     Q     �  <     P     �  �  �  �  �   �  4  �  O    �  �      �  �  �  �  �  �  A  �   �  �   O  =  �   �  �  V  �   �  �  �  =     �  \  A  �   �  �   �   =     �  �  W     �  �  �     �  �     �  �  �  �  �  �  �  �     a  �  �  �  �  �  4  �  I    �  �      �  �  �  �  �  �  A  �   �  �   I  =  �   �  �  V  �   �  �  �  =     �  \  A  �   �  �   �   =     �  �  W     �  �  �     �  Q     �  �     �     �  >  �  Q     �  �     �     �  A  �  �  �  �  �  �     Y  A  �  �  �  �     X  >  �  �  �       �     +   X  �   �        �     +   Y  �   �   �  4  �  L    �  �      �  �  �  �  �  �  =  	   �  �  Q     �  �      Q     �  �     Q     �  �     P     �  �  �  �  �        �     E   �  O  	   �  �  �            �  �  �  �  =  	   �  �  =  	   �  �  =  	   �  �  P  
   �  �  �  �  =     �  \  A  �   \	  �   L  =  �   ]	  \	  V  �   _	  ]	  �  A  �   a	  �   �   =     b	  a	  W     c	  _	  �     b	  O  	   d	  c	  c	            �  	   f	  d	  �   �  	   h	  f	  |  R  	   �  �   h	     �     l	  �  �  �     m	  �   l	       n	        m	  Q     q	  h	           r	  q	  Q     u	  h	          v	  u	  P  	   #  r	  v	  n	  �  	   z	  �  #  Q     |	  z	      Q     }	  z	     Q     ~	  z	     P     	  |	  }	  ~	  �        �	     E   	  O  	   �  �	  �	            �  �  �  �  �  	   d  �  �  �  �  =  	   �  �    	   �  �    	   �     E   �  O  	   �	  ]  ]            P  	   �	  �  �  �    	   �	     .   �  �	  �	  A  �   �	  �   h  =     �	  �	  �  	   V
  d  �  	   X
  d  �     ]
  V
  V
  �     `
  X
  X
  �     a
  ]
  `
  �     b
    a
       e
     %   b
  �  �     i
  e
  �	       j
     2   �  �  i
       k
     +   j
  �   �   A  L  �	  �     =  �   �	  �	  �  4  �	  �	  �   �  �	      �  �	  �	  �	  �  �	  �    �    �  	   i    �	  �  �  �  �   h  �   �	  �  �  A  L    K  �  =  �       �  4    h    �  �  �      �      �  �    A  T    K  �  h  A  G  P    �   =  	   Q  P  A  �   R    �   =     S  R  A  G  T    �   =  	   U  T  A  �   V    ?  =     W  V  A  �  Y    X  =     Z  Y  A  �   [    �   =     \  [  A  G  ]      =  	   ^  ]  A  c  _    �   =     `  _  �     2  ^  ^  �  4  3  2  T  �  �      �  3  4  i  �  i  =     l  �  O  	   m  l  l                 n     C   Q  m  �  4  s  n  S  �  �      �  s  t  �  �  t  �     _  S  6       b     1   _  S  n  �     c  �   b  O  	   }  Z  Z            �  	   u  U  �       w     B   u  P  	   z  w  w  w  �  	   {  u  z            2   w  w  �  �     �  �     �  	   �  }  �  �  	   �  �  W  �  	   �  {  �    	   �     E   �  �     �  k
  k
  �     �  d  �       �     (   �  �   �     �  �  �       �     2   k
  k
  6       �     2   �  �  �   �     �    �  �     �  �  �  �     �  �  �  �       �  �   �           �         %  �       d  �            (     �        &         !     2   &  %  �        $     2     !    �     '    $  �       d  {            (     �        0     2     !    �     3    0  �       '  3  �     9  �  �       :     (   9  �   �  	   >  |  �	       A     2   N  :  7  �     C  A  :       D        C  �  	   E  >  D  �  	   F  �	  E  �  	   �  |  F  �     �  �   �  �  	   �  �  �  �     �  �    �  	   �  F  �  �     �  I    �     �  �         �     (   �  �  P  	   �  �  �  �  �  	   �  �  �  �  	   �  �  �	    	   �     2   �  (  �  �  	   �  �  �  �  	   �  �    �  	   �  �  c  �  �  �  �    �  4  �  �  �  �      �  �  �  �  �  �  �  [      �  �   P  �  P  �  4  R  `    �  T      �  R  S  T  �  S  �  [  �  T  �  �      �  �   p  �  p  A  �   r  �   �  `  �  =     s  r  A  �   u  �   �  `  �  =     v  u  �  	   z  m  Q    	   ~     E   u  =  	     �  �     �    ~       �     (   �   �  �     �  �   �       �        �  l       �     2   �  �  �    	   �        z  Q     �  �      Q     �  �     Q     �  �          �     (   �  �       �     (   �  �  �     �  v  s  �     �  v  s  �     �  �  �  �     �  �   v  �     �  �  s  �     �  �  �  �     �  �  �  �     �  �  �  �     �  �  �   �  4  �  �  �  �  �      �  �  �  �  �  �  �  4  �  �  �  �  �  �  �  �  4  �  �  p  �  �  �  :      �  �  �    �    �  4  	  �  �  �  9      �  	  
  !  �  !  Q     #  z      �  4  $  #  �   �     %  $  �   �   �     (  -  �  �  3      �  $  ,  /  �  /  Q     1  z          2  1  �  3  �  ,  Q     .  z     �  3  �  3  �     j  .  ,  2  /  Q     6  z          7  6  P     8  j  7  �  9  �  
  Q       z     �  4      �   �         l  �   �       -  �  Q       z      �        �        �    Q       z     �    �    Q       z              �    �    �     k          P          k  �  9  �  9  �     v      %  3  �     q      (  3  �     n       8  3  �  :  �  �  Q     �  z     �  4  �  �  �   �     �  �  H  I  �     �  -  �  �  �      �  �  �  �  �  �  Q     �  z      �  �  �  �  Q     �  z           �  �  �  �  �  �  �     l  �  �  �  �  Q       z              P       l    �  :  �  :  �     t  �  �  v  9  �     o  �  �  q  9  �     m    �  n  9  �     =  m  o  �     ?  =    Q     A  ?      Q     B  ?     P  	   C  A  B  t  n     �  t  A  �  �  �   �  `  �  �  =     �  �  Q     �  �      �  4  �  �  �   �  �      �  �  �  �  �  �  �  �  �  �  O     �  �  �         O     �  �  �        O     �  C  C              �     .   �  �  �  =  �   �  �  =  �   �  �  V  �  �  �  �       �     2   �  -  �  Q     �  �      Q     �  �     P  	   �  �  �  �  Y     �  �  �  �  �  �  �  �  �     y  �   �  �  �  �  [  �  [  �     z  �   S  y  �  �  �  �  �  �     {  �   t  z  [  �  	   �  �  {  �  	   �  i  �  �  �  �  �  �  	     i  i  �  �  �  �  �  4  =     7  �  O  	   8  7  7                 9     C   Q  8  �  4  >  9  S  �  h      �  >  ?  h  �  ?  �     �  S  6       �     1   �  S  9  �     �  �   �  O  	   H  Z  Z            �  �      �  �   �  �  �  �  	   �  U  �       �     B   �  P  	   �  �  �  �  �  	   �  �  �       �     2   �  �  �  �     �  �   �    	   �  ^  �     �  �  �       �        \       �     .   �  �   -       �     1   �  �  �  �     �  �  W  �     �  �  �  �  4  �  �  �   �  �      �  �  �  �  �  �  �  �  �  �  �  	   �  H  �  �  	     �  �    	        E     �     W  k
  k
  �     Z  d         [     (   Z  �   �     ^  [  [       `     2   k
  k
  6       b     2   ^  `  �   �     d    b  �     f  d  b  �     i  W  f  �     u  �  �   �     x  u  u  �     y  x  %  �     |  d  �       }     (   |  �        *  x       �     2   *  %  �        �     2   }  �  y  �     �  }  �  �     �  d  �       �     (   �  �        �     2   �  �  y  �     �  �  �  �     �  �  �  �     �  �         �     (   �  �   �  	   �  |  �	       �     2   N  �  7  �     �  �  �       �        �  �  	   �  �  �  �  	   �  �	  �  �  	   '  |  �  �     )  �   �  �  	   +  '  )  �     6  i  �  �  	   8  �  6  �     :  I  }  �     <  :  �       ?     (   <  �  P  	   @  ?  ?  ?  �  	   A  8  @  �  	   D  +  �	    	   H     2   D  (  A  �  	   J  H  �  �  	   L  J  �  �  �  �  �  �  �  �  �  	   �  L  �    �  �  	   Y  �  �  �  �  [  Y    �  4  \  [  �  b      �  \  ]  b  �  ]  �  �      �  �   �  �  �  �  4  �  `    �  �      �  �  �  �  �  �  �  �  �  �  A    �  �     `  �  =  �   �  �  �     �  �  7  O  	   �  �  �            Q     �  �     P  	   �  �  �  �  �  	   �  �  �  O     �  �  �         �     �  �  -  �     �  �    Q     �  �     �     �  �   �  R     *  �  �     A  3  �  �     `  �  =     �  �  A  3  �  �     `  �  =     �  �       �     .   �  �  *  Q     �  �     =  
   �  C  A  G  �  �     `  F  =  	   �  �  �  	   �  �  �    	   �  �  �       d  �  �       �     �       T    �           �                	     (     [         �  Q       �      Q       �     Q             Q            P               =  �     �  =  �     k  V  �        d  �      g  o      �  �  o        o         �    �    �     �  �   �  �  U  �     �    �  W  U  �  4    �  �  �  X  U      �       X  �     �  !  �  !  �     �  �       Q  �     �       S  Q  �  4  $  �  �  �  T  Q      �  $  %  T  �  %  o     (  �  o     *  �  P     +  (  *       .     2   +    �  Q     /  .      Q     0  .     P     1  /  0  �   �   O     3  1  1              4  3  Q     6  4      R     -  6  1     Q     8  4     R     /  8  -     �  �  ;  /    �  4  <  ;  �  P      �  <  =  M  �  M  �     O  �  �   �  P  �  =  V  �  @      �     E  �  	  P  	   H  /  0  E  Y     J  @  H  E  �     L  �  J  �  P  �  P  �       L  =  O  M  �  Q  �  Q  �     S  �  �  �  !  �  T  �  U  �  U  �     W  �  �  �    �  X  �     Z  �  ,  �  �  �  �  �     �  �   �  Z  X  �  b  �  b  �     �  �   �  �  �  �  	   e  Y  �  �  	   g  i  e  �  h  �  h  �  	      i  4  g  b  �  �  �  �  �  	   �     h    �  �  �  �  �  �  �   �  h  �  �    �  �  �  �	  �  �	  A  L  �
    �  `  V	  �  =  �   �
  �
  A  L  �
    �  `  V	  �  =  �   �
  �
  �  �   �
  �
  �
  �  �
  �  �
  �  	   �    �	    :  �  �   �  �
  �	  <  :  �  4  �
  �  �
  �  =  :      �  �
  �
  =  �  �
  A  L  �
    �  c  �  =  �   �
  �
  A  T  �
  K  �  �
  A  G  a  �
  �   =  	   b  a  A  �   c  �
  �   =     d  c  A  G  e  �
  �   =  	   f  e  A  �   g  �
  ?  =     h  g  A  �  i  �
  X  =     j  i  A  �   k  �
  �   =     l  k  A  G  m  �
    =  	   n  m  A  c  o  �
  �   =     p  o  �     �
  n  n  �  4  �
  �
  T  �  9      �  �
  �
    �    =       �  O  	                        	     C   b    �  4    	  d  �  8      �      8  �    �     �  d  6       �     1   �  d  	  �     �  �   �  O  	     j  j            �  	     f  �            B     P  	           �  	                   2       �  �       �     �  	         �  	        h  �  	   D    �    	   E     E   D  �     �  k
  k
  �     �  d  E       �     (   �  �   �     �  �  �       �     2   k
  k
  6       �     2   �  �  �   �     �    �  �     �  �  �  �     �  �  �  �     �  �  �   �     �  �  �  �     �  �  %  �     �  d  �       �     (   �  �        1  �       �     2   1  %  �        �     2   �  �  �  �     �  �  �  �     �  d         �     (   �  �        �     2   �  �  �  �     �  �  �  �     �  �  �  �     �  �  E       �     (   �  �   �  	   �  |  �	       �     2   N  �  7  �     �  �  �       �        �  �  	   �  �  �  �  	   �  �	  �  �  	   U  |  �  �     W  �   �  �  	   Y  U  W  �     d  �  �  �  	   f  �  d  �     h  I  �  �     j  h  �       m     (   j  �  P  	   n  m  m  m  �  	   o  f  n  �  	   r  Y  �	    	   v     2   r  (  o  �  	   x  v     �  	   z  x  �  �  	   %  z  �  �  �  '  %    �  4  (  '  �  2      �  (  )  2  �  )  �  �      �  �   �  �  �  �  4  �  p    �  �      �  �  �  �  �  �  �  �  �  �  �  I      �  �     �    A  �     �   �  p  �  =         A  �     �   �  p  �  =         �  	       b    	        E     =  	     �  �                     (   �     �       �                     l       #     2   �     �    	   Q          Q     S  Q      Q     U  Q     Q     W  Q          X     (   U  W       Y     (   S  X  �     \      �     _      �     `  \  _  �     b  �     �     d  b    �     h  d  _  �     j  h  Y  �     k  `  j  �     m  k  �   �  4  ~  W  S  �  �      �  ~    �  �    �  4  �  W  U  �  �  �  �  �  4  �  ~    �    �  �      �  �  �  �  �  �  �  4  �  U  S  �  �      �  �  �  �  �  �  Q     �        �  4  �  �  �   �     �  �  �   �   �     �  -  S  �  �      �  �  �  �  �  �  Q     �            �  �  �  �  �  �  Q     �       �  �  �  �  �     �  �  �  �  �  Q     �            �  �  P     �  �  �  �  �  �  �  Q     �       �  4  �  �  �   �     �  �  l  �   �     �  -  U  Q     �        �  �      �  �  �  �  �  �  Q     �       �  �  �  �  Q     �            �  �  �  �  �  �  �     �  �  �  �  �  P     �  �  �  �  �  �  �  �     �  �  �  �  �  �     �  �  �  �  �  �     �  �  �  �  �  �  �  �  �  Q     �       �  4  �  �  �   �     �  �  H  I  �     �  -  W  �  �      �  �  �  �  �  �  Q     �        �  �  �  �  Q     �             �  �  �  �  �  �  �     �  �  �  �  �  Q     �            �  �  P     �  �  �  �  �  �  �  �     �  �  �  �  �  �     �  �  �  �  �  �     �  �  �  �  �  �     �  �  �  �     �  �    Q     �  �      Q     �  �     P  	   �  �  �  �  n     -  �  A  �  .  �   �  p  �  -  =     /  .  Q     1  /      �  4  2  1  �   �  4      �  2  3  4  �  3  �  I  �  4  O     6  /  /         O     8  /  /        O     :  �  �              ;     .   6  8  :  =  �   <  �  =  �   =  �  V  �  >  <  =       B     2   m  -  #  Q     C  ;      Q     D  ;     P  	   E  C  D  B  Y     G  >  E  B  �  I  �  I  �     �  �   3  G  4  �  �  �  �  �     �  �   �  �  I  �  2  �  2  �     �  �     �  �  �  	   5  %  �  �  	   7  �  5  �  8  �  8  �  	     �    7  2  �  9  �  �
  =     �
  �  O  	   �
  �
  �
                 �
     C   b  �
  �  4  �
  �
  d  �        �  �
  �
    �  �
  �     B  d  6       E     1   B  d  �
  �     F  �   E  O  	   �
  j  j            �  �      �  �   \  �  \  �  	   _  f  �       a     B   _  P  	   d  a  a  a  �  	   e  _  d       i     2   a  a  �  �     j  �   i    	   �  n  �     �  �  e       �        l       �     .   �  �   -       �     1   �  �  �  �     q  j  h  �     s  q  �  �  4  u  s  �   �  �      �  u  v  �  �  �  �  �  �  v  �  	   y  �
  s  �  	   �  e  �    	   �     E   �  �     �  k
  k
  �     �  d  �       �     (   �  �   �     �  �  �       �     2   k
  k
  6       �     2   �  �  �   �     �    �  �       �  �  �       �    �       �  �   �           �         %  �       d  �            (     �        4         )     2   4  %  �        ,     2     )    �     /    ,  �       d  e            (     �        8     2     )    �     ;    8  �     #  /  ;  �     A  �  �       B     (   A  �   �  	   F  |  �	       I     2   N  B  7  �     K  I  B       L        K  �  	   M  F  L  �  	   N  �	  M  �  	   �  |  N  �     �  �   �  �  	   �  �  �  �     �    #  �  	   �  N  �  �     �  I    �     �  �         �     (   �  �  P  	   �  �  �  �  �  	   �  �  �  �  	   �  �  �	    	   �     2   �  (  �  �  	   �  �  y  �  	   �  �    �  �  �  �  �  �  �  �  	   �  �  v    �  �  	   �
  �  F  �  �  �
  �
    �  4  �
  �
  �  �
      �  �
  �
  �
  �  �
  �  _      �  �   V  �  V  �  4  X  p    �  Z      �  X  Y  Z  �  Y  �  _  �  Z  A    t  �     p  �  =  �   u  t  �     w  u  �
  O  	   y  w  w            Q     {  w     P  	   |  {  {  {  �  	   }  y  |  O       }  }         �     �    -  �     �  �    Q     �  �     �     �  �   �  R       �  �     A  3  �  �     p  �  =     �  �  A  3  �  �     p  �  =     �  �       �     .   �  �    Q     �  }     =  
   �  C  A  G  �  �     p  F  =  	   �  �  �  	   �  �  �    	   �  �  �     �  d  �  �     �  �   �  �     �  T  �  �     �  �  �  �     �  �  �       �     (   �  [       �  �  Q     �  �      Q     �  �     Q     �  �      Q     �  �     P     �  �  �  �  �  =  �   �  �  =  �   �  k  V  �  �  �  �  d  �  �  �  g  o  �  �  �  �  o  �    �  o     �  �  �  �  �  �  �     �  �   Z  �  �  �     �    Z  �  �  �  4  �  �  �  �  �  �      �  �  �  �  �  �  �  �  �  �  �     �  �  �    �  �     �    �  �  �  �  4  �  �  �  �  �  �      �  �  �  �  �  �  o     �  �  o     �  �  P     �  �  �       �     2   �  �  �  Q     �  �      Q     �  �     P     �  �  �  �   �   O     �  �  �              �  �  Q     �  �      R       �  �     Q     �  �     R     	  �       �  �  �  	  �  �  4  �  �  �  �      �  �  �  �  �  �  �     �  �  �   �  �  �  �  V  �  �  �  �  �     �  �  �  P  	   �  �  �  �  Y     �  �  �  �  �     �  �  �  �  �  �  �  �       �  �  �  �  �  �  �  �  �     �  �  �  �  �  �  �  �  �  �  �  �     �  �  �  �  �  �  �  �     �  �  ,  �  _  �  _  �     �  �   Y  �  �  �  �
  �  �
  �     �  �   �  �  _  �  	      �
  �  �  	     �     �    �    �  	     �  �
    �
  �  9  �  9  �  	           8  �  :  �  :  �  �   <  �  �  �  �
  �  =  �  �	  �  �	  �  	   �  �  =  i  �  �  �	  �  �	  �  	   �  �  �	  �	  �	  �  �   �  �   �	  
  �	  A  L  �	  �   k  =  �   �	  �	  �  4  �	  �  �	  �  
  �	      �  �	  �	  
  �  �	  A  :  �	  �   7  �  A  �   q  �	  �   =     r  q  A  �  s  �	  �   =     t  s  A  G  u  �	  ?  =  	   v  u    	   S  v  O  	   V  t  t            �  	   Y  V  r  �  	   }  S  �    	   ~     E   }  �     �  k
  k
  �     �  d  ~       �     (   �  �   �     �  �  �       �     2   k
  k
  6       �     2   �  �  �   �     �    �  �     �  �  �  �     �  �  �  �     �  �  �   �     �  �  �  �     �  �  %  �     �  d  �       �     (   �  �        .  �       �     2   .  %  �        �     2   �  �  �  �     �  �  �  �     �  d  S       �     (   �  �             2   �  �  �  �       �    �     �  �    �       �  ~            (     �   �  	     |  �	            2   N    7  �                          �  	         �  	     �	    �  	   �  |    �     �  �   �  �  	   �  �  �  �     �  �  �  �  	   �    �  �     �  I  �  �     �  �  �       �     (   �  �  P  	   �  �  �  �  �  	   �  �  �  �  	   �  �  �	    	   �     2   �  (  �  �  	   �  �  Y  �  	   �  �  �  �  �  �	  �    �  4  �	  �	  �  �	      �  �	  �	  �	  �  �	  A  c  �	  �   7  �  `  =     �	  �	  �  +      �  �   "  �  "  �  4  $  �	    �  &      �  $  %  &  �  %  �  +  �  &  A    @  �     �	  �  =  �   A  @  =     B  �  �     C  A  B  O  	   E  C  C            Q     G  C     P  	   H  G  G  G  �  	   I  E  H  O     K  I  I         �     L  K  -  �     N  L    Q     P  N     �     Q  �   P  R     P  Q  N     A  3  T  �     �	  �  =     U  T  A  3  W  �     �	  �  =     X  W       \     .   U  X  P  Q     ^  I     =  
   _  C  A  G  a  �     �	  F  =  	   b  a  �  	   c  _  b    	   e  c  �     h  d  e  �     i  �   h  �     k  T  i  �     m  k  i  �     o  m  i       p     (   o  [       u  X  Q     v  U      Q     w  U     Q     x  u      Q     y  u     P     z  v  w  x  y  =  �   {  �  =  �   |  k  V  �  }  {  |  d  �  ~  }  g  o    ~  �  �  o  �      o     �  �  �  �  �  �  �     �  �   &  �  �  �     �    &  �  �  �  4  �  �  �  �  �  �      �  �  �  �  �  �  �  �  �  �  �     �  �  �  
  �  �     �    �  �  �  �  4  �  �  �  �  �  �      �  �  �  �  �  �  o     �  �  o     �  �  P     �  �  �       �     2   �  �  \  Q     �  �      Q     �  �     P     �  �  �  �   �   O     �  �  �              �  �  Q     �  �      R     S  �  �     Q     �  �     R     U  �  S     �  �  �  U  z  �  4  �  �  �  �      �  �  �  �  �  �  �     �  �  �   �  �  �  �  V  �  �  {  |  �     �  ^  p  P  	   �  �  �  �  Y     �  �  �  �  �     �  �  �  �  �  �  �  �     
  �  �  �  �  �  �  �  �  �     �  �  �  �  �  �  �  �  �  �  �  �     �  �  �  �  �  �  �  �     �  �  ,  �  +  �  +  �     �  �   %  �  �  �  �	  �  �	  �     �  �   �	  �  +  �  	   �	  �  �  �  	   �	  �  �	  �  �	  �  �	  �  �   
  �  �  �  �	  �  
  A  �  
  �   �  =     
  
  O  	   
  
  
            �  	   
  
  �	  �  	   

  
  �  =  �   
  G  A  �  
  �  �  
  A  �   x  
  w  =     y  x  A  L  z  
  �   =  �   {  z  A  L  |  
    =  �   }  |  �  4  
  {  �   �  4  !
  }  �   �  @
      �  
  0
  3
  �  3
  �  <
      �  !
  6
  <
  �  6
  Q     8
  ]     �  4  ;
  8
  y  �  <
  �  <
  �  4  =
  !
  3
  ;
  6
  �     8  =
  �   �   �  @
  �  0
  Q     2
  ]     �  @
  �  @
  �     �  2
  0
  8  <
  �  	   C
  

  �  O  	   E
  a  a            �  	   F
  C
  E
  Q     I
  F
      Q     J
  F
     Q     K
  F
     P     L
  I
  J
  K
  �  >    L
  �  8                   �                            PerViewData               PerViewData �             PerViewData.view           PerViewData.view_proj@          PerViewData.ambient_light�          PerViewData.jitter_amount�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.mip_bias�          PerViewData.ndf_filter_amount�   #       PerViewData.directional_light_count�   "       PerViewData.use_clustered_lighting�   .       PerViewData.directional_lights[0].direction_ws�   +       PerViewData.directional_lights[0].intensity�   '       PerViewData.directional_lights[0].color�   .       PerViewData.directional_lights[0].direction_vs�   ,       PerViewData.directional_lights[0].shadow_map�   .       PerViewData.directional_lights[1].direction_ws�   +       PerViewData.directional_lights[1].intensity�   '       PerViewData.directional_lights[1].color�   .       PerViewData.directional_lights[1].direction_vs   ,       PerViewData.directional_lights[1].shadow_map  .       PerViewData.directional_lights[2].direction_ws  +       PerViewData.directional_lights[2].intensity  '       PerViewData.directional_lights[2].color   .       PerViewData.directional_lights[2].direction_vs0  ,       PerViewData.directional_lights[2].shadow_map<  .       PerViewData.directional_lights[3].direction_ws@  +       PerViewData.directional_lights[3].intensityL  '       PerViewData.directional_lights[3].colorP  .       PerViewData.directional_lights[3].direction_vs`  ,       PerViewData.directional_lights[3].shadow_mapl  .       PerViewData.directional_lights[4].direction_wsp  +       PerViewData.directional_lights[4].intensity|  '       PerViewData.directional_lights[4].color�  .       PerViewData.directional_lights[4].direction_vs�  ,       PerViewData.directional_lights[4].shadow_map�  .       PerViewData.directional_lights[5].direction_ws�  +       PerViewData.directional_lights[5].intensity�  '       PerViewData.directional_lights[5].color�  .       PerViewData.directional_lights[5].direction_vs�  ,       PerViewData.directional_lights[5].shadow_map�  .       PerViewData.directional_lights[6].direction_ws�  +       PerViewData.directional_lights[6].intensity�  '       PerViewData.directional_lights[6].color�  .       PerViewData.directional_lights[6].direction_vs�  ,       PerViewData.directional_lights[6].shadow_map�  .       PerViewData.directional_lights[7].direction_ws   +       PerViewData.directional_lights[7].intensity  '       PerViewData.directional_lights[7].color  .       PerViewData.directional_lights[7].direction_vs   ,       PerViewData.directional_lights[7].shadow_map,  (       PerViewData.shadow_map_2d_data[0].uv_min0  (       PerViewData.shadow_map_2d_data[0].uv_max8  6       PerViewData.shadow_map_2d_data[0].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[0].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[1].uv_min�  (       PerViewData.shadow_map_2d_data[1].uv_max�  6       PerViewData.shadow_map_2d_data[1].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[1].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[2].uv_min�  (       PerViewData.shadow_map_2d_data[2].uv_max�  6       PerViewData.shadow_map_2d_data[2].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[2].shadow_map_light_dir@  (       PerViewData.shadow_map_2d_data[3].uv_minP  (       PerViewData.shadow_map_2d_data[3].uv_maxX  6       PerViewData.shadow_map_2d_data[3].shadow_map_view_proj`  6       PerViewData.shadow_map_2d_data[3].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[4].uv_min�  (       PerViewData.shadow_map_2d_data[4].uv_max�  6       PerViewData.shadow_map_2d_data[4].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[4].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[5].uv_min  (       PerViewData.shadow_map_2d_data[5].uv_max  6       PerViewData.shadow_map_2d_data[5].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[5].shadow_map_light_dir`  (       PerViewData.shadow_map_2d_data[6].uv_minp  (       PerViewData.shadow_map_2d_data[6].uv_maxx  6       PerViewData.shadow_map_2d_data[6].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[6].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[7].uv_min�  (       PerViewData.shadow_map_2d_data[7].uv_max�  6       PerViewData.shadow_map_2d_data[7].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[7].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[8].uv_min0  (       PerViewData.shadow_map_2d_data[8].uv_max8  6       PerViewData.shadow_map_2d_data[8].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[8].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[9].uv_min�  (       PerViewData.shadow_map_2d_data[9].uv_max�  6       PerViewData.shadow_map_2d_data[9].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[9].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[10].uv_min�  )       PerViewData.shadow_map_2d_data[10].uv_max�  7       PerViewData.shadow_map_2d_data[10].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[10].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[11].uv_minP  )       PerViewData.shadow_map_2d_data[11].uv_maxX  7       PerViewData.shadow_map_2d_data[11].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[11].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[12].uv_min�  )       PerViewData.shadow_map_2d_data[12].uv_max�  7       PerViewData.shadow_map_2d_data[12].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[12].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[13].uv_min  )       PerViewData.shadow_map_2d_data[13].uv_max  7       PerViewData.shadow_map_2d_data[13].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[13].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[14].uv_minp  )       PerViewData.shadow_map_2d_data[14].uv_maxx  7       PerViewData.shadow_map_2d_data[14].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[14].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[15].uv_min�  )       PerViewData.shadow_map_2d_data[15].uv_max�  7       PerViewData.shadow_map_2d_data[15].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[15].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[16].uv_min0  )       PerViewData.shadow_map_2d_data[16].uv_max8  7       PerViewData.shadow_map_2d_data[16].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[16].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[17].uv_min�  )       PerViewData.shadow_map_2d_data[17].uv_max�  7       PerViewData.shadow_map_2d_data[17].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[17].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[18].uv_min�  )       PerViewData.shadow_map_2d_data[18].uv_max�  7       PerViewData.shadow_map_2d_data[18].shadow_map_view_proj 	  7       PerViewData.shadow_map_2d_data[18].shadow_map_light_dir@	  )       PerViewData.shadow_map_2d_data[19].uv_minP	  )       PerViewData.shadow_map_2d_data[19].uv_maxX	  7       PerViewData.shadow_map_2d_data[19].shadow_map_view_proj`	  7       PerViewData.shadow_map_2d_data[19].shadow_map_light_dir�	  )       PerViewData.shadow_map_2d_data[20].uv_min�	  )       PerViewData.shadow_map_2d_data[20].uv_max�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_view_proj�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_light_dir 
  )       PerViewData.shadow_map_2d_data[21].uv_min
  )       PerViewData.shadow_map_2d_data[21].uv_max
  7       PerViewData.shadow_map_2d_data[21].shadow_map_view_proj 
  7       PerViewData.shadow_map_2d_data[21].shadow_map_light_dir`
  )       PerViewData.shadow_map_2d_data[22].uv_minp
  )       PerViewData.shadow_map_2d_data[22].uv_maxx
  7       PerViewData.shadow_map_2d_data[22].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[22].shadow_map_light_dir�
  )       PerViewData.shadow_map_2d_data[23].uv_min�
  )       PerViewData.shadow_map_2d_data[23].uv_max�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[24].uv_min0  )       PerViewData.shadow_map_2d_data[24].uv_max8  7       PerViewData.shadow_map_2d_data[24].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[24].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[25].uv_min�  )       PerViewData.shadow_map_2d_data[25].uv_max�  7       PerViewData.shadow_map_2d_data[25].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[25].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[26].uv_min�  )       PerViewData.shadow_map_2d_data[26].uv_max�  7       PerViewData.shadow_map_2d_data[26].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[26].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[27].uv_minP  )       PerViewData.shadow_map_2d_data[27].uv_maxX  7       PerViewData.shadow_map_2d_data[27].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[27].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[28].uv_min�  )       PerViewData.shadow_map_2d_data[28].uv_max�  7       PerViewData.shadow_map_2d_data[28].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[28].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[29].uv_min  )       PerViewData.shadow_map_2d_data[29].uv_max  7       PerViewData.shadow_map_2d_data[29].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[29].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[30].uv_minp  )       PerViewData.shadow_map_2d_data[30].uv_maxx  7       PerViewData.shadow_map_2d_data[30].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[30].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[31].uv_min�  )       PerViewData.shadow_map_2d_data[31].uv_max�  7       PerViewData.shadow_map_2d_data[31].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[31].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[32].uv_min0  )       PerViewData.shadow_map_2d_data[32].uv_max8  7       PerViewData.shadow_map_2d_data[32].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[32].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[33].uv_min�  )       PerViewData.shadow_map_2d_data[33].uv_max�  7       PerViewData.shadow_map_2d_data[33].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[33].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[34].uv_min�  )       PerViewData.shadow_map_2d_data[34].uv_max�  7       PerViewData.shadow_map_2d_data[34].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[34].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[35].uv_minP  )       PerViewData.shadow_map_2d_data[35].uv_maxX  7       PerViewData.shadow_map_2d_data[35].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[35].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[36].uv_min�  )       PerViewData.shadow_map_2d_data[36].uv_max�  7       PerViewData.shadow_map_2d_data[36].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[36].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[37].uv_min  )       PerViewData.shadow_map_2d_data[37].uv_max  7       PerViewData.shadow_map_2d_data[37].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[37].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[38].uv_minp  )       PerViewData.shadow_map_2d_data[38].uv_maxx  7       PerViewData.shadow_map_2d_data[38].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[38].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[39].uv_min�  )       PerViewData.shadow_map_2d_data[39].uv_max�  7       PerViewData.shadow_map_2d_data[39].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[39].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[40].uv_min0  )       PerViewData.shadow_map_2d_data[40].uv_max8  7       PerViewData.shadow_map_2d_data[40].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[40].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[41].uv_min�  )       PerViewData.shadow_map_2d_data[41].uv_max�  7       PerViewData.shadow_map_2d_data[41].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[41].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[42].uv_min�  )       PerViewData.shadow_map_2d_data[42].uv_max�  7       PerViewData.shadow_map_2d_data[42].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[42].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[43].uv_minP  )       PerViewData.shadow_map_2d_data[43].uv_maxX  7       PerViewData.shadow_map_2d_data[43].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[43].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[44].uv_min�  )       PerViewData.shadow_map_2d_data[44].uv_max�  7       PerViewData.shadow_map_2d_data[44].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[44].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[45].uv_min  )       PerViewData.shadow_map_2d_data[45].uv_max  7       PerViewData.shadow_map_2d_data[45].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[45].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[46].uv_minp  )       PerViewData.shadow_map_2d_data[46].uv_maxx  7       PerViewData.shadow_map_2d_data[46].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[46].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[47].uv_min�  )       PerViewData.shadow_map_2d_data[47].uv_max�  7       PerViewData.shadow_map_2d_data[47].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[47].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[48].uv_min0  )       PerViewData.shadow_map_2d_data[48].uv_max8  7       PerViewData.shadow_map_2d_data[48].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[48].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[49].uv_min�  )       PerViewData.shadow_map_2d_data[49].uv_max�  7       PerViewData.shadow_map_2d_data[49].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[49].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[50].uv_min�  )       PerViewData.shadow_map_2d_data[50].uv_max�  7       PerViewData.shadow_map_2d_data[50].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[50].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[51].uv_minP  )       PerViewData.shadow_map_2d_data[51].uv_maxX  7       PerViewData.shadow_map_2d_data[51].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[51].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[52].uv_min�  )       PerViewData.shadow_map_2d_data[52].uv_max�  7       PerViewData.shadow_map_2d_data[52].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[52].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[53].uv_min  )       PerViewData.shadow_map_2d_data[53].uv_max  7       PerViewData.shadow_map_2d_data[53].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[53].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[54].uv_minp  )       PerViewData.shadow_map_2d_data[54].uv_maxx  7       PerViewData.shadow_map_2d_data[54].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[54].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[55].uv_min�  )       PerViewData.shadow_map_2d_data[55].uv_max�  7       PerViewData.shadow_map_2d_data[55].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[55].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[56].uv_min0  )       PerViewData.shadow_map_2d_data[56].uv_max8  7       PerViewData.shadow_map_2d_data[56].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[56].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[57].uv_min�  )       PerViewData.shadow_map_2d_data[57].uv_max�  7       PerViewData.shadow_map_2d_data[57].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[57].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[58].uv_min�  )       PerViewData.shadow_map_2d_data[58].uv_max�  7       PerViewData.shadow_map_2d_data[58].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[58].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[59].uv_minP  )       PerViewData.shadow_map_2d_data[59].uv_maxX  7       PerViewData.shadow_map_2d_data[59].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[59].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[60].uv_min�  )       PerViewData.shadow_map_2d_data[60].uv_max�  7       PerViewData.shadow_map_2d_data[60].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[60].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[61].uv_min  )       PerViewData.shadow_map_2d_data[61].uv_max  7       PerViewData.shadow_map_2d_data[61].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[61].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[62].uv_minp  )       PerViewData.shadow_map_2d_data[62].uv_maxx  7       PerViewData.shadow_map_2d_data[62].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[62].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[63].uv_min�  )       PerViewData.shadow_map_2d_data[63].uv_max�  7       PerViewData.shadow_map_2d_data[63].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[63].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[64].uv_min0  )       PerViewData.shadow_map_2d_data[64].uv_max8  7       PerViewData.shadow_map_2d_data[64].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[64].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[65].uv_min�  )       PerViewData.shadow_map_2d_data[65].uv_max�  7       PerViewData.shadow_map_2d_data[65].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[65].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[66].uv_min�  )       PerViewData.shadow_map_2d_data[66].uv_max�  7       PerViewData.shadow_map_2d_data[66].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[66].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[67].uv_minP  )       PerViewData.shadow_map_2d_data[67].uv_maxX  7       PerViewData.shadow_map_2d_data[67].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[67].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[68].uv_min�  )       PerViewData.shadow_map_2d_data[68].uv_max�  7       PerViewData.shadow_map_2d_data[68].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[68].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[69].uv_min  )       PerViewData.shadow_map_2d_data[69].uv_max  7       PerViewData.shadow_map_2d_data[69].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[69].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[70].uv_minp  )       PerViewData.shadow_map_2d_data[70].uv_maxx  7       PerViewData.shadow_map_2d_data[70].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[70].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[71].uv_min�  )       PerViewData.shadow_map_2d_data[71].uv_max�  7       PerViewData.shadow_map_2d_data[71].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[71].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[72].uv_min0  )       PerViewData.shadow_map_2d_data[72].uv_max8  7       PerViewData.shadow_map_2d_data[72].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[72].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[73].uv_min�  )       PerViewData.shadow_map_2d_data[73].uv_max�  7       PerViewData.shadow_map_2d_data[73].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[73].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[74].uv_min�  )       PerViewData.shadow_map_2d_data[74].uv_max�  7       PerViewData.shadow_map_2d_data[74].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[74].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[75].uv_minP  )       PerViewData.shadow_map_2d_data[75].uv_maxX  7       PerViewData.shadow_map_2d_data[75].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[75].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[76].uv_min�  )       PerViewData.shadow_map_2d_data[76].uv_max�  7       PerViewData.shadow_map_2d_data[76].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[76].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[77].uv_min  )       PerViewData.shadow_map_2d_data[77].uv_max  7       PerViewData.shadow_map_2d_data[77].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[77].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[78].uv_minp  )       PerViewData.shadow_map_2d_data[78].uv_maxx  7       PerViewData.shadow_map_2d_data[78].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[78].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[79].uv_min�  )       PerViewData.shadow_map_2d_data[79].uv_max�  7       PerViewData.shadow_map_2d_data[79].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[79].shadow_map_light_dir    )       PerViewData.shadow_map_2d_data[80].uv_min0   )       PerViewData.shadow_map_2d_data[80].uv_max8   7       PerViewData.shadow_map_2d_data[80].shadow_map_view_proj@   7       PerViewData.shadow_map_2d_data[80].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[81].uv_min�   )       PerViewData.shadow_map_2d_data[81].uv_max�   7       PerViewData.shadow_map_2d_data[81].shadow_map_view_proj�   7       PerViewData.shadow_map_2d_data[81].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[82].uv_min�   )       PerViewData.shadow_map_2d_data[82].uv_max�   7       PerViewData.shadow_map_2d_data[82].shadow_map_view_proj !  7       PerViewData.shadow_map_2d_data[82].shadow_map_light_dir@!  )       PerViewData.shadow_map_2d_data[83].uv_minP!  )       PerViewData.shadow_map_2d_data[83].uv_maxX!  7       PerViewData.shadow_map_2d_data[83].shadow_map_view_proj`!  7       PerViewData.shadow_map_2d_data[83].shadow_map_light_dir�!  )       PerViewData.shadow_map_2d_data[84].uv_min�!  )       PerViewData.shadow_map_2d_data[84].uv_max�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_view_proj�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_light_dir "  )       PerViewData.shadow_map_2d_data[85].uv_min"  )       PerViewData.shadow_map_2d_data[85].uv_max"  7       PerViewData.shadow_map_2d_data[85].shadow_map_view_proj "  7       PerViewData.shadow_map_2d_data[85].shadow_map_light_dir`"  )       PerViewData.shadow_map_2d_data[86].uv_minp"  )       PerViewData.shadow_map_2d_data[86].uv_maxx"  7       PerViewData.shadow_map_2d_data[86].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[86].shadow_map_light_dir�"  )       PerViewData.shadow_map_2d_data[87].uv_min�"  )       PerViewData.shadow_map_2d_data[87].uv_max�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_light_dir #  )       PerViewData.shadow_map_2d_data[88].uv_min0#  )       PerViewData.shadow_map_2d_data[88].uv_max8#  7       PerViewData.shadow_map_2d_data[88].shadow_map_view_proj@#  7       PerViewData.shadow_map_2d_data[88].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[89].uv_min�#  )       PerViewData.shadow_map_2d_data[89].uv_max�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_view_proj�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[90].uv_min�#  )       PerViewData.shadow_map_2d_data[90].uv_max�#  7       PerViewData.shadow_map_2d_data[90].shadow_map_view_proj $  7       PerViewData.shadow_map_2d_data[90].shadow_map_light_dir@$  )       PerViewData.shadow_map_2d_data[91].uv_minP$  )       PerViewData.shadow_map_2d_data[91].uv_maxX$  7       PerViewData.shadow_map_2d_data[91].shadow_map_view_proj`$  7       PerViewData.shadow_map_2d_data[91].shadow_map_light_dir�$  )       PerViewData.shadow_map_2d_data[92].uv_min�$  )       PerViewData.shadow_map_2d_data[92].uv_max�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_view_proj�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_light_dir %  )       PerViewData.shadow_map_2d_data[93].uv_min%  )       PerViewData.shadow_map_2d_data[93].uv_max%  7       PerViewData.shadow_map_2d_data[93].shadow_map_view_proj %  7       PerViewData.shadow_map_2d_data[93].shadow_map_light_dir`%  )       PerViewData.shadow_map_2d_data[94].uv_minp%  )       PerViewData.shadow_map_2d_data[94].uv_maxx%  7       PerViewData.shadow_map_2d_data[94].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[94].shadow_map_light_dir�%  )       PerViewData.shadow_map_2d_data[95].uv_min�%  )       PerViewData.shadow_map_2d_data[95].uv_max�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_light_dir &  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[0]0&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[1]@&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[2]P&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[3]`&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[4]p&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[0].cube_map_projection_near_z�&  =       PerViewData.shadow_map_cube_data[0].cube_map_projection_far_z�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[0]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[1]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[2]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[3]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[4]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[1].cube_map_projection_near_z '  =       PerViewData.shadow_map_cube_data[1].cube_map_projection_far_z'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[0]'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[1] '  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[2]0'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[3]@'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[4]P'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[5]`'  >       PerViewData.shadow_map_cube_data[2].cube_map_projection_near_zp'  =       PerViewData.shadow_map_cube_data[2].cube_map_projection_far_zt'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[1]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[2]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[3]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[4]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[5]�'  >       PerViewData.shadow_map_cube_data[3].cube_map_projection_near_z�'  =       PerViewData.shadow_map_cube_data[3].cube_map_projection_far_z�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[1] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[2](  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[3] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[4]0(  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[5]@(  >       PerViewData.shadow_map_cube_data[4].cube_map_projection_near_zP(  =       PerViewData.shadow_map_cube_data[4].cube_map_projection_far_zT(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[0]`(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[1]p(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[3]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[4]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[5]�(  >       PerViewData.shadow_map_cube_data[5].cube_map_projection_near_z�(  =       PerViewData.shadow_map_cube_data[5].cube_map_projection_far_z�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[0]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[1]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[3] )  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[4])  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[5] )  >       PerViewData.shadow_map_cube_data[6].cube_map_projection_near_z0)  =       PerViewData.shadow_map_cube_data[6].cube_map_projection_far_z4)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[0]@)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[1]P)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[2]`)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[3]p)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[5]�)  >       PerViewData.shadow_map_cube_data[7].cube_map_projection_near_z�)  =       PerViewData.shadow_map_cube_data[7].cube_map_projection_far_z�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[0]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[1]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[2]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[3]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[5] *  >       PerViewData.shadow_map_cube_data[8].cube_map_projection_near_z*  =       PerViewData.shadow_map_cube_data[8].cube_map_projection_far_z*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[0] *  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[1]0*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[2]@*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[3]P*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[4]`*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[5]p*  >       PerViewData.shadow_map_cube_data[9].cube_map_projection_near_z�*  =       PerViewData.shadow_map_cube_data[9].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[0]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[1]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[2]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[3]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[4]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[5]�*  ?       PerViewData.shadow_map_cube_data[10].cube_map_projection_near_z�*  >       PerViewData.shadow_map_cube_data[10].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[0] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[1]+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[2] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[3]0+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[4]@+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[5]P+  ?       PerViewData.shadow_map_cube_data[11].cube_map_projection_near_z`+  >       PerViewData.shadow_map_cube_data[11].cube_map_projection_far_zd+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[0]p+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[2]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[3]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[4]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[5]�+  ?       PerViewData.shadow_map_cube_data[12].cube_map_projection_near_z�+  >       PerViewData.shadow_map_cube_data[12].cube_map_projection_far_z�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[0]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[2] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[3],  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[4] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[5]0,  ?       PerViewData.shadow_map_cube_data[13].cube_map_projection_near_z@,  >       PerViewData.shadow_map_cube_data[13].cube_map_projection_far_zD,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[0]P,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[1]`,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[2]p,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[4]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[5]�,  ?       PerViewData.shadow_map_cube_data[14].cube_map_projection_near_z�,  >       PerViewData.shadow_map_cube_data[14].cube_map_projection_far_z�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[0]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[1]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[2]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[4] -  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[5]-  ?       PerViewData.shadow_map_cube_data[15].cube_map_projection_near_z -  >       PerViewData.shadow_map_cube_data[15].cube_map_projection_far_z$-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[0]0-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[1]@-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[2]P-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[3]`-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[4]p-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[16].cube_map_projection_near_z�-  >       PerViewData.shadow_map_cube_data[16].cube_map_projection_far_z�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[0]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[1]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[2]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[3]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[4]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[17].cube_map_projection_near_z .  >       PerViewData.shadow_map_cube_data[17].cube_map_projection_far_z.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[0].  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[1] .  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[2]0.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[3]@.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[4]P.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[5]`.  ?       PerViewData.shadow_map_cube_data[18].cube_map_projection_near_zp.  >       PerViewData.shadow_map_cube_data[18].cube_map_projection_far_zt.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[1]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[2]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[3]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[4]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[5]�.  ?       PerViewData.shadow_map_cube_data[19].cube_map_projection_near_z�.  >       PerViewData.shadow_map_cube_data[19].cube_map_projection_far_z�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[1] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[2]/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[3] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[4]0/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[5]@/  ?       PerViewData.shadow_map_cube_data[20].cube_map_projection_near_zP/  >       PerViewData.shadow_map_cube_data[20].cube_map_projection_far_zT/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[0]`/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[1]p/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[3]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[4]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[5]�/  ?       PerViewData.shadow_map_cube_data[21].cube_map_projection_near_z�/  >       PerViewData.shadow_map_cube_data[21].cube_map_projection_far_z�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[0]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[1]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[3] 0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[4]0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[5] 0  ?       PerViewData.shadow_map_cube_data[22].cube_map_projection_near_z00  >       PerViewData.shadow_map_cube_data[22].cube_map_projection_far_z40  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[0]@0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[1]P0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[2]`0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[3]p0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[5]�0  ?       PerViewData.shadow_map_cube_data[23].cube_map_projection_near_z�0  >       PerViewData.shadow_map_cube_data[23].cube_map_projection_far_z�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[0]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[1]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[2]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[3]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[5] 1  ?       PerViewData.shadow_map_cube_data[24].cube_map_projection_near_z1  >       PerViewData.shadow_map_cube_data[24].cube_map_projection_far_z1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[0] 1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[1]01  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[2]@1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[3]P1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[4]`1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[5]p1  ?       PerViewData.shadow_map_cube_data[25].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[25].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[0]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[1]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[2]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[3]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[4]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[5]�1  ?       PerViewData.shadow_map_cube_data[26].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[26].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[0] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[1]2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[2] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[3]02  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[4]@2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[5]P2  ?       PerViewData.shadow_map_cube_data[27].cube_map_projection_near_z`2  >       PerViewData.shadow_map_cube_data[27].cube_map_projection_far_zd2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[0]p2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[2]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[3]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[4]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[5]�2  ?       PerViewData.shadow_map_cube_data[28].cube_map_projection_near_z�2  >       PerViewData.shadow_map_cube_data[28].cube_map_projection_far_z�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[0]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[2] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[3]3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[4] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[5]03  ?       PerViewData.shadow_map_cube_data[29].cube_map_projection_near_z@3  >       PerViewData.shadow_map_cube_data[29].cube_map_projection_far_zD3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[0]P3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[1]`3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[2]p3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[4]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[5]�3  ?       PerViewData.shadow_map_cube_data[30].cube_map_projection_near_z�3  >       PerViewData.shadow_map_cube_data[30].cube_map_projection_far_z�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[0]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[1]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[2]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[4] 4  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[5]4  ?       PerViewData.shadow_map_cube_data[31].cube_map_projection_near_z 4  >       PerViewData.shadow_map_cube_data[31].cube_map_projection_far_z$4                             smp              smp                                    smp_depth_linear              smp_depth_linear                                    smp_depth_nearest              smp_depth_nearest                                    shadow_map_atlas              shadow_map_atlas                                     LightBinOutput              light_bin_output                              	       AllLights       
       all_lights                                    ssao_texture              ssao_texture                                     AllTransforms              all_transforms                                    AllDrawData             all_draw_data                                     AllMaterials              all_materials                                   all_material_textures             all_material_textures                            main              �                            PerViewData               PerViewData �             PerViewData.view           PerViewData.view_proj@          PerViewData.ambient_light�          PerViewData.jitter_amount�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.mip_bias�          PerViewData.ndf_filter_amount�   #       PerViewData.directional_light_count�   "       PerViewData.use_clustered_lighting�   .       PerViewData.directional_lights[0].direction_ws�   +       PerViewData.directional_lights[0].intensity�   '       PerViewData.directional_lights[0].color�   .       PerViewData.directional_lights[0].direction_vs�   ,       PerViewData.directional_lights[0].shadow_map�   .       PerViewData.directional_lights[1].direction_ws�   +       PerViewData.directional_lights[1].intensity�   '       PerViewData.directional_lights[1].color�   .       PerViewData.directional_lights[1].direction_vs   ,       PerViewData.directional_lights[1].shadow_map  .       PerViewData.directional_lights[2].direction_ws  +       PerViewData.directional_lights[2].intensity  '       PerViewData.directional_lights[2].color   .       PerViewData.directional_lights[2].direction_vs0  ,       PerViewData.directional_lights[2].shadow_map<  .       PerViewData.directional_lights[3].direction_ws@  +       PerViewData.directional_lights[3].intensityL  '       PerViewData.directional_lights[3].colorP  .       PerViewData.directional_lights[3].direction_vs`  ,       PerViewData.directional_lights[3].shadow_mapl  .       PerViewData.directional_lights[4].direction_wsp  +       PerViewData.directional_lights[4].intensity|  '       PerViewData.directional_lights[4].color�  .       PerViewData.directional_lights[4].direction_vs�  ,       PerViewData.directional_lights[4].shadow_map�  .       PerViewData.directional_lights[5].direction_ws�  +       PerViewData.directional_lights[5].intensity�  '       PerViewData.directional_lights[5].color�  .       PerViewData.directional_lights[5].direction_vs�  ,       PerViewData.directional_lights[5].shadow_map�  .       PerViewData.directional_lights[6].direction_ws�  +       PerViewData.directional_lights[6].intensity�  '       PerViewData.directional_lights[6].color�  .       PerViewData.directional_lights[6].direction_vs�  ,       PerViewData.directional_lights[6].shadow_map�  .       PerViewData.directional_lights[7].direction_ws   +       PerViewData.directional_lights[7].intensity  '       PerViewData.directional_lights[7].color  .       PerViewData.directional_lights[7].direction_vs   ,       PerViewData.directional_lights[7].shadow_map,  (       PerViewData.shadow_map_2d_data[0].uv_min0  (       PerViewData.shadow_map_2d_data[0].uv_max8  6       PerViewData.shadow_map_2d_data[0].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[0].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[1].uv_min�  (       PerViewData.shadow_map_2d_data[1].uv_max�  6       PerViewData.shadow_map_2d_data[1].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[1].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[2].uv_min�  (       PerViewData.shadow_map_2d_data[2].uv_max�  6       PerViewData.shadow_map_2d_data[2].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[2].shadow_map_light_dir@  (       PerViewData.shadow_map_2d_data[3].uv_minP  (       PerViewData.shadow_map_2d_data[3].uv_maxX  6       PerViewData.shadow_map_2d_data[3].shadow_map_view_proj`  6       PerViewData.shadow_map_2d_data[3].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[4].uv_min�  (       PerViewData.shadow_map_2d_data[4].uv_max�  6       PerViewData.shadow_map_2d_data[4].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[4].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[5].uv_min  (       PerViewData.shadow_map_2d_data[5].uv_max  6       PerViewData.shadow_map_2d_data[5].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[5].shadow_map_light_dir`  (       PerViewData.shadow_map_2d_data[6].uv_minp  (       PerViewData.shadow_map_2d_data[6].uv_maxx  6       PerViewData.shadow_map_2d_data[6].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[6].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[7].uv_min�  (       PerViewData.shadow_map_2d_data[7].uv_max�  6       PerViewData.shadow_map_2d_data[7].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[7].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[8].uv_min0  (       PerViewData.shadow_map_2d_data[8].uv_max8  6       PerViewData.shadow_map_2d_data[8].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[8].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[9].uv_min�  (       PerViewData.shadow_map_2d_data[9].uv_max�  6       PerViewData.shadow_map_2d_data[9].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[9].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[10].uv_min�  )       PerViewData.shadow_map_2d_data[10].uv_max�  7       PerViewData.shadow_map_2d_data[10].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[10].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[11].uv_minP  )       PerViewData.shadow_map_2d_data[11].uv_maxX  7       PerViewData.shadow_map_2d_data[11].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[11].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[12].uv_min�  )       PerViewData.shadow_map_2d_data[12].uv_max�  7       PerViewData.shadow_map_2d_data[12].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[12].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[13].uv_min  )       PerViewData.shadow_map_2d_data[13].uv_max  7       PerViewData.shadow_map_2d_data[13].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[13].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[14].uv_minp  )       PerViewData.shadow_map_2d_data[14].uv_maxx  7       PerViewData.shadow_map_2d_data[14].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[14].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[15].uv_min�  )       PerViewData.shadow_map_2d_data[15].uv_max�  7       PerViewData.shadow_map_2d_data[15].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[15].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[16].uv_min0  )       PerViewData.shadow_map_2d_data[16].uv_max8  7       PerViewData.shadow_map_2d_data[16].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[16].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[17].uv_min�  )       PerViewData.shadow_map_2d_data[17].uv_max�  7       PerViewData.shadow_map_2d_data[17].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[17].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[18].uv_min�  )       PerViewData.shadow_map_2d_data[18].uv_max�  7       PerViewData.shadow_map_2d_data[18].shadow_map_view_proj 	  7       PerViewData.shadow_map_2d_data[18].shadow_map_light_dir@	  )       PerViewData.shadow_map_2d_data[19].uv_minP	  )       PerViewData.shadow_map_2d_data[19].uv_maxX	  7       PerViewData.shadow_map_2d_data[19].shadow_map_view_proj`	  7       PerViewData.shadow_map_2d_data[19].shadow_map_light_dir�	  )       PerViewData.shadow_map_2d_data[20].uv_min�	  )       PerViewData.shadow_map_2d_data[20].uv_max�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_view_proj�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_light_dir 
  )       PerViewData.shadow_map_2d_data[21].uv_min
  )       PerViewData.shadow_map_2d_data[21].uv_max
  7       PerViewData.shadow_map_2d_data[21].shadow_map_view_proj 
  7       PerViewData.shadow_map_2d_data[21].shadow_map_light_dir`
  )       PerViewData.shadow_map_2d_data[22].uv_minp
  )       PerViewData.shadow_map_2d_data[22].uv_maxx
  7       PerViewData.shadow_map_2d_data[22].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[22].shadow_map_light_dir�
  )       PerViewData.shadow_map_2d_data[23].uv_min�
  )       PerViewData.shadow_map_2d_data[23].uv_max�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[24].uv_min0  )       PerViewData.shadow_map_2d_data[24].uv_max8  7       PerViewData.shadow_map_2d_data[24].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[24].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[25].uv_min�  )       PerViewData.shadow_map_2d_data[25].uv_max�  7       PerViewData.shadow_map_2d_data[25].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[25].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[26].uv_min�  )       PerViewData.shadow_map_2d_data[26].uv_max�  7       PerViewData.shadow_map_2d_data[26].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[26].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[27].uv_minP  )       PerViewData.shadow_map_2d_data[27].uv_maxX  7       PerViewData.shadow_map_2d_data[27].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[27].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[28].uv_min�  )       PerViewData.shadow_map_2d_data[28].uv_max�  7       PerViewData.shadow_map_2d_data[28].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[28].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[29].uv_min  )       PerViewData.shadow_map_2d_data[29].uv_max  7       PerViewData.shadow_map_2d_data[29].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[29].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[30].uv_minp  )       PerViewData.shadow_map_2d_data[30].uv_maxx  7       PerViewData.shadow_map_2d_data[30].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[30].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[31].uv_min�  )       PerViewData.shadow_map_2d_data[31].uv_max�  7       PerViewData.shadow_map_2d_data[31].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[31].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[32].uv_min0  )       PerViewData.shadow_map_2d_data[32].uv_max8  7       PerViewData.shadow_map_2d_data[32].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[32].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[33].uv_min�  )       PerViewData.shadow_map_2d_data[33].uv_max�  7       PerViewData.shadow_map_2d_data[33].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[33].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[34].uv_min�  )       PerViewData.shadow_map_2d_data[34].uv_max�  7       PerViewData.shadow_map_2d_data[34].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[34].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[35].uv_minP  )       PerViewData.shadow_map_2d_data[35].uv_maxX  7       PerViewData.shadow_map_2d_data[35].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[35].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[36].uv_min�  )       PerViewData.shadow_map_2d_data[36].uv_max�  7       PerViewData.shadow_map_2d_data[36].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[36].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[37].uv_min  )       PerViewData.shadow_map_2d_data[37].uv_max  7       PerViewData.shadow_map_2d_data[37].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[37].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[38].uv_minp  )       PerViewData.shadow_map_2d_data[38].uv_maxx  7       PerViewData.shadow_map_2d_data[38].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[38].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[39].uv_min�  )       PerViewData.shadow_map_2d_data[39].uv_max�  7       PerViewData.shadow_map_2d_data[39].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[39].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[40].uv_min0  )       PerViewData.shadow_map_2d_data[40].uv_max8  7       PerViewData.shadow_map_2d_data[40].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[40].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[41].uv_min�  )       PerViewData.shadow_map_2d_data[41].uv_max�  7       PerViewData.shadow_map_2d_data[41].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[41].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[42].uv_min�  )       PerViewData.shadow_map_2d_data[42].uv_max�  7       PerViewData.shadow_map_2d_data[42].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[42].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[43].uv_minP  )       PerViewData.shadow_map_2d_data[43].uv_maxX  7       PerViewData.shadow_map_2d_data[43].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[43].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[44].uv_min�  )       PerViewData.shadow_map_2d_data[44].uv_max�  7       PerViewData.shadow_map_2d_data[44].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[44].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[45].uv_min  )       PerViewData.shadow_map_2d_data[45].uv_max  7       PerViewData.shadow_map_2d_data[45].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[45].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[46].uv_minp  )       PerViewData.shadow_map_2d_data[46].uv_maxx  7       PerViewData.shadow_map_2d_data[46].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[46].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[47].uv_min�  )       PerViewData.shadow_map_2d_data[47].uv_max�  7       PerViewData.shadow_map_2d_data[47].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[47].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[48].uv_min0  )       PerViewData.shadow_map_2d_data[48].uv_max8  7       PerViewData.shadow_map_2d_data[48].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[48].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[49].uv_min�  )       PerViewData.shadow_map_2d_data[49].uv_max�  7       PerViewData.shadow_map_2d_data[49].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[49].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[50].uv_min�  )       PerViewData.shadow_map_2d_data[50].uv_max�  7       PerViewData.shadow_map_2d_data[50].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[50].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[51].uv_minP  )       PerViewData.shadow_map_2d_data[51].uv_maxX  7       PerViewData.shadow_map_2d_data[51].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[51].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[52].uv_min�  )       PerViewData.shadow_map_2d_data[52].uv_max�  7       PerViewData.shadow_map_2d_data[52].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[52].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[53].uv_min  )       PerViewData.shadow_map_2d_data[53].uv_max  7       PerViewData.shadow_map_2d_data[53].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[53].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[54].uv_minp  )       PerViewData.shadow_map_2d_data[54].uv_maxx  7       PerViewData.shadow_map_2d_data[54].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[54].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[55].uv_min�  )       PerViewData.shadow_map_2d_data[55].uv_max�  7       PerViewData.shadow_map_2d_data[55].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[55].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[56].uv_min0  )       PerViewData.shadow_map_2d_data[56].uv_max8  7       PerViewData.shadow_map_2d_data[56].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[56].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[57].uv_min�  )       PerViewData.shadow_map_2d_data[57].uv_max�  7       PerViewData.shadow_map_2d_data[57].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[57].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[58].uv_min�  )       PerViewData.shadow_map_2d_data[58].uv_max�  7       PerViewData.shadow_map_2d_data[58].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[58].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[59].uv_minP  )       PerViewData.shadow_map_2d_data[59].uv_maxX  7       PerViewData.shadow_map_2d_data[59].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[59].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[60].uv_min�  )       PerViewData.shadow_map_2d_data[60].uv_max�  7       PerViewData.shadow_map_2d_data[60].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[60].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[61].uv_min  )       PerViewData.shadow_map_2d_data[61].uv_max  7       PerViewData.shadow_map_2d_data[61].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[61].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[62].uv_minp  )       PerViewData.shadow_map_2d_data[62].uv_maxx  7       PerViewData.shadow_map_2d_data[62].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[62].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[63].uv_min�  )       PerViewData.shadow_map_2d_data[63].uv_max�  7       PerViewData.shadow_map_2d_data[63].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[63].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[64].uv_min0  )       PerViewData.shadow_map_2d_data[64].uv_max8  7       PerViewData.shadow_map_2d_data[64].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[64].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[65].uv_min�  )       PerViewData.shadow_map_2d_data[65].uv_max�  7       PerViewData.shadow_map_2d_data[65].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[65].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[66].uv_min�  )       PerViewData.shadow_map_2d_data[66].uv_max�  7       PerViewData.shadow_map_2d_data[66].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[66].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[67].uv_minP  )       PerViewData.shadow_map_2d_data[67].uv_maxX  7       PerViewData.shadow_map_2d_data[67].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[67].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[68].uv_min�  )       PerViewData.shadow_map_2d_data[68].uv_max�  7       PerViewData.shadow_map_2d_data[68].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[68].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[69].uv_min  )       PerViewData.shadow_map_2d_data[69].uv_max  7       PerViewData.shadow_map_2d_data[69].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[69].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[70].uv_minp  )       PerViewData.shadow_map_2d_data[70].uv_maxx  7       PerViewData.shadow_map_2d_data[70].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[70].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[71].uv_min�  )       PerViewData.shadow_map_2d_data[71].uv_max�  7       PerViewData.shadow_map_2d_data[71].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[71].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[72].uv_min0  )       PerViewData.shadow_map_2d_data[72].uv_max8  7       PerViewData.shadow_map_2d_data[72].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[72].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[73].uv_min�  )       PerViewData.shadow_map_2d_data[73].uv_max�  7       PerViewData.shadow_map_2d_data[73].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[73].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[74].uv_min�  )       PerViewData.shadow_map_2d_data[74].uv_max�  7       PerViewData.shadow_map_2d_data[74].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[74].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[75].uv_minP  )       PerViewData.shadow_map_2d_data[75].uv_maxX  7       PerViewData.shadow_map_2d_data[75].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[75].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[76].uv_min�  )       PerViewData.shadow_map_2d_data[76].uv_max�  7       PerViewData.shadow_map_2d_data[76].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[76].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[77].uv_min  )       PerViewData.shadow_map_2d_data[77].uv_max  7       PerViewData.shadow_map_2d_data[77].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[77].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[78].uv_minp  )       PerViewData.shadow_map_2d_data[78].uv_maxx  7       PerViewData.shadow_map_2d_data[78].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[78].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[79].uv_min�  )       PerViewData.shadow_map_2d_data[79].uv_max�  7       PerViewData.shadow_map_2d_data[79].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[79].shadow_map_light_dir    )       PerViewData.shadow_map_2d_data[80].uv_min0   )       PerViewData.shadow_map_2d_data[80].uv_max8   7       PerViewData.shadow_map_2d_data[80].shadow_map_view_proj@   7       PerViewData.shadow_map_2d_data[80].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[81].uv_min�   )       PerViewData.shadow_map_2d_data[81].uv_max�   7       PerViewData.shadow_map_2d_data[81].shadow_map_view_proj�   7       PerViewData.shadow_map_2d_data[81].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[82].uv_min�   )       PerViewData.shadow_map_2d_data[82].uv_max�   7       PerViewData.shadow_map_2d_data[82].shadow_map_view_proj !  7       PerViewData.shadow_map_2d_data[82].shadow_map_light_dir@!  )       PerViewData.shadow_map_2d_data[83].uv_minP!  )       PerViewData.shadow_map_2d_data[83].uv_maxX!  7       PerViewData.shadow_map_2d_data[83].shadow_map_view_proj`!  7       PerViewData.shadow_map_2d_data[83].shadow_map_light_dir�!  )       PerViewData.shadow_map_2d_data[84].uv_min�!  )       PerViewData.shadow_map_2d_data[84].uv_max�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_view_proj�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_light_dir "  )       PerViewData.shadow_map_2d_data[85].uv_min"  )       PerViewData.shadow_map_2d_data[85].uv_max"  7       PerViewData.shadow_map_2d_data[85].shadow_map_view_proj "  7       PerViewData.shadow_map_2d_data[85].shadow_map_light_dir`"  )       PerViewData.shadow_map_2d_data[86].uv_minp"  )       PerViewData.shadow_map_2d_data[86].uv_maxx"  7       PerViewData.shadow_map_2d_data[86].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[86].shadow_map_light_dir�"  )       PerViewData.shadow_map_2d_data[87].uv_min�"  )       PerViewData.shadow_map_2d_data[87].uv_max�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_light_dir #  )       PerViewData.shadow_map_2d_data[88].uv_min0#  )       PerViewData.shadow_map_2d_data[88].uv_max8#  7       PerViewData.shadow_map_2d_data[88].shadow_map_view_proj@#  7       PerViewData.shadow_map_2d_data[88].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[89].uv_min�#  )       PerViewData.shadow_map_2d_data[89].uv_max�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_view_proj�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[90].uv_min�#  )       PerViewData.shadow_map_2d_data[90].uv_max�#  7       PerViewData.shadow_map_2d_data[90].shadow_map_view_proj $  7       PerViewData.shadow_map_2d_data[90].shadow_map_light_dir@$  )       PerViewData.shadow_map_2d_data[91].uv_minP$  )       PerViewData.shadow_map_2d_data[91].uv_maxX$  7       PerViewData.shadow_map_2d_data[91].shadow_map_view_proj`$  7       PerViewData.shadow_map_2d_data[91].shadow_map_light_dir�$  )       PerViewData.shadow_map_2d_data[92].uv_min�$  )       PerViewData.shadow_map_2d_data[92].uv_max�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_view_proj�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_light_dir %  )       PerViewData.shadow_map_2d_data[93].uv_min%  )       PerViewData.shadow_map_2d_data[93].uv_max%  7       PerViewData.shadow_map_2d_data[93].shadow_map_view_proj %  7       PerViewData.shadow_map_2d_data[93].shadow_map_light_dir`%  )       PerViewData.shadow_map_2d_data[94].uv_minp%  )       PerViewData.shadow_map_2d_data[94].uv_maxx%  7       PerViewData.shadow_map_2d_data[94].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[94].shadow_map_light_dir�%  )       PerViewData.shadow_map_2d_data[95].uv_min�%  )       PerViewData.shadow_map_2d_data[95].uv_max�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_light_dir &  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[0]0&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[1]@&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[2]P&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[3]`&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[4]p&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[0].cube_map_projection_near_z�&  =       PerViewData.shadow_map_cube_data[0].cube_map_projection_far_z�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[0]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[1]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[2]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[3]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[4]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[1].cube_map_projection_near_z '  =       PerViewData.shadow_map_cube_data[1].cube_map_projection_far_z'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[0]'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[1] '  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[2]0'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[3]@'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[4]P'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[5]`'  >       PerViewData.shadow_map_cube_data[2].cube_map_projection_near_zp'  =       PerViewData.shadow_map_cube_data[2].cube_map_projection_far_zt'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[1]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[2]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[3]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[4]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[5]�'  >       PerViewData.shadow_map_cube_data[3].cube_map_projection_near_z�'  =       PerViewData.shadow_map_cube_data[3].cube_map_projection_far_z�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[1] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[2](  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[3] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[4]0(  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[5]@(  >       PerViewData.shadow_map_cube_data[4].cube_map_projection_near_zP(  =       PerViewData.shadow_map_cube_data[4].cube_map_projection_far_zT(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[0]`(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[1]p(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[3]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[4]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[5]�(  >       PerViewData.shadow_map_cube_data[5].cube_map_projection_near_z�(  =       PerViewData.shadow_map_cube_data[5].cube_map_projection_far_z�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[0]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[1]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[3] )  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[4])  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[5] )  >       PerViewData.shadow_map_cube_data[6].cube_map_projection_near_z0)  =       PerViewData.shadow_map_cube_data[6].cube_map_projection_far_z4)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[0]@)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[1]P)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[2]`)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[3]p)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[5]�)  >       PerViewData.shadow_map_cube_data[7].cube_map_projection_near_z�)  =       PerViewData.shadow_map_cube_data[7].cube_map_projection_far_z�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[0]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[1]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[2]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[3]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[5] *  >       PerViewData.shadow_map_cube_data[8].cube_map_projection_near_z*  =       PerViewData.shadow_map_cube_data[8].cube_map_projection_far_z*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[0] *  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[1]0*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[2]@*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[3]P*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[4]`*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[5]p*  >       PerViewData.shadow_map_cube_data[9].cube_map_projection_near_z�*  =       PerViewData.shadow_map_cube_data[9].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[0]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[1]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[2]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[3]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[4]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[5]�*  ?       PerViewData.shadow_map_cube_data[10].cube_map_projection_near_z�*  >       PerViewData.shadow_map_cube_data[10].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[0] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[1]+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[2] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[3]0+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[4]@+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[5]P+  ?       PerViewData.shadow_map_cube_data[11].cube_map_projection_near_z`+  >       PerViewData.shadow_map_cube_data[11].cube_map_projection_far_zd+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[0]p+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[2]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[3]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[4]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[5]�+  ?       PerViewData.shadow_map_cube_data[12].cube_map_projection_near_z�+  >       PerViewData.shadow_map_cube_data[12].cube_map_projection_far_z�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[0]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[2] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[3],  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[4] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[5]0,  ?       PerViewData.shadow_map_cube_data[13].cube_map_projection_near_z@,  >       PerViewData.shadow_map_cube_data[13].cube_map_projection_far_zD,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[0]P,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[1]`,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[2]p,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[4]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[5]�,  ?       PerViewData.shadow_map_cube_data[14].cube_map_projection_near_z�,  >       PerViewData.shadow_map_cube_data[14].cube_map_projection_far_z�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[0]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[1]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[2]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[4] -  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[5]-  ?       PerViewData.shadow_map_cube_data[15].cube_map_projection_near_z -  >       PerViewData.shadow_map_cube_data[15].cube_map_projection_far_z$-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[0]0-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[1]@-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[2]P-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[3]`-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[4]p-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[16].cube_map_projection_near_z�-  >       PerViewData.shadow_map_cube_data[16].cube_map_projection_far_z�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[0]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[1]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[2]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[3]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[4]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[17].cube_map_projection_near_z .  >       PerViewData.shadow_map_cube_data[17].cube_map_projection_far_z.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[0].  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[1] .  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[2]0.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[3]@.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[4]P.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[5]`.  ?       PerViewData.shadow_map_cube_data[18].cube_map_projection_near_zp.  >       PerViewData.shadow_map_cube_data[18].cube_map_projection_far_zt.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[1]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[2]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[3]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[4]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[5]�.  ?       PerViewData.shadow_map_cube_data[19].cube_map_projection_near_z�.  >       PerViewData.shadow_map_cube_data[19].cube_map_projection_far_z�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[1] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[2]/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[3] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[4]0/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[5]@/  ?       PerViewData.shadow_map_cube_data[20].cube_map_projection_near_zP/  >       PerViewData.shadow_map_cube_data[20].cube_map_projection_far_zT/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[0]`/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[1]p/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[3]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[4]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[5]�/  ?       PerViewData.shadow_map_cube_data[21].cube_map_projection_near_z�/  >       PerViewData.shadow_map_cube_data[21].cube_map_projection_far_z�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[0]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[1]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[3] 0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[4]0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[5] 0  ?       PerViewData.shadow_map_cube_data[22].cube_map_projection_near_z00  >       PerViewData.shadow_map_cube_data[22].cube_map_projection_far_z40  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[0]@0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[1]P0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[2]`0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[3]p0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[5]�0  ?       PerViewData.shadow_map_cube_data[23].cube_map_projection_near_z�0  >       PerViewData.shadow_map_cube_data[23].cube_map_projection_far_z�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[0]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[1]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[2]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[3]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[5] 1  ?       PerViewData.shadow_map_cube_data[24].cube_map_projection_near_z1  >       PerViewData.shadow_map_cube_data[24].cube_map_projection_far_z1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[0] 1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[1]01  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[2]@1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[3]P1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[4]`1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[5]p1  ?       PerViewData.shadow_map_cube_data[25].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[25].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[0]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[1]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[2]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[3]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[4]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[5]�1  ?       PerViewData.shadow_map_cube_data[26].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[26].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[0] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[1]2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[2] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[3]02  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[4]@2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[5]P2  ?       PerViewData.shadow_map_cube_data[27].cube_map_projection_near_z`2  >       PerViewData.shadow_map_cube_data[27].cube_map_projection_far_zd2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[0]p2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[2]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[3]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[4]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[5]�2  ?       PerViewData.shadow_map_cube_data[28].cube_map_projection_near_z�2  >       PerViewData.shadow_map_cube_data[28].cube_map_projection_far_z�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[0]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[2] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[3]3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[4] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[5]03  ?       PerViewData.shadow_map_cube_data[29].cube_map_projection_near_z@3  >       PerViewData.shadow_map_cube_data[29].cube_map_projection_far_zD3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[0]P3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[1]`3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[2]p3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[4]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[5]�3  ?       PerViewData.shadow_map_cube_data[30].cube_map_projection_near_z�3  >       PerViewData.shadow_map_cube_data[30].cube_map_projection_far_z�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[0]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[1]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[2]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[4] 4  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[5]4  ?       PerViewData.shadow_map_cube_data[31].cube_map_projection_near_z 4  >       PerViewData.shadow_map_cube_data[31].cube_map_projection_far_z$4   04                             smp              smp                                        �A                                smp_depth_linear              smp_depth_linear                                        �?                               smp_depth_nearest              smp_depth_nearest                                           �?                               shadow_map_atlas              shadow_map_atlas                                       LightBinOutput              light_bin_output                                	       AllLights       
       all_lights                                             ssao_texture              ssao_texture                                              AllTransforms              all_transforms                                      AllDrawData             all_draw_data                                              AllMaterials              all_materials                                     all_material_textures             all_material_textures                                    �                            PerViewData               PerViewData �             PerViewData.view           PerViewData.view_proj@          PerViewData.ambient_light�          PerViewData.jitter_amount�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.mip_bias�          PerViewData.ndf_filter_amount�   #       PerViewData.directional_light_count�   "       PerViewData.use_clustered_lighting�   .       PerViewData.directional_lights[0].direction_ws�   +       PerViewData.directional_lights[0].intensity�   '       PerViewData.directional_lights[0].color�   .       PerViewData.directional_lights[0].direction_vs�   ,       PerViewData.directional_lights[0].shadow_map�   .       PerViewData.directional_lights[1].direction_ws�   +       PerViewData.directional_lights[1].intensity�   '       PerViewData.directional_lights[1].color�   .       PerViewData.directional_lights[1].direction_vs   ,       PerViewData.directional_lights[1].shadow_map  .       PerViewData.directional_lights[2].direction_ws  +       PerViewData.directional_lights[2].intensity  '       PerViewData.directional_lights[2].color   .       PerViewData.directional_lights[2].direction_vs0  ,       PerViewData.directional_lights[2].shadow_map<  .       PerViewData.directional_lights[3].direction_ws@  +       PerViewData.directional_lights[3].intensityL  '       PerViewData.directional_lights[3].colorP  .       PerViewData.directional_lights[3].direction_vs`  ,       PerViewData.directional_lights[3].shadow_mapl  .       PerViewData.directional_lights[4].direction_wsp  +       PerViewData.directional_lights[4].intensity|  '       PerViewData.directional_lights[4].color�  .       PerViewData.directional_lights[4].direction_vs�  ,       PerViewData.directional_lights[4].shadow_map�  .       PerViewData.directional_lights[5].direction_ws�  +       PerViewData.directional_lights[5].intensity�  '       PerViewData.directional_lights[5].color�  .       PerViewData.directional_lights[5].direction_vs�  ,       PerViewData.directional_lights[5].shadow_map�  .       PerViewData.directional_lights[6].direction_ws�  +       PerViewData.directional_lights[6].intensity�  '       PerViewData.directional_lights[6].color�  .       PerViewData.directional_lights[6].direction_vs�  ,       PerViewData.directional_lights[6].shadow_map�  .       PerViewData.directional_lights[7].direction_ws   +       PerViewData.directional_lights[7].intensity  '       PerViewData.directional_lights[7].color  .       PerViewData.directional_lights[7].direction_vs   ,       PerViewData.directional_lights[7].shadow_map,  (       PerViewData.shadow_map_2d_data[0].uv_min0  (       PerViewData.shadow_map_2d_data[0].uv_max8  6       PerViewData.shadow_map_2d_data[0].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[0].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[1].uv_min�  (       PerViewData.shadow_map_2d_data[1].uv_max�  6       PerViewData.shadow_map_2d_data[1].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[1].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[2].uv_min�  (       PerViewData.shadow_map_2d_data[2].uv_max�  6       PerViewData.shadow_map_2d_data[2].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[2].shadow_map_light_dir@  (       PerViewData.shadow_map_2d_data[3].uv_minP  (       PerViewData.shadow_map_2d_data[3].uv_maxX  6       PerViewData.shadow_map_2d_data[3].shadow_map_view_proj`  6       PerViewData.shadow_map_2d_data[3].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[4].uv_min�  (       PerViewData.shadow_map_2d_data[4].uv_max�  6       PerViewData.shadow_map_2d_data[4].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[4].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[5].uv_min  (       PerViewData.shadow_map_2d_data[5].uv_max  6       PerViewData.shadow_map_2d_data[5].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[5].shadow_map_light_dir`  (       PerViewData.shadow_map_2d_data[6].uv_minp  (       PerViewData.shadow_map_2d_data[6].uv_maxx  6       PerViewData.shadow_map_2d_data[6].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[6].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[7].uv_min�  (       PerViewData.shadow_map_2d_data[7].uv_max�  6       PerViewData.shadow_map_2d_data[7].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[7].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[8].uv_min0  (       PerViewData.shadow_map_2d_data[8].uv_max8  6       PerViewData.shadow_map_2d_data[8].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[8].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[9].uv_min�  (       PerViewData.shadow_map_2d_data[9].uv_max�  6       PerViewData.shadow_map_2d_data[9].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[9].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[10].uv_min�  )       PerViewData.shadow_map_2d_data[10].uv_max�  7       PerViewData.shadow_map_2d_data[10].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[10].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[11].uv_minP  )       PerViewData.shadow_map_2d_data[11].uv_maxX  7       PerViewData.shadow_map_2d_data[11].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[11].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[12].uv_min�  )       PerViewData.shadow_map_2d_data[12].uv_max�  7       PerViewData.shadow_map_2d_data[12].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[12].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[13].uv_min  )       PerViewData.shadow_map_2d_data[13].uv_max  7       PerViewData.shadow_map_2d_data[13].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[13].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[14].uv_minp  )       PerViewData.shadow_map_2d_data[14].uv_maxx  7       PerViewData.shadow_map_2d_data[14].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[14].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[15].uv_min�  )       PerViewData.shadow_map_2d_data[15].uv_max�  7       PerViewData.shadow_map_2d_data[15].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[15].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[16].uv_min0  )       PerViewData.shadow_map_2d_data[16].uv_max8  7       PerViewData.shadow_map_2d_data[16].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[16].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[17].uv_min�  )       PerViewData.shadow_map_2d_data[17].uv_max�  7       PerViewData.shadow_map_2d_data[17].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[17].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[18].uv_min�  )       PerViewData.shadow_map_2d_data[18].uv_max�  7       PerViewData.shadow_map_2d_data[18].shadow_map_view_proj 	  7       PerViewData.shadow_map_2d_data[18].shadow_map_light_dir@	  )       PerViewData.shadow_map_2d_data[19].uv_minP	  )       PerViewData.shadow_map_2d_data[19].uv_maxX	  7       PerViewData.shadow_map_2d_data[19].shadow_map_view_proj`	  7       PerViewData.shadow_map_2d_data[19].shadow_map_light_dir�	  )       PerViewData.shadow_map_2d_data[20].uv_min�	  )       PerViewData.shadow_map_2d_data[20].uv_max�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_view_proj�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_light_dir 
  )       PerViewData.shadow_map_2d_data[21].uv_min
  )       PerViewData.shadow_map_2d_data[21].uv_max
  7       PerViewData.shadow_map_2d_data[21].shadow_map_view_proj 
  7       PerViewData.shadow_map_2d_data[21].shadow_map_light_dir`
  )       PerViewData.shadow_map_2d_data[22].uv_minp
  )       PerViewData.shadow_map_2d_data[22].uv_maxx
  7       PerViewData.shadow_map_2d_data[22].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[22].shadow_map_light_dir�
  )       PerViewData.shadow_map_2d_data[23].uv_min�
  )       PerViewData.shadow_map_2d_data[23].uv_max�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[24].uv_min0  )       PerViewData.shadow_map_2d_data[24].uv_max8  7       PerViewData.shadow_map_2d_data[24].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[24].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[25].uv_min�  )       PerViewData.shadow_map_2d_data[25].uv_max�  7       PerViewData.shadow_map_2d_data[25].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[25].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[26].uv_min�  )       PerViewData.shadow_map_2d_data[26].uv_max�  7       PerViewData.shadow_map_2d_data[26].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[26].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[27].uv_minP  )       PerViewData.shadow_map_2d_data[27].uv_maxX  7       PerViewData.shadow_map_2d_data[27].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[27].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[28].uv_min�  )       PerViewData.shadow_map_2d_data[28].uv_max�  7       PerViewData.shadow_map_2d_data[28].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[28].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[29].uv_min  )       PerViewData.shadow_map_2d_data[29].uv_max  7       PerViewData.shadow_map_2d_data[29].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[29].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[30].uv_minp  )       PerViewData.shadow_map_2d_data[30].uv_maxx  7       PerViewData.shadow_map_2d_data[30].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[30].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[31].uv_min�  )       PerViewData.shadow_map_2d_data[31].uv_max�  7       PerViewData.shadow_map_2d_data[31].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[31].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[32].uv_min0  )       PerViewData.shadow_map_2d_data[32].uv_max8  7       PerViewData.shadow_map_2d_data[32].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[32].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[33].uv_min�  )       PerViewData.shadow_map_2d_data[33].uv_max�  7       PerViewData.shadow_map_2d_data[33].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[33].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[34].uv_min�  )       PerViewData.shadow_map_2d_data[34].uv_max�  7       PerViewData.shadow_map_2d_data[34].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[34].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[35].uv_minP  )       PerViewData.shadow_map_2d_data[35].uv_maxX  7       PerViewData.shadow_map_2d_data[35].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[35].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[36].uv_min�  )       PerViewData.shadow_map_2d_data[36].uv_max�  7       PerViewData.shadow_map_2d_data[36].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[36].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[37].uv_min  )       PerViewData.shadow_map_2d_data[37].uv_max  7       PerViewData.shadow_map_2d_data[37].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[37].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[38].uv_minp  )       PerViewData.shadow_map_2d_data[38].uv_maxx  7       PerViewData.shadow_map_2d_data[38].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[38].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[39].uv_min�  )       PerViewData.shadow_map_2d_data[39].uv_max�  7       PerViewData.shadow_map_2d_data[39].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[39].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[40].uv_min0  )       PerViewData.shadow_map_2d_data[40].uv_max8  7       PerViewData.shadow_map_2d_data[40].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[40].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[41].uv_min�  )       PerViewData.shadow_map_2d_data[41].uv_max�  7       PerViewData.shadow_map_2d_data[41].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[41].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[42].uv_min�  )       PerViewData.shadow_map_2d_data[42].uv_max�  7       PerViewData.shadow_map_2d_data[42].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[42].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[43].uv_minP  )       PerViewData.shadow_map_2d_data[43].uv_maxX  7       PerViewData.shadow_map_2d_data[43].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[43].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[44].uv_min�  )       PerViewData.shadow_map_2d_data[44].uv_max�  7       PerViewData.shadow_map_2d_data[44].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[44].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[45].uv_min  )       PerViewData.shadow_map_2d_data[45].uv_max  7       PerViewData.shadow_map_2d_data[45].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[45].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[46].uv_minp  )       PerViewData.shadow_map_2d_data[46].uv_maxx  7       PerViewData.shadow_map_2d_data[46].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[46].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[47].uv_min�  )       PerViewData.shadow_map_2d_data[47].uv_max�  7       PerViewData.shadow_map_2d_data[47].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[47].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[48].uv_min0  )       PerViewData.shadow_map_2d_data[48].uv_max8  7       PerViewData.shadow_map_2d_data[48].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[48].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[49].uv_min�  )       PerViewData.shadow_map_2d_data[49].uv_max�  7       PerViewData.shadow_map_2d_data[49].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[49].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[50].uv_min�  )       PerViewData.shadow_map_2d_data[50].uv_max�  7       PerViewData.shadow_map_2d_data[50].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[50].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[51].uv_minP  )       PerViewData.shadow_map_2d_data[51].uv_maxX  7       PerViewData.shadow_map_2d_data[51].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[51].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[52].uv_min�  )       PerViewData.shadow_map_2d_data[52].uv_max�  7       PerViewData.shadow_map_2d_data[52].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[52].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[53].uv_min  )       PerViewData.shadow_map_2d_data[53].uv_max  7       PerViewData.shadow_map_2d_data[53].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[53].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[54].uv_minp  )       PerViewData.shadow_map_2d_data[54].uv_maxx  7       PerViewData.shadow_map_2d_data[54].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[54].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[55].uv_min�  )       PerViewData.shadow_map_2d_data[55].uv_max�  7       PerViewData.shadow_map_2d_data[55].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[55].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[56].uv_min0  )       PerViewData.shadow_map_2d_data[56].uv_max8  7       PerViewData.shadow_map_2d_data[56].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[56].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[57].uv_min�  )       PerViewData.shadow_map_2d_data[57].uv_max�  7       PerViewData.shadow_map_2d_data[57].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[57].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[58].uv_min�  )       PerViewData.shadow_map_2d_data[58].uv_max�  7       PerViewData.shadow_map_2d_data[58].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[58].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[59].uv_minP  )       PerViewData.shadow_map_2d_data[59].uv_maxX  7       PerViewData.shadow_map_2d_data[59].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[59].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[60].uv_min�  )       PerViewData.shadow_map_2d_data[60].uv_max�  7       PerViewData.shadow_map_2d_data[60].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[60].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[61].uv_min  )       PerViewData.shadow_map_2d_data[61].uv_max  7       PerViewData.shadow_map_2d_data[61].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[61].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[62].uv_minp  )       PerViewData.shadow_map_2d_data[62].uv_maxx  7       PerViewData.shadow_map_2d_data[62].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[62].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[63].uv_min�  )       PerViewData.shadow_map_2d_data[63].uv_max�  7       PerViewData.shadow_map_2d_data[63].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[63].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[64].uv_min0  )       PerViewData.shadow_map_2d_data[64].uv_max8  7       PerViewData.shadow_map_2d_data[64].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[64].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[65].uv_min�  )       PerViewData.shadow_map_2d_data[65].uv_max�  7       PerViewData.shadow_map_2d_data[65].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[65].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[66].uv_min�  )       PerViewData.shadow_map_2d_data[66].uv_max�  7       PerViewData.shadow_map_2d_data[66].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[66].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[67].uv_minP  )       PerViewData.shadow_map_2d_data[67].uv_maxX  7       PerViewData.shadow_map_2d_data[67].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[67].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[68].uv_min�  )       PerViewData.shadow_map_2d_data[68].uv_max�  7       PerViewData.shadow_map_2d_data[68].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[68].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[69].uv_min  )       PerViewData.shadow_map_2d_data[69].uv_max  7       PerViewData.shadow_map_2d_data[69].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[69].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[70].uv_minp  )       PerViewData.shadow_map_2d_data[70].uv_maxx  7       PerViewData.shadow_map_2d_data[70].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[70].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[71].uv_min�  )       PerViewData.shadow_map_2d_data[71].uv_max�  7       PerViewData.shadow_map_2d_data[71].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[71].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[72].uv_min0  )       PerViewData.shadow_map_2d_data[72].uv_max8  7       PerViewData.shadow_map_2d_data[72].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[72].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[73].uv_min�  )       PerViewData.shadow_map_2d_data[73].uv_max�  7       PerViewData.shadow_map_2d_data[73].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[73].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[74].uv_min�  )       PerViewData.shadow_map_2d_data[74].uv_max�  7       PerViewData.shadow_map_2d_data[74].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[74].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[75].uv_minP  )       PerViewData.shadow_map_2d_data[75].uv_maxX  7       PerViewData.shadow_map_2d_data[75].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[75].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[76].uv_min�  )       PerViewData.shadow_map_2d_data[76].uv_max�  7       PerViewData.shadow_map_2d_data[76].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[76].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[77].uv_min  )       PerViewData.shadow_map_2d_data[77].uv_max  7       PerViewData.shadow_map_2d_data[77].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[77].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[78].uv_minp  )       PerViewData.shadow_map_2d_data[78].uv_maxx  7       PerViewData.shadow_map_2d_data[78].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[78].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[79].uv_min�  )       PerViewData.shadow_map_2d_data[79].uv_max�  7       PerViewData.shadow_map_2d_data[79].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[79].shadow_map_light_dir    )       PerViewData.shadow_map_2d_data[80].uv_min0   )       PerViewData.shadow_map_2d_data[80].uv_max8   7       PerViewData.shadow_map_2d_data[80].shadow_map_view_proj@   7       PerViewData.shadow_map_2d_data[80].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[81].uv_min�   )       PerViewData.shadow_map_2d_data[81].uv_max�   7       PerViewData.shadow_map_2d_data[81].shadow_map_view_proj�   7       PerViewData.shadow_map_2d_data[81].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[82].uv_min�   )       PerViewData.shadow_map_2d_data[82].uv_max�   7       PerViewData.shadow_map_2d_data[82].shadow_map_view_proj !  7       PerViewData.shadow_map_2d_data[82].shadow_map_light_dir@!  )       PerViewData.shadow_map_2d_data[83].uv_minP!  )       PerViewData.shadow_map_2d_data[83].uv_maxX!  7       PerViewData.shadow_map_2d_data[83].shadow_map_view_proj`!  7       PerViewData.shadow_map_2d_data[83].shadow_map_light_dir�!  )       PerViewData.shadow_map_2d_data[84].uv_min�!  )       PerViewData.shadow_map_2d_data[84].uv_max�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_view_proj�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_light_dir "  )       PerViewData.shadow_map_2d_data[85].uv_min"  )       PerViewData.shadow_map_2d_data[85].uv_max"  7       PerViewData.shadow_map_2d_data[85].shadow_map_view_proj "  7       PerViewData.shadow_map_2d_data[85].shadow_map_light_dir`"  )       PerViewData.shadow_map_2d_data[86].uv_minp"  )       PerViewData.shadow_map_2d_data[86].uv_maxx"  7       PerViewData.shadow_map_2d_data[86].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[86].shadow_map_light_dir�"  )       PerViewData.shadow_map_2d_data[87].uv_min�"  )       PerViewData.shadow_map_2d_data[87].uv_max�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_light_dir #  )       PerViewData.shadow_map_2d_data[88].uv_min0#  )       PerViewData.shadow_map_2d_data[88].uv_max8#  7       PerViewData.shadow_map_2d_data[88].shadow_map_view_proj@#  7       PerViewData.shadow_map_2d_data[88].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[89].uv_min�#  )       PerViewData.shadow_map_2d_data[89].uv_max�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_view_proj�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[90].uv_min�#  )       PerViewData.shadow_map_2d_data[90].uv_max�#  7       PerViewData.shadow_map_2d_data[90].shadow_map_view_proj $  7       PerViewData.shadow_map_2d_data[90].shadow_map_light_dir@$  )       PerViewData.shadow_map_2d_data[91].uv_minP$  )       PerViewData.shadow_map_2d_data[91].uv_maxX$  7       PerViewData.shadow_map_2d_data[91].shadow_map_view_proj`$  7       PerViewData.shadow_map_2d_data[91].shadow_map_light_dir�$  )       PerViewData.shadow_map_2d_data[92].uv_min�$  )       PerViewData.shadow_map_2d_data[92].uv_max�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_view_proj�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_light_dir %  )       PerViewData.shadow_map_2d_data[93].uv_min%  )       PerViewData.shadow_map_2d_data[93].uv_max%  7       PerViewData.shadow_map_2d_data[93].shadow_map_view_proj %  7       PerViewData.shadow_map_2d_data[93].shadow_map_light_dir`%  )       PerViewData.shadow_map_2d_data[94].uv_minp%  )       PerViewData.shadow_map_2d_data[94].uv_maxx%  7       PerViewData.shadow_map_2d_data[94].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[94].shadow_map_light_dir�%  )       PerViewData.shadow_map_2d_data[95].uv_min�%  )       PerViewData.shadow_map_2d_data[95].uv_max�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_light_dir &  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[0]0&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[1]@&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[2]P&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[3]`&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[4]p&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[0].cube_map_projection_near_z�&  =       PerViewData.shadow_map_cube_data[0].cube_map_projection_far_z�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[0]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[1]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[2]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[3]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[4]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[1].cube_map_projection_near_z '  =       PerViewData.shadow_map_cube_data[1].cube_map_projection_far_z'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[0]'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[1] '  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[2]0'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[3]@'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[4]P'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[5]`'  >       PerViewData.shadow_map_cube_data[2].cube_map_projection_near_zp'  =       PerViewData.shadow_map_cube_data[2].cube_map_projection_far_zt'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[1]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[2]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[3]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[4]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[5]�'  >       PerViewData.shadow_map_cube_data[3].cube_map_projection_near_z�'  =       PerViewData.shadow_map_cube_data[3].cube_map_projection_far_z�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[1] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[2](  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[3] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[4]0(  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[5]@(  >       PerViewData.shadow_map_cube_data[4].cube_map_projection_near_zP(  =       PerViewData.shadow_map_cube_data[4].cube_map_projection_far_zT(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[0]`(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[1]p(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[3]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[4]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[5]�(  >       PerViewData.shadow_map_cube_data[5].cube_map_projection_near_z�(  =       PerViewData.shadow_map_cube_data[5].cube_map_projection_far_z�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[0]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[1]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[3] )  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[4])  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[5] )  >       PerViewData.shadow_map_cube_data[6].cube_map_projection_near_z0)  =       PerViewData.shadow_map_cube_data[6].cube_map_projection_far_z4)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[0]@)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[1]P)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[2]`)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[3]p)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[5]�)  >       PerViewData.shadow_map_cube_data[7].cube_map_projection_near_z�)  =       PerViewData.shadow_map_cube_data[7].cube_map_projection_far_z�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[0]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[1]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[2]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[3]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[5] *  >       PerViewData.shadow_map_cube_data[8].cube_map_projection_near_z*  =       PerViewData.shadow_map_cube_data[8].cube_map_projection_far_z*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[0] *  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[1]0*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[2]@*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[3]P*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[4]`*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[5]p*  >       PerViewData.shadow_map_cube_data[9].cube_map_projection_near_z�*  =       PerViewData.shadow_map_cube_data[9].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[0]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[1]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[2]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[3]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[4]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[5]�*  ?       PerViewData.shadow_map_cube_data[10].cube_map_projection_near_z�*  >       PerViewData.shadow_map_cube_data[10].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[0] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[1]+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[2] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[3]0+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[4]@+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[5]P+  ?       PerViewData.shadow_map_cube_data[11].cube_map_projection_near_z`+  >       PerViewData.shadow_map_cube_data[11].cube_map_projection_far_zd+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[0]p+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[2]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[3]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[4]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[5]�+  ?       PerViewData.shadow_map_cube_data[12].cube_map_projection_near_z�+  >       PerViewData.shadow_map_cube_data[12].cube_map_projection_far_z�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[0]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[2] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[3],  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[4] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[5]0,  ?       PerViewData.shadow_map_cube_data[13].cube_map_projection_near_z@,  >       PerViewData.shadow_map_cube_data[13].cube_map_projection_far_zD,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[0]P,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[1]`,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[2]p,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[4]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[5]�,  ?       PerViewData.shadow_map_cube_data[14].cube_map_projection_near_z�,  >       PerViewData.shadow_map_cube_data[14].cube_map_projection_far_z�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[0]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[1]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[2]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[4] -  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[5]-  ?       PerViewData.shadow_map_cube_data[15].cube_map_projection_near_z -  >       PerViewData.shadow_map_cube_data[15].cube_map_projection_far_z$-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[0]0-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[1]@-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[2]P-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[3]`-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[4]p-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[16].cube_map_projection_near_z�-  >       PerViewData.shadow_map_cube_data[16].cube_map_projection_far_z�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[0]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[1]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[2]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[3]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[4]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[17].cube_map_projection_near_z .  >       PerViewData.shadow_map_cube_data[17].cube_map_projection_far_z.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[0].  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[1] .  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[2]0.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[3]@.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[4]P.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[5]`.  ?       PerViewData.shadow_map_cube_data[18].cube_map_projection_near_zp.  >       PerViewData.shadow_map_cube_data[18].cube_map_projection_far_zt.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[1]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[2]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[3]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[4]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[5]�.  ?       PerViewData.shadow_map_cube_data[19].cube_map_projection_near_z�.  >       PerViewData.shadow_map_cube_data[19].cube_map_projection_far_z�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[1] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[2]/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[3] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[4]0/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[5]@/  ?       PerViewData.shadow_map_cube_data[20].cube_map_projection_near_zP/  >       PerViewData.shadow_map_cube_data[20].cube_map_projection_far_zT/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[0]`/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[1]p/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[3]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[4]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[5]�/  ?       PerViewData.shadow_map_cube_data[21].cube_map_projection_near_z�/  >       PerViewData.shadow_map_cube_data[21].cube_map_projection_far_z�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[0]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[1]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[3] 0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[4]0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[5] 0  ?       PerViewData.shadow_map_cube_data[22].cube_map_projection_near_z00  >       PerViewData.shadow_map_cube_data[22].cube_map_projection_far_z40  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[0]@0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[1]P0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[2]`0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[3]p0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[5]�0  ?       PerViewData.shadow_map_cube_data[23].cube_map_projection_near_z�0  >       PerViewData.shadow_map_cube_data[23].cube_map_projection_far_z�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[0]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[1]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[2]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[3]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[5] 1  ?       PerViewData.shadow_map_cube_data[24].cube_map_projection_near_z1  >       PerViewData.shadow_map_cube_data[24].cube_map_projection_far_z1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[0] 1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[1]01  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[2]@1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[3]P1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[4]`1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[5]p1  ?       PerViewData.shadow_map_cube_data[25].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[25].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[0]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[1]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[2]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[3]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[4]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[5]�1  ?       PerViewData.shadow_map_cube_data[26].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[26].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[0] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[1]2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[2] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[3]02  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[4]@2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[5]P2  ?       PerViewData.shadow_map_cube_data[27].cube_map_projection_near_z`2  >       PerViewData.shadow_map_cube_data[27].cube_map_projection_far_zd2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[0]p2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[2]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[3]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[4]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[5]�2  ?       PerViewData.shadow_map_cube_data[28].cube_map_projection_near_z�2  >       PerViewData.shadow_map_cube_data[28].cube_map_projection_far_z�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[0]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[2] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[3]3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[4] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[5]03  ?       PerViewData.shadow_map_cube_data[29].cube_map_projection_near_z@3  >       PerViewData.shadow_map_cube_data[29].cube_map_projection_far_zD3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[0]P3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[1]`3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[2]p3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[4]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[5]�3  ?       PerViewData.shadow_map_cube_data[30].cube_map_projection_near_z�3  >       PerViewData.shadow_map_cube_data[30].cube_map_projection_far_z�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[0]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[1]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[2]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[4] 4  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[5]4  ?       PerViewData.shadow_map_cube_data[31].cube_map_projection_near_z 4  >       PerViewData.shadow_map_cube_data[31].cube_map_projection_far_z$4                             smp              smp                                    smp_depth_linear              smp_depth_linear                                    smp_depth_nearest              smp_depth_nearest                                    shadow_map_atlas              shadow_map_atlas                                     LightBinOutput              light_bin_output                              	       AllLights       
       all_lights                                    ssao_texture              ssao_texture                                     AllTransforms              all_transforms                                    AllDrawData             all_draw_data                                     AllMaterials              all_materials                                   all_material_textures             all_material_textures            ��������                 push_constants                                    main              �                            PerViewData               PerViewData �             PerViewData.view           PerViewData.view_proj@          PerViewData.ambient_light�          PerViewData.jitter_amount�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.mip_bias�          PerViewData.ndf_filter_amount�   #       PerViewData.directional_light_count�   "       PerViewData.use_clustered_lighting�   .       PerViewData.directional_lights[0].direction_ws�   +       PerViewData.directional_lights[0].intensity�   '       PerViewData.directional_lights[0].color�   .       PerViewData.directional_lights[0].direction_vs�   ,       PerViewData.directional_lights[0].shadow_map�   .       PerViewData.directional_lights[1].direction_ws�   +       PerViewData.directional_lights[1].intensity�   '       PerViewData.directional_lights[1].color�   .       PerViewData.directional_lights[1].direction_vs   ,       PerViewData.directional_lights[1].shadow_map  .       PerViewData.directional_lights[2].direction_ws  +       PerViewData.directional_lights[2].intensity  '       PerViewData.directional_lights[2].color   .       PerViewData.directional_lights[2].direction_vs0  ,       PerViewData.directional_lights[2].shadow_map<  .       PerViewData.directional_lights[3].direction_ws@  +       PerViewData.directional_lights[3].intensityL  '       PerViewData.directional_lights[3].colorP  .       PerViewData.directional_lights[3].direction_vs`  ,       PerViewData.directional_lights[3].shadow_mapl  .       PerViewData.directional_lights[4].direction_wsp  +       PerViewData.directional_lights[4].intensity|  '       PerViewData.directional_lights[4].color�  .       PerViewData.directional_lights[4].direction_vs�  ,       PerViewData.directional_lights[4].shadow_map�  .       PerViewData.directional_lights[5].direction_ws�  +       PerViewData.directional_lights[5].intensity�  '       PerViewData.directional_lights[5].color�  .       PerViewData.directional_lights[5].direction_vs�  ,       PerViewData.directional_lights[5].shadow_map�  .       PerViewData.directional_lights[6].direction_ws�  +       PerViewData.directional_lights[6].intensity�  '       PerViewData.directional_lights[6].color�  .       PerViewData.directional_lights[6].direction_vs�  ,       PerViewData.directional_lights[6].shadow_map�  .       PerViewData.directional_lights[7].direction_ws   +       PerViewData.directional_lights[7].intensity  '       PerViewData.directional_lights[7].color  .       PerViewData.directional_lights[7].direction_vs   ,       PerViewData.directional_lights[7].shadow_map,  (       PerViewData.shadow_map_2d_data[0].uv_min0  (       PerViewData.shadow_map_2d_data[0].uv_max8  6       PerViewData.shadow_map_2d_data[0].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[0].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[1].uv_min�  (       PerViewData.shadow_map_2d_data[1].uv_max�  6       PerViewData.shadow_map_2d_data[1].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[1].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[2].uv_min�  (       PerViewData.shadow_map_2d_data[2].uv_max�  6       PerViewData.shadow_map_2d_data[2].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[2].shadow_map_light_dir@  (       PerViewData.shadow_map_2d_data[3].uv_minP  (       PerViewData.shadow_map_2d_data[3].uv_maxX  6       PerViewData.shadow_map_2d_data[3].shadow_map_view_proj`  6       PerViewData.shadow_map_2d_data[3].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[4].uv_min�  (       PerViewData.shadow_map_2d_data[4].uv_max�  6       PerViewData.shadow_map_2d_data[4].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[4].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[5].uv_min  (       PerViewData.shadow_map_2d_data[5].uv_max  6       PerViewData.shadow_map_2d_data[5].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[5].shadow_map_light_dir`  (       PerViewData.shadow_map_2d_data[6].uv_minp  (       PerViewData.shadow_map_2d_data[6].uv_maxx  6       PerViewData.shadow_map_2d_data[6].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[6].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[7].uv_min�  (       PerViewData.shadow_map_2d_data[7].uv_max�  6       PerViewData.shadow_map_2d_data[7].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[7].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[8].uv_min0  (       PerViewData.shadow_map_2d_data[8].uv_max8  6       PerViewData.shadow_map_2d_data[8].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[8].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[9].uv_min�  (       PerViewData.shadow_map_2d_data[9].uv_max�  6       PerViewData.shadow_map_2d_data[9].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[9].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[10].uv_min�  )       PerViewData.shadow_map_2d_data[10].uv_max�  7       PerViewData.shadow_map_2d_data[10].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[10].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[11].uv_minP  )       PerViewData.shadow_map_2d_data[11].uv_maxX  7       PerViewData.shadow_map_2d_data[11].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[11].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[12].uv_min�  )       PerViewData.shadow_map_2d_data[12].uv_max�  7       PerViewData.shadow_map_2d_data[12].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[12].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[13].uv_min  )       PerViewData.shadow_map_2d_data[13].uv_max  7       PerViewData.shadow_map_2d_data[13].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[13].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[14].uv_minp  )       PerViewData.shadow_map_2d_data[14].uv_maxx  7       PerViewData.shadow_map_2d_data[14].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[14].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[15].uv_min�  )       PerViewData.shadow_map_2d_data[15].uv_max�  7       PerViewData.shadow_map_2d_data[15].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[15].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[16].uv_min0  )       PerViewData.shadow_map_2d_data[16].uv_max8  7       PerViewData.shadow_map_2d_data[16].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[16].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[17].uv_min�  )       PerViewData.shadow_map_2d_data[17].uv_max�  7       PerViewData.shadow_map_2d_data[17].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[17].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[18].uv_min�  )       PerViewData.shadow_map_2d_data[18].uv_max�  7       PerViewData.shadow_map_2d_data[18].shadow_map_view_proj 	  7       PerViewData.shadow_map_2d_data[18].shadow_map_light_dir@	  )       PerViewData.shadow_map_2d_data[19].uv_minP	  )       PerViewData.shadow_map_2d_data[19].uv_maxX	  7       PerViewData.shadow_map_2d_data[19].shadow_map_view_proj`	  7       PerViewData.shadow_map_2d_data[19].shadow_map_light_dir�	  )       PerViewData.shadow_map_2d_data[20].uv_min�	  )       PerViewData.shadow_map_2d_data[20].uv_max�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_view_proj�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_light_dir 
  )       PerViewData.shadow_map_2d_data[21].uv_min
  )       PerViewData.shadow_map_2d_data[21].uv_max
  7       PerViewData.shadow_map_2d_data[21].shadow_map_view_proj 
  7       PerViewData.shadow_map_2d_data[21].shadow_map_light_dir`
  )       PerViewData.shadow_map_2d_data[22].uv_minp
  )       PerViewData.shadow_map_2d_data[22].uv_maxx
  7       PerViewData.shadow_map_2d_data[22].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[22].shadow_map_light_dir�
  )       PerViewData.shadow_map_2d_data[23].uv_min�
  )       PerViewData.shadow_map_2d_data[23].uv_max�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[24].uv_min0  )       PerViewData.shadow_map_2d_data[24].uv_max8  7       PerViewData.shadow_map_2d_data[24].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[24].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[25].uv_min�  )       PerViewData.shadow_map_2d_data[25].uv_max�  7       PerViewData.shadow_map_2d_data[25].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[25].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[26].uv_min�  )       PerViewData.shadow_map_2d_data[26].uv_max�  7       PerViewData.shadow_map_2d_data[26].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[26].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[27].uv_minP  )       PerViewData.shadow_map_2d_data[27].uv_maxX  7       PerViewData.shadow_map_2d_data[27].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[27].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[28].uv_min�  )       PerViewData.shadow_map_2d_data[28].uv_max�  7       PerViewData.shadow_map_2d_data[28].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[28].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[29].uv_min  )       PerViewData.shadow_map_2d_data[29].uv_max  7       PerViewData.shadow_map_2d_data[29].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[29].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[30].uv_minp  )       PerViewData.shadow_map_2d_data[30].uv_maxx  7       PerViewData.shadow_map_2d_data[30].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[30].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[31].uv_min�  )       PerViewData.shadow_map_2d_data[31].uv_max�  7       PerViewData.shadow_map_2d_data[31].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[31].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[32].uv_min0  )       PerViewData.shadow_map_2d_data[32].uv_max8  7       PerViewData.shadow_map_2d_data[32].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[32].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[33].uv_min�  )       PerViewData.shadow_map_2d_data[33].uv_max�  7       PerViewData.shadow_map_2d_data[33].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[33].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[34].uv_min�  )       PerViewData.shadow_map_2d_data[34].uv_max�  7       PerViewData.shadow_map_2d_data[34].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[34].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[35].uv_minP  )       PerViewData.shadow_map_2d_data[35].uv_maxX  7       PerViewData.shadow_map_2d_data[35].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[35].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[36].uv_min�  )       PerViewData.shadow_map_2d_data[36].uv_max�  7       PerViewData.shadow_map_2d_data[36].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[36].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[37].uv_min  )       PerViewData.shadow_map_2d_data[37].uv_max  7       PerViewData.shadow_map_2d_data[37].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[37].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[38].uv_minp  )       PerViewData.shadow_map_2d_data[38].uv_maxx  7       PerViewData.shadow_map_2d_data[38].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[38].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[39].uv_min�  )       PerViewData.shadow_map_2d_data[39].uv_max�  7       PerViewData.shadow_map_2d_data[39].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[39].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[40].uv_min0  )       PerViewData.shadow_map_2d_data[40].uv_max8  7       PerViewData.shadow_map_2d_data[40].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[40].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[41].uv_min�  )       PerViewData.shadow_map_2d_data[41].uv_max�  7       PerViewData.shadow_map_2d_data[41].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[41].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[42].uv_min�  )       PerViewData.shadow_map_2d_data[42].uv_max�  7       PerViewData.shadow_map_2d_data[42].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[42].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[43].uv_minP  )       PerViewData.shadow_map_2d_data[43].uv_maxX  7       PerViewData.shadow_map_2d_data[43].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[43].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[44].uv_min�  )       PerViewData.shadow_map_2d_data[44].uv_max�  7       PerViewData.shadow_map_2d_data[44].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[44].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[45].uv_min  )       PerViewData.shadow_map_2d_data[45].uv_max  7       PerViewData.shadow_map_2d_data[45].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[45].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[46].uv_minp  )       PerViewData.shadow_map_2d_data[46].uv_maxx  7       PerViewData.shadow_map_2d_data[46].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[46].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[47].uv_min�  )       PerViewData.shadow_map_2d_data[47].uv_max�  7       PerViewData.shadow_map_2d_data[47].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[47].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[48].uv_min0  )       PerViewData.shadow_map_2d_data[48].uv_max8  7       PerViewData.shadow_map_2d_data[48].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[48].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[49].uv_min�  )       PerViewData.shadow_map_2d_data[49].uv_max�  7       PerViewData.shadow_map_2d_data[49].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[49].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[50].uv_min�  )       PerViewData.shadow_map_2d_data[50].uv_max�  7       PerViewData.shadow_map_2d_data[50].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[50].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[51].uv_minP  )       PerViewData.shadow_map_2d_data[51].uv_maxX  7       PerViewData.shadow_map_2d_data[51].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[51].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[52].uv_min�  )       PerViewData.shadow_map_2d_data[52].uv_max�  7       PerViewData.shadow_map_2d_data[52].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[52].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[53].uv_min  )       PerViewData.shadow_map_2d_data[53].uv_max  7       PerViewData.shadow_map_2d_data[53].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[53].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[54].uv_minp  )       PerViewData.shadow_map_2d_data[54].uv_maxx  7       PerViewData.shadow_map_2d_data[54].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[54].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[55].uv_min�  )       PerViewData.shadow_map_2d_data[55].uv_max�  7       PerViewData.shadow_map_2d_data[55].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[55].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[56].uv_min0  )       PerViewData.shadow_map_2d_data[56].uv_max8  7       PerViewData.shadow_map_2d_data[56].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[56].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[57].uv_min�  )       PerViewData.shadow_map_2d_data[57].uv_max�  7       PerViewData.shadow_map_2d_data[57].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[57].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[58].uv_min�  )       PerViewData.shadow_map_2d_data[58].uv_max�  7       PerViewData.shadow_map_2d_data[58].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[58].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[59].uv_minP  )       PerViewData.shadow_map_2d_data[59].uv_maxX  7       PerViewData.shadow_map_2d_data[59].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[59].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[60].uv_min�  )       PerViewData.shadow_map_2d_data[60].uv_max�  7       PerViewData.shadow_map_2d_data[60].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[60].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[61].uv_min  )       PerViewData.shadow_map_2d_data[61].uv_max  7       PerViewData.shadow_map_2d_data[61].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[61].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[62].uv_minp  )       PerViewData.shadow_map_2d_data[62].uv_maxx  7       PerViewData.shadow_map_2d_data[62].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[62].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[63].uv_min�  )       PerViewData.shadow_map_2d_data[63].uv_max�  7       PerViewData.shadow_map_2d_data[63].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[63].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[64].uv_min0  )       PerViewData.shadow_map_2d_data[64].uv_max8  7       PerViewData.shadow_map_2d_data[64].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[64].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[65].uv_min�  )       PerViewData.shadow_map_2d_data[65].uv_max�  7       PerViewData.shadow_map_2d_data[65].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[65].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[66].uv_min�  )       PerViewData.shadow_map_2d_data[66].uv_max�  7       PerViewData.shadow_map_2d_data[66].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[66].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[67].uv_minP  )       PerViewData.shadow_map_2d_data[67].uv_maxX  7       PerViewData.shadow_map_2d_data[67].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[67].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[68].uv_min�  )       PerViewData.shadow_map_2d_data[68].uv_max�  7       PerViewData.shadow_map_2d_data[68].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[68].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[69].uv_min  )       PerViewData.shadow_map_2d_data[69].uv_max  7       PerViewData.shadow_map_2d_data[69].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[69].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[70].uv_minp  )       PerViewData.shadow_map_2d_data[70].uv_maxx  7       PerViewData.shadow_map_2d_data[70].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[70].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[71].uv_min�  )       PerViewData.shadow_map_2d_data[71].uv_max�  7       PerViewData.shadow_map_2d_data[71].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[71].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[72].uv_min0  )       PerViewData.shadow_map_2d_data[72].uv_max8  7       PerViewData.shadow_map_2d_data[72].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[72].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[73].uv_min�  )       PerViewData.shadow_map_2d_data[73].uv_max�  7       PerViewData.shadow_map_2d_data[73].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[73].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[74].uv_min�  )       PerViewData.shadow_map_2d_data[74].uv_max�  7       PerViewData.shadow_map_2d_data[74].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[74].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[75].uv_minP  )       PerViewData.shadow_map_2d_data[75].uv_maxX  7       PerViewData.shadow_map_2d_data[75].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[75].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[76].uv_min�  )       PerViewData.shadow_map_2d_data[76].uv_max�  7       PerViewData.shadow_map_2d_data[76].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[76].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[77].uv_min  )       PerViewData.shadow_map_2d_data[77].uv_max  7       PerViewData.shadow_map_2d_data[77].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[77].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[78].uv_minp  )       PerViewData.shadow_map_2d_data[78].uv_maxx  7       PerViewData.shadow_map_2d_data[78].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[78].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[79].uv_min�  )       PerViewData.shadow_map_2d_data[79].uv_max�  7       PerViewData.shadow_map_2d_data[79].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[79].shadow_map_light_dir    )       PerViewData.shadow_map_2d_data[80].uv_min0   )       PerViewData.shadow_map_2d_data[80].uv_max8   7       PerViewData.shadow_map_2d_data[80].shadow_map_view_proj@   7       PerViewData.shadow_map_2d_data[80].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[81].uv_min�   )       PerViewData.shadow_map_2d_data[81].uv_max�   7       PerViewData.shadow_map_2d_data[81].shadow_map_view_proj�   7       PerViewData.shadow_map_2d_data[81].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[82].uv_min�   )       PerViewData.shadow_map_2d_data[82].uv_max�   7       PerViewData.shadow_map_2d_data[82].shadow_map_view_proj !  7       PerViewData.shadow_map_2d_data[82].shadow_map_light_dir@!  )       PerViewData.shadow_map_2d_data[83].uv_minP!  )       PerViewData.shadow_map_2d_data[83].uv_maxX!  7       PerViewData.shadow_map_2d_data[83].shadow_map_view_proj`!  7       PerViewData.shadow_map_2d_data[83].shadow_map_light_dir�!  )       PerViewData.shadow_map_2d_data[84].uv_min�!  )       PerViewData.shadow_map_2d_data[84].uv_max�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_view_proj�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_light_dir "  )       PerViewData.shadow_map_2d_data[85].uv_min"  )       PerViewData.shadow_map_2d_data[85].uv_max"  7       PerViewData.shadow_map_2d_data[85].shadow_map_view_proj "  7       PerViewData.shadow_map_2d_data[85].shadow_map_light_dir`"  )       PerViewData.shadow_map_2d_data[86].uv_minp"  )       PerViewData.shadow_map_2d_data[86].uv_maxx"  7       PerViewData.shadow_map_2d_data[86].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[86].shadow_map_light_dir�"  )       PerViewData.shadow_map_2d_data[87].uv_min�"  )       PerViewData.shadow_map_2d_data[87].uv_max�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_light_dir #  )       PerViewData.shadow_map_2d_data[88].uv_min0#  )       PerViewData.shadow_map_2d_data[88].uv_max8#  7       PerViewData.shadow_map_2d_data[88].shadow_map_view_proj@#  7       PerViewData.shadow_map_2d_data[88].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[89].uv_min�#  )       PerViewData.shadow_map_2d_data[89].uv_max�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_view_proj�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[90].uv_min�#  )       PerViewData.shadow_map_2d_data[90].uv_max�#  7       PerViewData.shadow_map_2d_data[90].shadow_map_view_proj $  7       PerViewData.shadow_map_2d_data[90].shadow_map_light_dir@$  )       PerViewData.shadow_map_2d_data[91].uv_minP$  )       PerViewData.shadow_map_2d_data[91].uv_maxX$  7       PerViewData.shadow_map_2d_data[91].shadow_map_view_proj`$  7       PerViewData.shadow_map_2d_data[91].shadow_map_light_dir�$  )       PerViewData.shadow_map_2d_data[92].uv_min�$  )       PerViewData.shadow_map_2d_data[92].uv_max�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_view_proj�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_light_dir %  )       PerViewData.shadow_map_2d_data[93].uv_min%  )       PerViewData.shadow_map_2d_data[93].uv_max%  7       PerViewData.shadow_map_2d_data[93].shadow_map_view_proj %  7       PerViewData.shadow_map_2d_data[93].shadow_map_light_dir`%  )       PerViewData.shadow_map_2d_data[94].uv_minp%  )       PerViewData.shadow_map_2d_data[94].uv_maxx%  7       PerViewData.shadow_map_2d_data[94].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[94].shadow_map_light_dir�%  )       PerViewData.shadow_map_2d_data[95].uv_min�%  )       PerViewData.shadow_map_2d_data[95].uv_max�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_light_dir &  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[0]0&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[1]@&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[2]P&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[3]`&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[4]p&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[0].cube_map_projection_near_z�&  =       PerViewData.shadow_map_cube_data[0].cube_map_projection_far_z�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[0]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[1]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[2]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[3]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[4]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[1].cube_map_projection_near_z '  =       PerViewData.shadow_map_cube_data[1].cube_map_projection_far_z'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[0]'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[1] '  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[2]0'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[3]@'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[4]P'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[5]`'  >       PerViewData.shadow_map_cube_data[2].cube_map_projection_near_zp'  =       PerViewData.shadow_map_cube_data[2].cube_map_projection_far_zt'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[1]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[2]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[3]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[4]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[5]�'  >       PerViewData.shadow_map_cube_data[3].cube_map_projection_near_z�'  =       PerViewData.shadow_map_cube_data[3].cube_map_projection_far_z�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[1] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[2](  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[3] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[4]0(  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[5]@(  >       PerViewData.shadow_map_cube_data[4].cube_map_projection_near_zP(  =       PerViewData.shadow_map_cube_data[4].cube_map_projection_far_zT(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[0]`(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[1]p(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[3]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[4]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[5]�(  >       PerViewData.shadow_map_cube_data[5].cube_map_projection_near_z�(  =       PerViewData.shadow_map_cube_data[5].cube_map_projection_far_z�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[0]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[1]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[3] )  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[4])  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[5] )  >       PerViewData.shadow_map_cube_data[6].cube_map_projection_near_z0)  =       PerViewData.shadow_map_cube_data[6].cube_map_projection_far_z4)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[0]@)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[1]P)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[2]`)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[3]p)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[5]�)  >       PerViewData.shadow_map_cube_data[7].cube_map_projection_near_z�)  =       PerViewData.shadow_map_cube_data[7].cube_map_projection_far_z�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[0]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[1]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[2]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[3]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[5] *  >       PerViewData.shadow_map_cube_data[8].cube_map_projection_near_z*  =       PerViewData.shadow_map_cube_data[8].cube_map_projection_far_z*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[0] *  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[1]0*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[2]@*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[3]P*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[4]`*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[5]p*  >       PerViewData.shadow_map_cube_data[9].cube_map_projection_near_z�*  =       PerViewData.shadow_map_cube_data[9].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[0]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[1]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[2]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[3]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[4]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[5]�*  ?       PerViewData.shadow_map_cube_data[10].cube_map_projection_near_z�*  >       PerViewData.shadow_map_cube_data[10].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[0] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[1]+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[2] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[3]0+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[4]@+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[5]P+  ?       PerViewData.shadow_map_cube_data[11].cube_map_projection_near_z`+  >       PerViewData.shadow_map_cube_data[11].cube_map_projection_far_zd+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[0]p+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[2]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[3]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[4]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[5]�+  ?       PerViewData.shadow_map_cube_data[12].cube_map_projection_near_z�+  >       PerViewData.shadow_map_cube_data[12].cube_map_projection_far_z�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[0]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[2] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[3],  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[4] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[5]0,  ?       PerViewData.shadow_map_cube_data[13].cube_map_projection_near_z@,  >       PerViewData.shadow_map_cube_data[13].cube_map_projection_far_zD,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[0]P,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[1]`,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[2]p,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[4]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[5]�,  ?       PerViewData.shadow_map_cube_data[14].cube_map_projection_near_z�,  >       PerViewData.shadow_map_cube_data[14].cube_map_projection_far_z�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[0]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[1]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[2]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[4] -  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[5]-  ?       PerViewData.shadow_map_cube_data[15].cube_map_projection_near_z -  >       PerViewData.shadow_map_cube_data[15].cube_map_projection_far_z$-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[0]0-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[1]@-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[2]P-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[3]`-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[4]p-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[16].cube_map_projection_near_z�-  >       PerViewData.shadow_map_cube_data[16].cube_map_projection_far_z�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[0]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[1]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[2]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[3]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[4]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[17].cube_map_projection_near_z .  >       PerViewData.shadow_map_cube_data[17].cube_map_projection_far_z.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[0].  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[1] .  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[2]0.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[3]@.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[4]P.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[5]`.  ?       PerViewData.shadow_map_cube_data[18].cube_map_projection_near_zp.  >       PerViewData.shadow_map_cube_data[18].cube_map_projection_far_zt.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[1]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[2]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[3]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[4]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[5]�.  ?       PerViewData.shadow_map_cube_data[19].cube_map_projection_near_z�.  >       PerViewData.shadow_map_cube_data[19].cube_map_projection_far_z�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[1] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[2]/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[3] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[4]0/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[5]@/  ?       PerViewData.shadow_map_cube_data[20].cube_map_projection_near_zP/  >       PerViewData.shadow_map_cube_data[20].cube_map_projection_far_zT/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[0]`/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[1]p/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[3]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[4]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[5]�/  ?       PerViewData.shadow_map_cube_data[21].cube_map_projection_near_z�/  >       PerViewData.shadow_map_cube_data[21].cube_map_projection_far_z�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[0]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[1]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[3] 0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[4]0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[5] 0  ?       PerViewData.shadow_map_cube_data[22].cube_map_projection_near_z00  >       PerViewData.shadow_map_cube_data[22].cube_map_projection_far_z40  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[0]@0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[1]P0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[2]`0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[3]p0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[5]�0  ?       PerViewData.shadow_map_cube_data[23].cube_map_projection_near_z�0  >       PerViewData.shadow_map_cube_data[23].cube_map_projection_far_z�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[0]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[1]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[2]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[3]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[5] 1  ?       PerViewData.shadow_map_cube_data[24].cube_map_projection_near_z1  >       PerViewData.shadow_map_cube_data[24].cube_map_projection_far_z1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[0] 1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[1]01  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[2]@1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[3]P1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[4]`1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[5]p1  ?       PerViewData.shadow_map_cube_data[25].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[25].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[0]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[1]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[2]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[3]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[4]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[5]�1  ?       PerViewData.shadow_map_cube_data[26].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[26].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[0] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[1]2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[2] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[3]02  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[4]@2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[5]P2  ?       PerViewData.shadow_map_cube_data[27].cube_map_projection_near_z`2  >       PerViewData.shadow_map_cube_data[27].cube_map_projection_far_zd2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[0]p2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[2]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[3]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[4]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[5]�2  ?       PerViewData.shadow_map_cube_data[28].cube_map_projection_near_z�2  >       PerViewData.shadow_map_cube_data[28].cube_map_projection_far_z�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[0]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[2] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[3]3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[4] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[5]03  ?       PerViewData.shadow_map_cube_data[29].cube_map_projection_near_z@3  >       PerViewData.shadow_map_cube_data[29].cube_map_projection_far_zD3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[0]P3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[1]`3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[2]p3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[4]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[5]�3  ?       PerViewData.shadow_map_cube_data[30].cube_map_projection_near_z�3  >       PerViewData.shadow_map_cube_data[30].cube_map_projection_far_z�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[0]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[1]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[2]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[4] 4  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[5]4  ?       PerViewData.shadow_map_cube_data[31].cube_map_projection_near_z 4  >       PerViewData.shadow_map_cube_data[31].cube_map_projection_far_z$4   04                             smp              smp                                        �A                                smp_depth_linear              smp_depth_linear                                        �?                               smp_depth_nearest              smp_depth_nearest                                           �?                               shadow_map_atlas              shadow_map_atlas                                       LightBinOutput              light_bin_output                                	       AllLights       
       all_lights                                             ssao_texture              ssao_texture                                              AllTransforms              all_transforms                                      AllDrawData             all_draw_data                                              AllMaterials              all_materials                                     all_material_textures             all_material_textures                                    �                            PerViewData               PerViewData �             PerViewData.view           PerViewData.view_proj@          PerViewData.ambient_light�          PerViewData.jitter_amount�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.mip_bias�          PerViewData.ndf_filter_amount�   #       PerViewData.directional_light_count�   "       PerViewData.use_clustered_lighting�   .       PerViewData.directional_lights[0].direction_ws�   +       PerViewData.directional_lights[0].intensity�   '       PerViewData.directional_lights[0].color�   .       PerViewData.directional_lights[0].direction_vs�   ,       PerViewData.directional_lights[0].shadow_map�   .       PerViewData.directional_lights[1].direction_ws�   +       PerViewData.directional_lights[1].intensity�   '       PerViewData.directional_lights[1].color�   .       PerViewData.directional_lights[1].direction_vs   ,       PerViewData.directional_lights[1].shadow_map  .       PerViewData.directional_lights[2].direction_ws  +       PerViewData.directional_lights[2].intensity  '       PerViewData.directional_lights[2].color   .       PerViewData.directional_lights[2].direction_vs0  ,       PerViewData.directional_lights[2].shadow_map<  .       PerViewData.directional_lights[3].direction_ws@  +       PerViewData.directional_lights[3].intensityL  '       PerViewData.directional_lights[3].colorP  .       PerViewData.directional_lights[3].direction_vs`  ,       PerViewData.directional_lights[3].shadow_mapl  .       PerViewData.directional_lights[4].direction_wsp  +       PerViewData.directional_lights[4].intensity|  '       PerViewData.directional_lights[4].color�  .       PerViewData.directional_lights[4].direction_vs�  ,       PerViewData.directional_lights[4].shadow_map�  .       PerViewData.directional_lights[5].direction_ws�  +       PerViewData.directional_lights[5].intensity�  '       PerViewData.directional_lights[5].color�  .       PerViewData.directional_lights[5].direction_vs�  ,       PerViewData.directional_lights[5].shadow_map�  .       PerViewData.directional_lights[6].direction_ws�  +       PerViewData.directional_lights[6].intensity�  '       PerViewData.directional_lights[6].color�  .       PerViewData.directional_lights[6].direction_vs�  ,       PerViewData.directional_lights[6].shadow_map�  .       PerViewData.directional_lights[7].direction_ws   +       PerViewData.directional_lights[7].intensity  '       PerViewData.directional_lights[7].color  .       PerViewData.directional_lights[7].direction_vs   ,       PerViewData.directional_lights[7].shadow_map,  (       PerViewData.shadow_map_2d_data[0].uv_min0  (       PerViewData.shadow_map_2d_data[0].uv_max8  6       PerViewData.shadow_map_2d_data[0].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[0].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[1].uv_min�  (       PerViewData.shadow_map_2d_data[1].uv_max�  6       PerViewData.shadow_map_2d_data[1].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[1].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[2].uv_min�  (       PerViewData.shadow_map_2d_data[2].uv_max�  6       PerViewData.shadow_map_2d_data[2].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[2].shadow_map_light_dir@  (       PerViewData.shadow_map_2d_data[3].uv_minP  (       PerViewData.shadow_map_2d_data[3].uv_maxX  6       PerViewData.shadow_map_2d_data[3].shadow_map_view_proj`  6       PerViewData.shadow_map_2d_data[3].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[4].uv_min�  (       PerViewData.shadow_map_2d_data[4].uv_max�  6       PerViewData.shadow_map_2d_data[4].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[4].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[5].uv_min  (       PerViewData.shadow_map_2d_data[5].uv_max  6       PerViewData.shadow_map_2d_data[5].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[5].shadow_map_light_dir`  (       PerViewData.shadow_map_2d_data[6].uv_minp  (       PerViewData.shadow_map_2d_data[6].uv_maxx  6       PerViewData.shadow_map_2d_data[6].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[6].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[7].uv_min�  (       PerViewData.shadow_map_2d_data[7].uv_max�  6       PerViewData.shadow_map_2d_data[7].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[7].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[8].uv_min0  (       PerViewData.shadow_map_2d_data[8].uv_max8  6       PerViewData.shadow_map_2d_data[8].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[8].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[9].uv_min�  (       PerViewData.shadow_map_2d_data[9].uv_max�  6       PerViewData.shadow_map_2d_data[9].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[9].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[10].uv_min�  )       PerViewData.shadow_map_2d_data[10].uv_max�  7       PerViewData.shadow_map_2d_data[10].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[10].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[11].uv_minP  )       PerViewData.shadow_map_2d_data[11].uv_maxX  7       PerViewData.shadow_map_2d_data[11].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[11].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[12].uv_min�  )       PerViewData.shadow_map_2d_data[12].uv_max�  7       PerViewData.shadow_map_2d_data[12].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[12].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[13].uv_min  )       PerViewData.shadow_map_2d_data[13].uv_max  7       PerViewData.shadow_map_2d_data[13].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[13].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[14].uv_minp  )       PerViewData.shadow_map_2d_data[14].uv_maxx  7       PerViewData.shadow_map_2d_data[14].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[14].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[15].uv_min�  )       PerViewData.shadow_map_2d_data[15].uv_max�  7       PerViewData.shadow_map_2d_data[15].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[15].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[16].uv_min0  )       PerViewData.shadow_map_2d_data[16].uv_max8  7       PerViewData.shadow_map_2d_data[16].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[16].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[17].uv_min�  )       PerViewData.shadow_map_2d_data[17].uv_max�  7       PerViewData.shadow_map_2d_data[17].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[17].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[18].uv_min�  )       PerViewData.shadow_map_2d_data[18].uv_max�  7       PerViewData.shadow_map_2d_data[18].shadow_map_view_proj 	  7       PerViewData.shadow_map_2d_data[18].shadow_map_light_dir@	  )       PerViewData.shadow_map_2d_data[19].uv_minP	  )       PerViewData.shadow_map_2d_data[19].uv_maxX	  7       PerViewData.shadow_map_2d_data[19].shadow_map_view_proj`	  7       PerViewData.shadow_map_2d_data[19].shadow_map_light_dir�	  )       PerViewData.shadow_map_2d_data[20].uv_min�	  )       PerViewData.shadow_map_2d_data[20].uv_max�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_view_proj�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_light_dir 
  )       PerViewData.shadow_map_2d_data[21].uv_min
  )       PerViewData.shadow_map_2d_data[21].uv_max
  7       PerViewData.shadow_map_2d_data[21].shadow_map_view_proj 
  7       PerViewData.shadow_map_2d_data[21].shadow_map_light_dir`
  )       PerViewData.shadow_map_2d_data[22].uv_minp
  )       PerViewData.shadow_map_2d_data[22].uv_maxx
  7       PerViewData.shadow_map_2d_data[22].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[22].shadow_map_light_dir�
  )       PerViewData.shadow_map_2d_data[23].uv_min�
  )       PerViewData.shadow_map_2d_data[23].uv_max�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[24].uv_min0  )       PerViewData.shadow_map_2d_data[24].uv_max8  7       PerViewData.shadow_map_2d_data[24].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[24].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[25].uv_min�  )       PerViewData.shadow_map_2d_data[25].uv_max�  7       PerViewData.shadow_map_2d_data[25].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[25].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[26].uv_min�  )       PerViewData.shadow_map_2d_data[26].uv_max�  7       PerViewData.shadow_map_2d_data[26].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[26].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[27].uv_minP  )       PerViewData.shadow_map_2d_data[27].uv_maxX  7       PerViewData.shadow_map_2d_data[27].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[27].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[28].uv_min�  )       PerViewData.shadow_map_2d_data[28].uv_max�  7       PerViewData.shadow_map_2d_data[28].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[28].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[29].uv_min  )       PerViewData.shadow_map_2d_data[29].uv_max  7       PerViewData.shadow_map_2d_data[29].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[29].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[30].uv_minp  )       PerViewData.shadow_map_2d_data[30].uv_maxx  7       PerViewData.shadow_map_2d_data[30].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[30].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[31].uv_min�  )       PerViewData.shadow_map_2d_data[31].uv_max�  7       PerViewData.shadow_map_2d_data[31].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[31].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[32].uv_min0  )       PerViewData.shadow_map_2d_data[32].uv_max8  7       PerViewData.shadow_map_2d_data[32].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[32].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[33].uv_min�  )       PerViewData.shadow_map_2d_data[33].uv_max�  7       PerViewData.shadow_map_2d_data[33].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[33].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[34].uv_min�  )       PerViewData.shadow_map_2d_data[34].uv_max�  7       PerViewData.shadow_map_2d_data[34].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[34].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[35].uv_minP  )       PerViewData.shadow_map_2d_data[35].uv_maxX  7       PerViewData.shadow_map_2d_data[35].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[35].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[36].uv_min�  )       PerViewData.shadow_map_2d_data[36].uv_max�  7       PerViewData.shadow_map_2d_data[36].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[36].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[37].uv_min  )       PerViewData.shadow_map_2d_data[37].uv_max  7       PerViewData.shadow_map_2d_data[37].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[37].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[38].uv_minp  )       PerViewData.shadow_map_2d_data[38].uv_maxx  7       PerViewData.shadow_map_2d_data[38].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[38].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[39].uv_min�  )       PerViewData.shadow_map_2d_data[39].uv_max�  7       PerViewData.shadow_map_2d_data[39].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[39].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[40].uv_min0  )       PerViewData.shadow_map_2d_data[40].uv_max8  7       PerViewData.shadow_map_2d_data[40].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[40].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[41].uv_min�  )       PerViewData.shadow_map_2d_data[41].uv_max�  7       PerViewData.shadow_map_2d_data[41].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[41].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[42].uv_min�  )       PerViewData.shadow_map_2d_data[42].uv_max�  7       PerViewData.shadow_map_2d_data[42].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[42].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[43].uv_minP  )       PerViewData.shadow_map_2d_data[43].uv_maxX  7       PerViewData.shadow_map_2d_data[43].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[43].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[44].uv_min�  )       PerViewData.shadow_map_2d_data[44].uv_max�  7       PerViewData.shadow_map_2d_data[44].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[44].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[45].uv_min  )       PerViewData.shadow_map_2d_data[45].uv_max  7       PerViewData.shadow_map_2d_data[45].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[45].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[46].uv_minp  )       PerViewData.shadow_map_2d_data[46].uv_maxx  7       PerViewData.shadow_map_2d_data[46].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[46].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[47].uv_min�  )       PerViewData.shadow_map_2d_data[47].uv_max�  7       PerViewData.shadow_map_2d_data[47].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[47].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[48].uv_min0  )       PerViewData.shadow_map_2d_data[48].uv_max8  7       PerViewData.shadow_map_2d_data[48].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[48].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[49].uv_min�  )       PerViewData.shadow_map_2d_data[49].uv_max�  7       PerViewData.shadow_map_2d_data[49].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[49].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[50].uv_min�  )       PerViewData.shadow_map_2d_data[50].uv_max�  7       PerViewData.shadow_map_2d_data[50].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[50].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[51].uv_minP  )       PerViewData.shadow_map_2d_data[51].uv_maxX  7       PerViewData.shadow_map_2d_data[51].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[51].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[52].uv_min�  )       PerViewData.shadow_map_2d_data[52].uv_max�  7       PerViewData.shadow_map_2d_data[52].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[52].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[53].uv_min  )       PerViewData.shadow_map_2d_data[53].uv_max  7       PerViewData.shadow_map_2d_data[53].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[53].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[54].uv_minp  )       PerViewData.shadow_map_2d_data[54].uv_maxx  7       PerViewData.shadow_map_2d_data[54].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[54].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[55].uv_min�  )       PerViewData.shadow_map_2d_data[55].uv_max�  7       PerViewData.shadow_map_2d_data[55].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[55].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[56].uv_min0  )       PerViewData.shadow_map_2d_data[56].uv_max8  7       PerViewData.shadow_map_2d_data[56].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[56].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[57].uv_min�  )       PerViewData.shadow_map_2d_data[57].uv_max�  7       PerViewData.shadow_map_2d_data[57].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[57].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[58].uv_min�  )       PerViewData.shadow_map_2d_data[58].uv_max�  7       PerViewData.shadow_map_2d_data[58].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[58].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[59].uv_minP  )       PerViewData.shadow_map_2d_data[59].uv_maxX  7       PerViewData.shadow_map_2d_data[59].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[59].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[60].uv_min�  )       PerViewData.shadow_map_2d_data[60].uv_max�  7       PerViewData.shadow_map_2d_data[60].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[60].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[61].uv_min  )       PerViewData.shadow_map_2d_data[61].uv_max  7       PerViewData.shadow_map_2d_data[61].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[61].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[62].uv_minp  )       PerViewData.shadow_map_2d_data[62].uv_maxx  7       PerViewData.shadow_map_2d_data[62].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[62].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[63].uv_min�  )       PerViewData.shadow_map_2d_data[63].uv_max�  7       PerViewData.shadow_map_2d_data[63].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[63].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[64].uv_min0  )       PerViewData.shadow_map_2d_data[64].uv_max8  7       PerViewData.shadow_map_2d_data[64].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[64].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[65].uv_min�  )       PerViewData.shadow_map_2d_data[65].uv_max�  7       PerViewData.shadow_map_2d_data[65].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[65].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[66].uv_min�  )       PerViewData.shadow_map_2d_data[66].uv_max�  7       PerViewData.shadow_map_2d_data[66].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[66].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[67].uv_minP  )       PerViewData.shadow_map_2d_data[67].uv_maxX  7       PerViewData.shadow_map_2d_data[67].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[67].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[68].uv_min�  )       PerViewData.shadow_map_2d_data[68].uv_max�  7       PerViewData.shadow_map_2d_data[68].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[68].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[69].uv_min  )       PerViewData.shadow_map_2d_data[69].uv_max  7       PerViewData.shadow_map_2d_data[69].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[69].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[70].uv_minp  )       PerViewData.shadow_map_2d_data[70].uv_maxx  7       PerViewData.shadow_map_2d_data[70].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[70].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[71].uv_min�  )       PerViewData.shadow_map_2d_data[71].uv_max�  7       PerViewData.shadow_map_2d_data[71].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[71].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[72].uv_min0  )       PerViewData.shadow_map_2d_data[72].uv_max8  7       PerViewData.shadow_map_2d_data[72].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[72].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[73].uv_min�  )       PerViewData.shadow_map_2d_data[73].uv_max�  7       PerViewData.shadow_map_2d_data[73].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[73].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[74].uv_min�  )       PerViewData.shadow_map_2d_data[74].uv_max�  7       PerViewData.shadow_map_2d_data[74].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[74].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[75].uv_minP  )       PerViewData.shadow_map_2d_data[75].uv_maxX  7       PerViewData.shadow_map_2d_data[75].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[75].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[76].uv_min�  )       PerViewData.shadow_map_2d_data[76].uv_max�  7       PerViewData.shadow_map_2d_data[76].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[76].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[77].uv_min  )       PerViewData.shadow_map_2d_data[77].uv_max  7       PerViewData.shadow_map_2d_data[77].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[77].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[78].uv_minp  )       PerViewData.shadow_map_2d_data[78].uv_maxx  7       PerViewData.shadow_map_2d_data[78].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[78].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[79].uv_min�  )       PerViewData.shadow_map_2d_data[79].uv_max�  7       PerViewData.shadow_map_2d_data[79].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[79].shadow_map_light_dir    )       PerViewData.shadow_map_2d_data[80].uv_min0   )       PerViewData.shadow_map_2d_data[80].uv_max8   7       PerViewData.shadow_map_2d_data[80].shadow_map_view_proj@   7       PerViewData.shadow_map_2d_data[80].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[81].uv_min�   )       PerViewData.shadow_map_2d_data[81].uv_max�   7       PerViewData.shadow_map_2d_data[81].shadow_map_view_proj�   7       PerViewData.shadow_map_2d_data[81].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[82].uv_min�   )       PerViewData.shadow_map_2d_data[82].uv_max�   7       PerViewData.shadow_map_2d_data[82].shadow_map_view_proj !  7       PerViewData.shadow_map_2d_data[82].shadow_map_light_dir@!  )       PerViewData.shadow_map_2d_data[83].uv_minP!  )       PerViewData.shadow_map_2d_data[83].uv_maxX!  7       PerViewData.shadow_map_2d_data[83].shadow_map_view_proj`!  7       PerViewData.shadow_map_2d_data[83].shadow_map_light_dir�!  )       PerViewData.shadow_map_2d_data[84].uv_min�!  )       PerViewData.shadow_map_2d_data[84].uv_max�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_view_proj�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_light_dir "  )       PerViewData.shadow_map_2d_data[85].uv_min"  )       PerViewData.shadow_map_2d_data[85].uv_max"  7       PerViewData.shadow_map_2d_data[85].shadow_map_view_proj "  7       PerViewData.shadow_map_2d_data[85].shadow_map_light_dir`"  )       PerViewData.shadow_map_2d_data[86].uv_minp"  )       PerViewData.shadow_map_2d_data[86].uv_maxx"  7       PerViewData.shadow_map_2d_data[86].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[86].shadow_map_light_dir�"  )       PerViewData.shadow_map_2d_data[87].uv_min�"  )       PerViewData.shadow_map_2d_data[87].uv_max�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_light_dir #  )       PerViewData.shadow_map_2d_data[88].uv_min0#  )       PerViewData.shadow_map_2d_data[88].uv_max8#  7       PerViewData.shadow_map_2d_data[88].shadow_map_view_proj@#  7       PerViewData.shadow_map_2d_data[88].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[89].uv_min�#  )       PerViewData.shadow_map_2d_data[89].uv_max�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_view_proj�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[90].uv_min�#  )       PerViewData.shadow_map_2d_data[90].uv_max�#  7       PerViewData.shadow_map_2d_data[90].shadow_map_view_proj $  7       PerViewData.shadow_map_2d_data[90].shadow_map_light_dir@$  )       PerViewData.shadow_map_2d_data[91].uv_minP$  )       PerViewData.shadow_map_2d_data[91].uv_maxX$  7       PerViewData.shadow_map_2d_data[91].shadow_map_view_proj`$  7       PerViewData.shadow_map_2d_data[91].shadow_map_light_dir�$  )       PerViewData.shadow_map_2d_data[92].uv_min�$  )       PerViewData.shadow_map_2d_data[92].uv_max�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_view_proj�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_light_dir %  )       PerViewData.shadow_map_2d_data[93].uv_min%  )       PerViewData.shadow_map_2d_data[93].uv_max%  7       PerViewData.shadow_map_2d_data[93].shadow_map_view_proj %  7       PerViewData.shadow_map_2d_data[93].shadow_map_light_dir`%  )       PerViewData.shadow_map_2d_data[94].uv_minp%  )       PerViewData.shadow_map_2d_data[94].uv_maxx%  7       PerViewData.shadow_map_2d_data[94].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[94].shadow_map_light_dir�%  )       PerViewData.shadow_map_2d_data[95].uv_min�%  )       PerViewData.shadow_map_2d_data[95].uv_max�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_light_dir &  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[0]0&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[1]@&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[2]P&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[3]`&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[4]p&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[0].cube_map_projection_near_z�&  =       PerViewData.shadow_map_cube_data[0].cube_map_projection_far_z�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[0]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[1]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[2]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[3]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[4]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[1].cube_map_projection_near_z '  =       PerViewData.shadow_map_cube_data[1].cube_map_projection_far_z'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[0]'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[1] '  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[2]0'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[3]@'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[4]P'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[5]`'  >       PerViewData.shadow_map_cube_data[2].cube_map_projection_near_zp'  =       PerViewData.shadow_map_cube_data[2].cube_map_projection_far_zt'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[1]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[2]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[3]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[4]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[5]�'  >       PerViewData.shadow_map_cube_data[3].cube_map_projection_near_z�'  =       PerViewData.shadow_map_cube_data[3].cube_map_projection_far_z�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[1] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[2](  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[3] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[4]0(  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[5]@(  >       PerViewData.shadow_map_cube_data[4].cube_map_projection_near_zP(  =       PerViewData.shadow_map_cube_data[4].cube_map_projection_far_zT(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[0]`(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[1]p(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[3]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[4]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[5]�(  >       PerViewData.shadow_map_cube_data[5].cube_map_projection_near_z�(  =       PerViewData.shadow_map_cube_data[5].cube_map_projection_far_z�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[0]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[1]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[3] )  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[4])  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[5] )  >       PerViewData.shadow_map_cube_data[6].cube_map_projection_near_z0)  =       PerViewData.shadow_map_cube_data[6].cube_map_projection_far_z4)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[0]@)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[1]P)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[2]`)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[3]p)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[5]�)  >       PerViewData.shadow_map_cube_data[7].cube_map_projection_near_z�)  =       PerViewData.shadow_map_cube_data[7].cube_map_projection_far_z�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[0]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[1]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[2]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[3]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[5] *  >       PerViewData.shadow_map_cube_data[8].cube_map_projection_near_z*  =       PerViewData.shadow_map_cube_data[8].cube_map_projection_far_z*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[0] *  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[1]0*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[2]@*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[3]P*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[4]`*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[5]p*  >       PerViewData.shadow_map_cube_data[9].cube_map_projection_near_z�*  =       PerViewData.shadow_map_cube_data[9].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[0]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[1]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[2]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[3]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[4]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[5]�*  ?       PerViewData.shadow_map_cube_data[10].cube_map_projection_near_z�*  >       PerViewData.shadow_map_cube_data[10].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[0] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[1]+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[2] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[3]0+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[4]@+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[5]P+  ?       PerViewData.shadow_map_cube_data[11].cube_map_projection_near_z`+  >       PerViewData.shadow_map_cube_data[11].cube_map_projection_far_zd+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[0]p+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[2]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[3]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[4]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[5]�+  ?       PerViewData.shadow_map_cube_data[12].cube_map_projection_near_z�+  >       PerViewData.shadow_map_cube_data[12].cube_map_projection_far_z�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[0]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[2] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[3],  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[4] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[5]0,  ?       PerViewData.shadow_map_cube_data[13].cube_map_projection_near_z@,  >       PerViewData.shadow_map_cube_data[13].cube_map_projection_far_zD,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[0]P,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[1]`,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[2]p,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[4]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[5]�,  ?       PerViewData.shadow_map_cube_data[14].cube_map_projection_near_z�,  >       PerViewData.shadow_map_cube_data[14].cube_map_projection_far_z�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[0]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[1]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[2]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[4] -  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[5]-  ?       PerViewData.shadow_map_cube_data[15].cube_map_projection_near_z -  >       PerViewData.shadow_map_cube_data[15].cube_map_projection_far_z$-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[0]0-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[1]@-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[2]P-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[3]`-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[4]p-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[16].cube_map_projection_near_z�-  >       PerViewData.shadow_map_cube_data[16].cube_map_projection_far_z�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[0]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[1]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[2]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[3]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[4]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[17].cube_map_projection_near_z .  >       PerViewData.shadow_map_cube_data[17].cube_map_projection_far_z.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[0].  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[1] .  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[2]0.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[3]@.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[4]P.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[5]`.  ?       PerViewData.shadow_map_cube_data[18].cube_map_projection_near_zp.  >       PerViewData.shadow_map_cube_data[18].cube_map_projection_far_zt.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[1]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[2]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[3]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[4]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[5]�.  ?       PerViewData.shadow_map_cube_data[19].cube_map_projection_near_z�.  >       PerViewData.shadow_map_cube_data[19].cube_map_projection_far_z�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[1] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[2]/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[3] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[4]0/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[5]@/  ?       PerViewData.shadow_map_cube_data[20].cube_map_projection_near_zP/  >       PerViewData.shadow_map_cube_data[20].cube_map_projection_far_zT/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[0]`/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[1]p/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[3]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[4]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[5]�/  ?       PerViewData.shadow_map_cube_data[21].cube_map_projection_near_z�/  >       PerViewData.shadow_map_cube_data[21].cube_map_projection_far_z�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[0]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[1]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[3] 0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[4]0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[5] 0  ?       PerViewData.shadow_map_cube_data[22].cube_map_projection_near_z00  >       PerViewData.shadow_map_cube_data[22].cube_map_projection_far_z40  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[0]@0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[1]P0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[2]`0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[3]p0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[5]�0  ?       PerViewData.shadow_map_cube_data[23].cube_map_projection_near_z�0  >       PerViewData.shadow_map_cube_data[23].cube_map_projection_far_z�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[0]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[1]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[2]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[3]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[5] 1  ?       PerViewData.shadow_map_cube_data[24].cube_map_projection_near_z1  >       PerViewData.shadow_map_cube_data[24].cube_map_projection_far_z1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[0] 1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[1]01  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[2]@1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[3]P1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[4]`1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[5]p1  ?       PerViewData.shadow_map_cube_data[25].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[25].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[0]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[1]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[2]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[3]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[4]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[5]�1  ?       PerViewData.shadow_map_cube_data[26].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[26].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[0] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[1]2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[2] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[3]02  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[4]@2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[5]P2  ?       PerViewData.shadow_map_cube_data[27].cube_map_projection_near_z`2  >       PerViewData.shadow_map_cube_data[27].cube_map_projection_far_zd2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[0]p2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[2]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[3]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[4]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[5]�2  ?       PerViewData.shadow_map_cube_data[28].cube_map_projection_near_z�2  >       PerViewData.shadow_map_cube_data[28].cube_map_projection_far_z�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[0]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[2] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[3]3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[4] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[5]03  ?       PerViewData.shadow_map_cube_data[29].cube_map_projection_near_z@3  >       PerViewData.shadow_map_cube_data[29].cube_map_projection_far_zD3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[0]P3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[1]`3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[2]p3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[4]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[5]�3  ?       PerViewData.shadow_map_cube_data[30].cube_map_projection_near_z�3  >       PerViewData.shadow_map_cube_data[30].cube_map_projection_far_z�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[0]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[1]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[2]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[4] 4  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[5]4  ?       PerViewData.shadow_map_cube_data[31].cube_map_projection_near_z 4  >       PerViewData.shadow_map_cube_data[31].cube_map_projection_far_z$4                             smp              smp                                    smp_depth_linear              smp_depth_linear                                    smp_depth_nearest              smp_depth_nearest                                    shadow_map_atlas              shadow_map_atlas                                     LightBinOutput              light_bin_output                              	       AllLights       
       all_lights                                    ssao_texture              ssao_texture                                     AllTransforms              all_transforms                                    AllDrawData             all_draw_data                                     AllMaterials              all_materials                                   all_material_textures             all_material_textures                            main              �                            PerViewData               PerViewData �             PerViewData.view           PerViewData.view_proj@          PerViewData.ambient_light�          PerViewData.jitter_amount�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.mip_bias�          PerViewData.ndf_filter_amount�   #       PerViewData.directional_light_count�   "       PerViewData.use_clustered_lighting�   .       PerViewData.directional_lights[0].direction_ws�   +       PerViewData.directional_lights[0].intensity�   '       PerViewData.directional_lights[0].color�   .       PerViewData.directional_lights[0].direction_vs�   ,       PerViewData.directional_lights[0].shadow_map�   .       PerViewData.directional_lights[1].direction_ws�   +       PerViewData.directional_lights[1].intensity�   '       PerViewData.directional_lights[1].color�   .       PerViewData.directional_lights[1].direction_vs   ,       PerViewData.directional_lights[1].shadow_map  .       PerViewData.directional_lights[2].direction_ws  +       PerViewData.directional_lights[2].intensity  '       PerViewData.directional_lights[2].color   .       PerViewData.directional_lights[2].direction_vs0  ,       PerViewData.directional_lights[2].shadow_map<  .       PerViewData.directional_lights[3].direction_ws@  +       PerViewData.directional_lights[3].intensityL  '       PerViewData.directional_lights[3].colorP  .       PerViewData.directional_lights[3].direction_vs`  ,       PerViewData.directional_lights[3].shadow_mapl  .       PerViewData.directional_lights[4].direction_wsp  +       PerViewData.directional_lights[4].intensity|  '       PerViewData.directional_lights[4].color�  .       PerViewData.directional_lights[4].direction_vs�  ,       PerViewData.directional_lights[4].shadow_map�  .       PerViewData.directional_lights[5].direction_ws�  +       PerViewData.directional_lights[5].intensity�  '       PerViewData.directional_lights[5].color�  .       PerViewData.directional_lights[5].direction_vs�  ,       PerViewData.directional_lights[5].shadow_map�  .       PerViewData.directional_lights[6].direction_ws�  +       PerViewData.directional_lights[6].intensity�  '       PerViewData.directional_lights[6].color�  .       PerViewData.directional_lights[6].direction_vs�  ,       PerViewData.directional_lights[6].shadow_map�  .       PerViewData.directional_lights[7].direction_ws   +       PerViewData.directional_lights[7].intensity  '       PerViewData.directional_lights[7].color  .       PerViewData.directional_lights[7].direction_vs   ,       PerViewData.directional_lights[7].shadow_map,  (       PerViewData.shadow_map_2d_data[0].uv_min0  (       PerViewData.shadow_map_2d_data[0].uv_max8  6       PerViewData.shadow_map_2d_data[0].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[0].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[1].uv_min�  (       PerViewData.shadow_map_2d_data[1].uv_max�  6       PerViewData.shadow_map_2d_data[1].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[1].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[2].uv_min�  (       PerViewData.shadow_map_2d_data[2].uv_max�  6       PerViewData.shadow_map_2d_data[2].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[2].shadow_map_light_dir@  (       PerViewData.shadow_map_2d_data[3].uv_minP  (       PerViewData.shadow_map_2d_data[3].uv_maxX  6       PerViewData.shadow_map_2d_data[3].shadow_map_view_proj`  6       PerViewData.shadow_map_2d_data[3].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[4].uv_min�  (       PerViewData.shadow_map_2d_data[4].uv_max�  6       PerViewData.shadow_map_2d_data[4].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[4].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[5].uv_min  (       PerViewData.shadow_map_2d_data[5].uv_max  6       PerViewData.shadow_map_2d_data[5].shadow_map_view_proj   6       PerViewData.shadow_map_2d_data[5].shadow_map_light_dir`  (       PerViewData.shadow_map_2d_data[6].uv_minp  (       PerViewData.shadow_map_2d_data[6].uv_maxx  6       PerViewData.shadow_map_2d_data[6].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[6].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[7].uv_min�  (       PerViewData.shadow_map_2d_data[7].uv_max�  6       PerViewData.shadow_map_2d_data[7].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[7].shadow_map_light_dir   (       PerViewData.shadow_map_2d_data[8].uv_min0  (       PerViewData.shadow_map_2d_data[8].uv_max8  6       PerViewData.shadow_map_2d_data[8].shadow_map_view_proj@  6       PerViewData.shadow_map_2d_data[8].shadow_map_light_dir�  (       PerViewData.shadow_map_2d_data[9].uv_min�  (       PerViewData.shadow_map_2d_data[9].uv_max�  6       PerViewData.shadow_map_2d_data[9].shadow_map_view_proj�  6       PerViewData.shadow_map_2d_data[9].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[10].uv_min�  )       PerViewData.shadow_map_2d_data[10].uv_max�  7       PerViewData.shadow_map_2d_data[10].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[10].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[11].uv_minP  )       PerViewData.shadow_map_2d_data[11].uv_maxX  7       PerViewData.shadow_map_2d_data[11].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[11].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[12].uv_min�  )       PerViewData.shadow_map_2d_data[12].uv_max�  7       PerViewData.shadow_map_2d_data[12].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[12].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[13].uv_min  )       PerViewData.shadow_map_2d_data[13].uv_max  7       PerViewData.shadow_map_2d_data[13].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[13].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[14].uv_minp  )       PerViewData.shadow_map_2d_data[14].uv_maxx  7       PerViewData.shadow_map_2d_data[14].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[14].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[15].uv_min�  )       PerViewData.shadow_map_2d_data[15].uv_max�  7       PerViewData.shadow_map_2d_data[15].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[15].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[16].uv_min0  )       PerViewData.shadow_map_2d_data[16].uv_max8  7       PerViewData.shadow_map_2d_data[16].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[16].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[17].uv_min�  )       PerViewData.shadow_map_2d_data[17].uv_max�  7       PerViewData.shadow_map_2d_data[17].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[17].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[18].uv_min�  )       PerViewData.shadow_map_2d_data[18].uv_max�  7       PerViewData.shadow_map_2d_data[18].shadow_map_view_proj 	  7       PerViewData.shadow_map_2d_data[18].shadow_map_light_dir@	  )       PerViewData.shadow_map_2d_data[19].uv_minP	  )       PerViewData.shadow_map_2d_data[19].uv_maxX	  7       PerViewData.shadow_map_2d_data[19].shadow_map_view_proj`	  7       PerViewData.shadow_map_2d_data[19].shadow_map_light_dir�	  )       PerViewData.shadow_map_2d_data[20].uv_min�	  )       PerViewData.shadow_map_2d_data[20].uv_max�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_view_proj�	  7       PerViewData.shadow_map_2d_data[20].shadow_map_light_dir 
  )       PerViewData.shadow_map_2d_data[21].uv_min
  )       PerViewData.shadow_map_2d_data[21].uv_max
  7       PerViewData.shadow_map_2d_data[21].shadow_map_view_proj 
  7       PerViewData.shadow_map_2d_data[21].shadow_map_light_dir`
  )       PerViewData.shadow_map_2d_data[22].uv_minp
  )       PerViewData.shadow_map_2d_data[22].uv_maxx
  7       PerViewData.shadow_map_2d_data[22].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[22].shadow_map_light_dir�
  )       PerViewData.shadow_map_2d_data[23].uv_min�
  )       PerViewData.shadow_map_2d_data[23].uv_max�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_view_proj�
  7       PerViewData.shadow_map_2d_data[23].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[24].uv_min0  )       PerViewData.shadow_map_2d_data[24].uv_max8  7       PerViewData.shadow_map_2d_data[24].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[24].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[25].uv_min�  )       PerViewData.shadow_map_2d_data[25].uv_max�  7       PerViewData.shadow_map_2d_data[25].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[25].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[26].uv_min�  )       PerViewData.shadow_map_2d_data[26].uv_max�  7       PerViewData.shadow_map_2d_data[26].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[26].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[27].uv_minP  )       PerViewData.shadow_map_2d_data[27].uv_maxX  7       PerViewData.shadow_map_2d_data[27].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[27].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[28].uv_min�  )       PerViewData.shadow_map_2d_data[28].uv_max�  7       PerViewData.shadow_map_2d_data[28].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[28].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[29].uv_min  )       PerViewData.shadow_map_2d_data[29].uv_max  7       PerViewData.shadow_map_2d_data[29].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[29].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[30].uv_minp  )       PerViewData.shadow_map_2d_data[30].uv_maxx  7       PerViewData.shadow_map_2d_data[30].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[30].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[31].uv_min�  )       PerViewData.shadow_map_2d_data[31].uv_max�  7       PerViewData.shadow_map_2d_data[31].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[31].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[32].uv_min0  )       PerViewData.shadow_map_2d_data[32].uv_max8  7       PerViewData.shadow_map_2d_data[32].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[32].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[33].uv_min�  )       PerViewData.shadow_map_2d_data[33].uv_max�  7       PerViewData.shadow_map_2d_data[33].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[33].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[34].uv_min�  )       PerViewData.shadow_map_2d_data[34].uv_max�  7       PerViewData.shadow_map_2d_data[34].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[34].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[35].uv_minP  )       PerViewData.shadow_map_2d_data[35].uv_maxX  7       PerViewData.shadow_map_2d_data[35].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[35].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[36].uv_min�  )       PerViewData.shadow_map_2d_data[36].uv_max�  7       PerViewData.shadow_map_2d_data[36].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[36].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[37].uv_min  )       PerViewData.shadow_map_2d_data[37].uv_max  7       PerViewData.shadow_map_2d_data[37].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[37].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[38].uv_minp  )       PerViewData.shadow_map_2d_data[38].uv_maxx  7       PerViewData.shadow_map_2d_data[38].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[38].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[39].uv_min�  )       PerViewData.shadow_map_2d_data[39].uv_max�  7       PerViewData.shadow_map_2d_data[39].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[39].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[40].uv_min0  )       PerViewData.shadow_map_2d_data[40].uv_max8  7       PerViewData.shadow_map_2d_data[40].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[40].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[41].uv_min�  )       PerViewData.shadow_map_2d_data[41].uv_max�  7       PerViewData.shadow_map_2d_data[41].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[41].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[42].uv_min�  )       PerViewData.shadow_map_2d_data[42].uv_max�  7       PerViewData.shadow_map_2d_data[42].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[42].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[43].uv_minP  )       PerViewData.shadow_map_2d_data[43].uv_maxX  7       PerViewData.shadow_map_2d_data[43].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[43].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[44].uv_min�  )       PerViewData.shadow_map_2d_data[44].uv_max�  7       PerViewData.shadow_map_2d_data[44].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[44].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[45].uv_min  )       PerViewData.shadow_map_2d_data[45].uv_max  7       PerViewData.shadow_map_2d_data[45].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[45].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[46].uv_minp  )       PerViewData.shadow_map_2d_data[46].uv_maxx  7       PerViewData.shadow_map_2d_data[46].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[46].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[47].uv_min�  )       PerViewData.shadow_map_2d_data[47].uv_max�  7       PerViewData.shadow_map_2d_data[47].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[47].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[48].uv_min0  )       PerViewData.shadow_map_2d_data[48].uv_max8  7       PerViewData.shadow_map_2d_data[48].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[48].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[49].uv_min�  )       PerViewData.shadow_map_2d_data[49].uv_max�  7       PerViewData.shadow_map_2d_data[49].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[49].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[50].uv_min�  )       PerViewData.shadow_map_2d_data[50].uv_max�  7       PerViewData.shadow_map_2d_data[50].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[50].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[51].uv_minP  )       PerViewData.shadow_map_2d_data[51].uv_maxX  7       PerViewData.shadow_map_2d_data[51].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[51].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[52].uv_min�  )       PerViewData.shadow_map_2d_data[52].uv_max�  7       PerViewData.shadow_map_2d_data[52].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[52].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[53].uv_min  )       PerViewData.shadow_map_2d_data[53].uv_max  7       PerViewData.shadow_map_2d_data[53].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[53].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[54].uv_minp  )       PerViewData.shadow_map_2d_data[54].uv_maxx  7       PerViewData.shadow_map_2d_data[54].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[54].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[55].uv_min�  )       PerViewData.shadow_map_2d_data[55].uv_max�  7       PerViewData.shadow_map_2d_data[55].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[55].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[56].uv_min0  )       PerViewData.shadow_map_2d_data[56].uv_max8  7       PerViewData.shadow_map_2d_data[56].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[56].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[57].uv_min�  )       PerViewData.shadow_map_2d_data[57].uv_max�  7       PerViewData.shadow_map_2d_data[57].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[57].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[58].uv_min�  )       PerViewData.shadow_map_2d_data[58].uv_max�  7       PerViewData.shadow_map_2d_data[58].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[58].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[59].uv_minP  )       PerViewData.shadow_map_2d_data[59].uv_maxX  7       PerViewData.shadow_map_2d_data[59].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[59].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[60].uv_min�  )       PerViewData.shadow_map_2d_data[60].uv_max�  7       PerViewData.shadow_map_2d_data[60].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[60].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[61].uv_min  )       PerViewData.shadow_map_2d_data[61].uv_max  7       PerViewData.shadow_map_2d_data[61].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[61].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[62].uv_minp  )       PerViewData.shadow_map_2d_data[62].uv_maxx  7       PerViewData.shadow_map_2d_data[62].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[62].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[63].uv_min�  )       PerViewData.shadow_map_2d_data[63].uv_max�  7       PerViewData.shadow_map_2d_data[63].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[63].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[64].uv_min0  )       PerViewData.shadow_map_2d_data[64].uv_max8  7       PerViewData.shadow_map_2d_data[64].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[64].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[65].uv_min�  )       PerViewData.shadow_map_2d_data[65].uv_max�  7       PerViewData.shadow_map_2d_data[65].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[65].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[66].uv_min�  )       PerViewData.shadow_map_2d_data[66].uv_max�  7       PerViewData.shadow_map_2d_data[66].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[66].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[67].uv_minP  )       PerViewData.shadow_map_2d_data[67].uv_maxX  7       PerViewData.shadow_map_2d_data[67].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[67].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[68].uv_min�  )       PerViewData.shadow_map_2d_data[68].uv_max�  7       PerViewData.shadow_map_2d_data[68].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[68].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[69].uv_min  )       PerViewData.shadow_map_2d_data[69].uv_max  7       PerViewData.shadow_map_2d_data[69].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[69].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[70].uv_minp  )       PerViewData.shadow_map_2d_data[70].uv_maxx  7       PerViewData.shadow_map_2d_data[70].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[70].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[71].uv_min�  )       PerViewData.shadow_map_2d_data[71].uv_max�  7       PerViewData.shadow_map_2d_data[71].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[71].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[72].uv_min0  )       PerViewData.shadow_map_2d_data[72].uv_max8  7       PerViewData.shadow_map_2d_data[72].shadow_map_view_proj@  7       PerViewData.shadow_map_2d_data[72].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[73].uv_min�  )       PerViewData.shadow_map_2d_data[73].uv_max�  7       PerViewData.shadow_map_2d_data[73].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[73].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[74].uv_min�  )       PerViewData.shadow_map_2d_data[74].uv_max�  7       PerViewData.shadow_map_2d_data[74].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[74].shadow_map_light_dir@  )       PerViewData.shadow_map_2d_data[75].uv_minP  )       PerViewData.shadow_map_2d_data[75].uv_maxX  7       PerViewData.shadow_map_2d_data[75].shadow_map_view_proj`  7       PerViewData.shadow_map_2d_data[75].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[76].uv_min�  )       PerViewData.shadow_map_2d_data[76].uv_max�  7       PerViewData.shadow_map_2d_data[76].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[76].shadow_map_light_dir   )       PerViewData.shadow_map_2d_data[77].uv_min  )       PerViewData.shadow_map_2d_data[77].uv_max  7       PerViewData.shadow_map_2d_data[77].shadow_map_view_proj   7       PerViewData.shadow_map_2d_data[77].shadow_map_light_dir`  )       PerViewData.shadow_map_2d_data[78].uv_minp  )       PerViewData.shadow_map_2d_data[78].uv_maxx  7       PerViewData.shadow_map_2d_data[78].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[78].shadow_map_light_dir�  )       PerViewData.shadow_map_2d_data[79].uv_min�  )       PerViewData.shadow_map_2d_data[79].uv_max�  7       PerViewData.shadow_map_2d_data[79].shadow_map_view_proj�  7       PerViewData.shadow_map_2d_data[79].shadow_map_light_dir    )       PerViewData.shadow_map_2d_data[80].uv_min0   )       PerViewData.shadow_map_2d_data[80].uv_max8   7       PerViewData.shadow_map_2d_data[80].shadow_map_view_proj@   7       PerViewData.shadow_map_2d_data[80].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[81].uv_min�   )       PerViewData.shadow_map_2d_data[81].uv_max�   7       PerViewData.shadow_map_2d_data[81].shadow_map_view_proj�   7       PerViewData.shadow_map_2d_data[81].shadow_map_light_dir�   )       PerViewData.shadow_map_2d_data[82].uv_min�   )       PerViewData.shadow_map_2d_data[82].uv_max�   7       PerViewData.shadow_map_2d_data[82].shadow_map_view_proj !  7       PerViewData.shadow_map_2d_data[82].shadow_map_light_dir@!  )       PerViewData.shadow_map_2d_data[83].uv_minP!  )       PerViewData.shadow_map_2d_data[83].uv_maxX!  7       PerViewData.shadow_map_2d_data[83].shadow_map_view_proj`!  7       PerViewData.shadow_map_2d_data[83].shadow_map_light_dir�!  )       PerViewData.shadow_map_2d_data[84].uv_min�!  )       PerViewData.shadow_map_2d_data[84].uv_max�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_view_proj�!  7       PerViewData.shadow_map_2d_data[84].shadow_map_light_dir "  )       PerViewData.shadow_map_2d_data[85].uv_min"  )       PerViewData.shadow_map_2d_data[85].uv_max"  7       PerViewData.shadow_map_2d_data[85].shadow_map_view_proj "  7       PerViewData.shadow_map_2d_data[85].shadow_map_light_dir`"  )       PerViewData.shadow_map_2d_data[86].uv_minp"  )       PerViewData.shadow_map_2d_data[86].uv_maxx"  7       PerViewData.shadow_map_2d_data[86].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[86].shadow_map_light_dir�"  )       PerViewData.shadow_map_2d_data[87].uv_min�"  )       PerViewData.shadow_map_2d_data[87].uv_max�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_view_proj�"  7       PerViewData.shadow_map_2d_data[87].shadow_map_light_dir #  )       PerViewData.shadow_map_2d_data[88].uv_min0#  )       PerViewData.shadow_map_2d_data[88].uv_max8#  7       PerViewData.shadow_map_2d_data[88].shadow_map_view_proj@#  7       PerViewData.shadow_map_2d_data[88].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[89].uv_min�#  )       PerViewData.shadow_map_2d_data[89].uv_max�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_view_proj�#  7       PerViewData.shadow_map_2d_data[89].shadow_map_light_dir�#  )       PerViewData.shadow_map_2d_data[90].uv_min�#  )       PerViewData.shadow_map_2d_data[90].uv_max�#  7       PerViewData.shadow_map_2d_data[90].shadow_map_view_proj $  7       PerViewData.shadow_map_2d_data[90].shadow_map_light_dir@$  )       PerViewData.shadow_map_2d_data[91].uv_minP$  )       PerViewData.shadow_map_2d_data[91].uv_maxX$  7       PerViewData.shadow_map_2d_data[91].shadow_map_view_proj`$  7       PerViewData.shadow_map_2d_data[91].shadow_map_light_dir�$  )       PerViewData.shadow_map_2d_data[92].uv_min�$  )       PerViewData.shadow_map_2d_data[92].uv_max�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_view_proj�$  7       PerViewData.shadow_map_2d_data[92].shadow_map_light_dir %  )       PerViewData.shadow_map_2d_data[93].uv_min%  )       PerViewData.shadow_map_2d_data[93].uv_max%  7       PerViewData.shadow_map_2d_data[93].shadow_map_view_proj %  7       PerViewData.shadow_map_2d_data[93].shadow_map_light_dir`%  )       PerViewData.shadow_map_2d_data[94].uv_minp%  )       PerViewData.shadow_map_2d_data[94].uv_maxx%  7       PerViewData.shadow_map_2d_data[94].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[94].shadow_map_light_dir�%  )       PerViewData.shadow_map_2d_data[95].uv_min�%  )       PerViewData.shadow_map_2d_data[95].uv_max�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_view_proj�%  7       PerViewData.shadow_map_2d_data[95].shadow_map_light_dir &  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[0]0&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[1]@&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[2]P&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[3]`&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[4]p&  4       PerViewData.shadow_map_cube_data[0].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[0].cube_map_projection_near_z�&  =       PerViewData.shadow_map_cube_data[0].cube_map_projection_far_z�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[0]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[1]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[2]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[3]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[4]�&  4       PerViewData.shadow_map_cube_data[1].uv_min_uv_max[5]�&  >       PerViewData.shadow_map_cube_data[1].cube_map_projection_near_z '  =       PerViewData.shadow_map_cube_data[1].cube_map_projection_far_z'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[0]'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[1] '  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[2]0'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[3]@'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[4]P'  4       PerViewData.shadow_map_cube_data[2].uv_min_uv_max[5]`'  >       PerViewData.shadow_map_cube_data[2].cube_map_projection_near_zp'  =       PerViewData.shadow_map_cube_data[2].cube_map_projection_far_zt'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[1]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[2]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[3]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[4]�'  4       PerViewData.shadow_map_cube_data[3].uv_min_uv_max[5]�'  >       PerViewData.shadow_map_cube_data[3].cube_map_projection_near_z�'  =       PerViewData.shadow_map_cube_data[3].cube_map_projection_far_z�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[0]�'  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[1] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[2](  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[3] (  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[4]0(  4       PerViewData.shadow_map_cube_data[4].uv_min_uv_max[5]@(  >       PerViewData.shadow_map_cube_data[4].cube_map_projection_near_zP(  =       PerViewData.shadow_map_cube_data[4].cube_map_projection_far_zT(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[0]`(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[1]p(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[3]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[4]�(  4       PerViewData.shadow_map_cube_data[5].uv_min_uv_max[5]�(  >       PerViewData.shadow_map_cube_data[5].cube_map_projection_near_z�(  =       PerViewData.shadow_map_cube_data[5].cube_map_projection_far_z�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[0]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[1]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[2]�(  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[3] )  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[4])  4       PerViewData.shadow_map_cube_data[6].uv_min_uv_max[5] )  >       PerViewData.shadow_map_cube_data[6].cube_map_projection_near_z0)  =       PerViewData.shadow_map_cube_data[6].cube_map_projection_far_z4)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[0]@)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[1]P)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[2]`)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[3]p)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[7].uv_min_uv_max[5]�)  >       PerViewData.shadow_map_cube_data[7].cube_map_projection_near_z�)  =       PerViewData.shadow_map_cube_data[7].cube_map_projection_far_z�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[0]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[1]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[2]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[3]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[4]�)  4       PerViewData.shadow_map_cube_data[8].uv_min_uv_max[5] *  >       PerViewData.shadow_map_cube_data[8].cube_map_projection_near_z*  =       PerViewData.shadow_map_cube_data[8].cube_map_projection_far_z*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[0] *  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[1]0*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[2]@*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[3]P*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[4]`*  4       PerViewData.shadow_map_cube_data[9].uv_min_uv_max[5]p*  >       PerViewData.shadow_map_cube_data[9].cube_map_projection_near_z�*  =       PerViewData.shadow_map_cube_data[9].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[0]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[1]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[2]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[3]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[4]�*  5       PerViewData.shadow_map_cube_data[10].uv_min_uv_max[5]�*  ?       PerViewData.shadow_map_cube_data[10].cube_map_projection_near_z�*  >       PerViewData.shadow_map_cube_data[10].cube_map_projection_far_z�*  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[0] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[1]+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[2] +  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[3]0+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[4]@+  5       PerViewData.shadow_map_cube_data[11].uv_min_uv_max[5]P+  ?       PerViewData.shadow_map_cube_data[11].cube_map_projection_near_z`+  >       PerViewData.shadow_map_cube_data[11].cube_map_projection_far_zd+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[0]p+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[2]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[3]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[4]�+  5       PerViewData.shadow_map_cube_data[12].uv_min_uv_max[5]�+  ?       PerViewData.shadow_map_cube_data[12].cube_map_projection_near_z�+  >       PerViewData.shadow_map_cube_data[12].cube_map_projection_far_z�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[0]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[1]�+  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[2] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[3],  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[4] ,  5       PerViewData.shadow_map_cube_data[13].uv_min_uv_max[5]0,  ?       PerViewData.shadow_map_cube_data[13].cube_map_projection_near_z@,  >       PerViewData.shadow_map_cube_data[13].cube_map_projection_far_zD,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[0]P,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[1]`,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[2]p,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[4]�,  5       PerViewData.shadow_map_cube_data[14].uv_min_uv_max[5]�,  ?       PerViewData.shadow_map_cube_data[14].cube_map_projection_near_z�,  >       PerViewData.shadow_map_cube_data[14].cube_map_projection_far_z�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[0]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[1]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[2]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[3]�,  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[4] -  5       PerViewData.shadow_map_cube_data[15].uv_min_uv_max[5]-  ?       PerViewData.shadow_map_cube_data[15].cube_map_projection_near_z -  >       PerViewData.shadow_map_cube_data[15].cube_map_projection_far_z$-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[0]0-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[1]@-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[2]P-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[3]`-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[4]p-  5       PerViewData.shadow_map_cube_data[16].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[16].cube_map_projection_near_z�-  >       PerViewData.shadow_map_cube_data[16].cube_map_projection_far_z�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[0]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[1]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[2]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[3]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[4]�-  5       PerViewData.shadow_map_cube_data[17].uv_min_uv_max[5]�-  ?       PerViewData.shadow_map_cube_data[17].cube_map_projection_near_z .  >       PerViewData.shadow_map_cube_data[17].cube_map_projection_far_z.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[0].  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[1] .  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[2]0.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[3]@.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[4]P.  5       PerViewData.shadow_map_cube_data[18].uv_min_uv_max[5]`.  ?       PerViewData.shadow_map_cube_data[18].cube_map_projection_near_zp.  >       PerViewData.shadow_map_cube_data[18].cube_map_projection_far_zt.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[1]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[2]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[3]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[4]�.  5       PerViewData.shadow_map_cube_data[19].uv_min_uv_max[5]�.  ?       PerViewData.shadow_map_cube_data[19].cube_map_projection_near_z�.  >       PerViewData.shadow_map_cube_data[19].cube_map_projection_far_z�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[0]�.  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[1] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[2]/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[3] /  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[4]0/  5       PerViewData.shadow_map_cube_data[20].uv_min_uv_max[5]@/  ?       PerViewData.shadow_map_cube_data[20].cube_map_projection_near_zP/  >       PerViewData.shadow_map_cube_data[20].cube_map_projection_far_zT/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[0]`/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[1]p/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[3]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[4]�/  5       PerViewData.shadow_map_cube_data[21].uv_min_uv_max[5]�/  ?       PerViewData.shadow_map_cube_data[21].cube_map_projection_near_z�/  >       PerViewData.shadow_map_cube_data[21].cube_map_projection_far_z�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[0]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[1]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[2]�/  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[3] 0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[4]0  5       PerViewData.shadow_map_cube_data[22].uv_min_uv_max[5] 0  ?       PerViewData.shadow_map_cube_data[22].cube_map_projection_near_z00  >       PerViewData.shadow_map_cube_data[22].cube_map_projection_far_z40  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[0]@0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[1]P0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[2]`0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[3]p0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[23].uv_min_uv_max[5]�0  ?       PerViewData.shadow_map_cube_data[23].cube_map_projection_near_z�0  >       PerViewData.shadow_map_cube_data[23].cube_map_projection_far_z�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[0]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[1]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[2]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[3]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[4]�0  5       PerViewData.shadow_map_cube_data[24].uv_min_uv_max[5] 1  ?       PerViewData.shadow_map_cube_data[24].cube_map_projection_near_z1  >       PerViewData.shadow_map_cube_data[24].cube_map_projection_far_z1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[0] 1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[1]01  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[2]@1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[3]P1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[4]`1  5       PerViewData.shadow_map_cube_data[25].uv_min_uv_max[5]p1  ?       PerViewData.shadow_map_cube_data[25].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[25].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[0]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[1]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[2]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[3]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[4]�1  5       PerViewData.shadow_map_cube_data[26].uv_min_uv_max[5]�1  ?       PerViewData.shadow_map_cube_data[26].cube_map_projection_near_z�1  >       PerViewData.shadow_map_cube_data[26].cube_map_projection_far_z�1  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[0] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[1]2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[2] 2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[3]02  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[4]@2  5       PerViewData.shadow_map_cube_data[27].uv_min_uv_max[5]P2  ?       PerViewData.shadow_map_cube_data[27].cube_map_projection_near_z`2  >       PerViewData.shadow_map_cube_data[27].cube_map_projection_far_zd2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[0]p2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[2]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[3]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[4]�2  5       PerViewData.shadow_map_cube_data[28].uv_min_uv_max[5]�2  ?       PerViewData.shadow_map_cube_data[28].cube_map_projection_near_z�2  >       PerViewData.shadow_map_cube_data[28].cube_map_projection_far_z�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[0]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[1]�2  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[2] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[3]3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[4] 3  5       PerViewData.shadow_map_cube_data[29].uv_min_uv_max[5]03  ?       PerViewData.shadow_map_cube_data[29].cube_map_projection_near_z@3  >       PerViewData.shadow_map_cube_data[29].cube_map_projection_far_zD3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[0]P3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[1]`3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[2]p3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[4]�3  5       PerViewData.shadow_map_cube_data[30].uv_min_uv_max[5]�3  ?       PerViewData.shadow_map_cube_data[30].cube_map_projection_near_z�3  >       PerViewData.shadow_map_cube_data[30].cube_map_projection_far_z�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[0]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[1]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[2]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[3]�3  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[4] 4  5       PerViewData.shadow_map_cube_data[31].uv_min_uv_max[5]4  ?       PerViewData.shadow_map_cube_data[31].cube_map_projection_near_z 4  >       PerViewData.shadow_map_cube_data[31].cube_map_projection_far_z$4   04                             smp              smp                                        �A                                smp_depth_linear              smp_depth_linear                                        �?                               smp_depth_nearest              smp_depth_nearest                                           �?                               shadow_map_atlas              shadow_map_atlas                                       LightBinOutput              light_bin_output                                	       AllLights       
       all_lights                                             ssao_texture              ssao_texture                                              AllTransforms              all_transforms                                      AllDrawData             all_draw_data                                              AllMaterials              all_materials                                     all_material_textures             all_material_textures                            mesh_adv_textured.frag