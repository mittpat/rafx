                       �""D@"���Ȟq��-�I      x      static const uint3 gl_WorkGroupSize = uint3(16u, 16u, 1u);

cbuffer BuildHistogramConfig : register(b0, space0)
{
    uint config_input_width : packoffset(c0);
    uint config_input_height : packoffset(c0.y);
    float config_min_log_luma : packoffset(c0.z);
    float config_one_over_log_luma_range : packoffset(c0.w);
};

RWByteAddressBuffer histogram_data : register(u3, space0);
Texture2D<float4> tex : register(t1, space0);
SamplerState smp : register(s2, space0);

static uint3 gl_GlobalInvocationID;
static uint gl_LocalInvocationIndex;
struct SPIRV_Cross_Input
{
    uint3 gl_GlobalInvocationID : SV_DispatchThreadID;
    uint gl_LocalInvocationIndex : SV_GroupIndex;
};

groupshared uint HistogramShared[256];

uint color_to_bin(float3 color)
{
    float luminance = dot(color, float3(0.2125999927520751953125f, 0.715200006961822509765625f, 0.072200000286102294921875f));
    if (luminance < 0.000750000006519258022308349609375f)
    {
        return 0u;
    }
    float log_luminance = clamp((log2(luminance) - config_min_log_luma) * config_one_over_log_luma_range, 0.0f, 1.0f);
    return uint((log_luminance * 254.0f) + 1.0f);
}

void comp_main()
{
    HistogramShared[gl_LocalInvocationIndex] = 0u;
    GroupMemoryBarrierWithGroupSync();
    bool _77 = gl_GlobalInvocationID.x < config_input_width;
    bool _87;
    if (_77)
    {
        _87 = gl_GlobalInvocationID.y < config_input_height;
    }
    else
    {
        _87 = _77;
    }
    if (_87)
    {
        float3 c = tex.SampleLevel(smp, float2(gl_GlobalInvocationID.xy) / float2(float(config_input_width), float(config_input_height)), 0.0f).xyz;
        float3 param = c;
        uint bin_index = color_to_bin(param);
        uint _124;
        InterlockedAdd(HistogramShared[bin_index], 1u, _124);
    }
    GroupMemoryBarrierWithGroupSync();
    uint _134;
    histogram_data.InterlockedAdd(gl_LocalInvocationIndex * 4 + 0, HistogramShared[gl_LocalInvocationIndex], _134);
}

[numthreads(16, 16, 1)]
void main(SPIRV_Cross_Input stage_input)
{
    gl_GlobalInvocationID = stage_input.gl_GlobalInvocationID;
    gl_LocalInvocationIndex = stage_input.gl_LocalInvocationIndex;
    comp_main();
}
    
      #pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wunused-variable"

#include <metal_stdlib>
#include <simd/simd.h>
#include <metal_atomic>

using namespace metal;

struct BuildHistogramConfig
{
    uint input_width;
    uint input_height;
    float min_log_luma;
    float one_over_log_luma_range;
};

struct HistogramData
{
    uint data[256];
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(16u, 16u, 1u);

struct spvDescriptorSetBuffer0
{
    constant BuildHistogramConfig* config [[id(0)]];
    texture2d<float> tex [[id(1)]];
    device HistogramData* histogram_data [[id(3)]];
};

static inline __attribute__((always_inline))
uint color_to_bin(thread const float3& color, constant BuildHistogramConfig& config)
{
    float luminance = dot(color, float3(0.2125999927520751953125, 0.715200006961822509765625, 0.072200000286102294921875));
    if (luminance < 0.000750000006519258022308349609375)
    {
        return 0u;
    }
    float log_luminance = fast::clamp((log2(luminance) - config.min_log_luma) * config.one_over_log_luma_range, 0.0, 1.0);
    return uint((log_luminance * 254.0) + 1.0);
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint gl_LocalInvocationIndex [[thread_index_in_threadgroup]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    constexpr sampler smp(mip_filter::linear, compare_func::never, max_anisotropy(1));
    threadgroup uint HistogramShared[256];
    HistogramShared[gl_LocalInvocationIndex] = 0u;
    threadgroup_barrier(mem_flags::mem_threadgroup);
    bool _77 = gl_GlobalInvocationID.x < (*spvDescriptorSet0.config).input_width;
    bool _87;
    if (_77)
    {
        _87 = gl_GlobalInvocationID.y < (*spvDescriptorSet0.config).input_height;
    }
    else
    {
        _87 = _77;
    }
    if (_87)
    {
        float3 c = spvDescriptorSet0.tex.sample(smp, (float2(gl_GlobalInvocationID.xy) / float2(float((*spvDescriptorSet0.config).input_width), float((*spvDescriptorSet0.config).input_height))), level(0.0)).xyz;
        float3 param = c;
        uint bin_index = color_to_bin(param, (*spvDescriptorSet0.config));
        uint _124 = atomic_fetch_add_explicit((threadgroup atomic_uint*)&HistogramShared[bin_index], 1u, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    uint _134 = atomic_fetch_add_explicit((device atomic_uint*)&(*spvDescriptorSet0.histogram_data).data[gl_LocalInvocationIndex], HistogramShared[gl_LocalInvocationIndex], memory_order_relaxed);
}

    �	      #     �                 GLSL.std.450                     main    =   E                    H  !       #       H  !      #      H  !      #      H  !      #      G  !      G  #   "       G  #   !       G  =         G  E         G  \   "       G  \   !      G  `   "       G  `   !      G  |         H  }       #       G  }      G     "       G     !      G  �              !                              	           +        гY>+        Y7?+        �ݓ=,                 +        ��D:     +  	            !   	   	            "      !   ;  "   #        $          +  $   %         &         +  $   *      +     .       +     /     �?+     2     ~C+  	   8        9   	   8      :      9   ;  :   ;         <      	   ;  <   =         ?      	   +  	   A      +  	   B       C   	         D      C   ;  D   E      +  $   H          I      	   +  	   O      +  $   R       	 Z                               [       Z   ;  [   \         ^      _       ^   ;  _   `         b   Z     d   	        g           q           |   	   8     }   |      ~      }   ;  ~         +  	   �      ,  C   �   �   �   O   6               �     =  	   >   =   A  ?   @   ;   >   >  @      �  A   A   B   A  <   F   E      =  	   G   F   A  I   J   #   H   =  	   K   J   �     L   G   K   �  N       �  L   M   N   �  M   A  <   P   E   O   =  	   Q   P   A  I   S   #   R   =  	   T   S   �     U   Q   T   �  N   �  N   �     V   L      U   M   �  X       �  V   W   X   �  W   =  Z   ]   \   =  ^   a   `   V  b   c   ]   a   =  C   e   E   O  d   f   e   e          p  g   h   f   p     k   K   A  I   l   #   R   =  	   m   l   p     n   m   P  g   o   k   n   �  g   p   h   o   X  q   r   c   p      .   O     s   r   r             �  �       �     �   �  �   �     �   s      �     �   �      �  �       �  �   �   �   �  �   �  �   �  �        �         �   A  &   �   #   %   =     �   �   �     �   �   �   A  &   �   #   *   =     �   �   �     �   �   �        �      +   �   .   /        �      2   �   2   /   m  	   �   �   �  �   �  �   �  	   �      �   �   �   A  ?   z   ;   �   �  	   {   z   O      O   �  X   �  X   �  A   A   B   A  I   �      H   >   =  	   �   @   �  	   �   �   O      �   �  8                    �                             BuildHistogramConfig               BuildHistogramConfig                BuildHistogramConfig.input_width    !       BuildHistogramConfig.input_height   !       BuildHistogramConfig.min_log_luma   ,       BuildHistogramConfig.one_over_log_luma_range                               tex              tex                                     smp              smp                                      HistogramData              histogram_data                         main              �                             BuildHistogramConfig               BuildHistogramConfig                BuildHistogramConfig.input_width    !       BuildHistogramConfig.input_height   !       BuildHistogramConfig.min_log_luma   ,       BuildHistogramConfig.one_over_log_luma_range                                   tex              tex                                       smp              smp                                                                              HistogramData              histogram_data                                     �                             BuildHistogramConfig               BuildHistogramConfig                BuildHistogramConfig.input_width    !       BuildHistogramConfig.input_height   !       BuildHistogramConfig.min_log_luma   ,       BuildHistogramConfig.one_over_log_luma_range                               tex              tex                                     smp              smp                                      HistogramData              histogram_data                         main              �                             BuildHistogramConfig               BuildHistogramConfig                BuildHistogramConfig.input_width    !       BuildHistogramConfig.input_height   !       BuildHistogramConfig.min_log_luma   ,       BuildHistogramConfig.one_over_log_luma_range                                   tex              tex                                       smp              smp                                                                              HistogramData              histogram_data                                     �                             BuildHistogramConfig               BuildHistogramConfig                BuildHistogramConfig.input_width    !       BuildHistogramConfig.input_height   !       BuildHistogramConfig.min_log_luma   ,       BuildHistogramConfig.one_over_log_luma_range                               tex              tex                                     smp              smp                                      HistogramData              histogram_data                         main              �                             BuildHistogramConfig               BuildHistogramConfig                BuildHistogramConfig.input_width    !       BuildHistogramConfig.input_height   !       BuildHistogramConfig.min_log_luma   ,       BuildHistogramConfig.one_over_log_luma_range                                   tex              tex                                       smp              smp                                                                              HistogramData              histogram_data                            luma_build_histogram.comp