                       �""D@"���Ȟ��W+��o      g      static const uint3 gl_WorkGroupSize = uint3(16u, 16u, 1u);

cbuffer DepthPyramidConfig : register(b0, space0)
{
    uint config_input_width : packoffset(c0);
    uint config_input_height : packoffset(c0.y);
    uint config_odd_width : packoffset(c0.z);
    uint config_odd_height : packoffset(c0.w);
};

Texture2D<float4> src_depth_tex : register(t1, space0);
SamplerState smp : register(s2, space0);
RWTexture2D<float4> dst_depth_tex : register(u3, space0);

static uint3 gl_GlobalInvocationID;
struct SPIRV_Cross_Input
{
    uint3 gl_GlobalInvocationID : SV_DispatchThreadID;
};

void comp_main()
{
    if (gl_GlobalInvocationID.x >= (config_input_width / 2u))
    {
        return;
    }
    if (gl_GlobalInvocationID.y >= (config_input_height / 2u))
    {
        return;
    }
    float2 texel_size = float2(1.0f / float(config_input_width), 1.0f / float(config_input_height));
    float2 src_uv = (float2(gl_GlobalInvocationID.xy * uint2(2u, 2u)) + 0.5f.xx) * texel_size;
    float4 gathered = src_depth_tex.GatherRed(smp, src_uv);
    float min_value = min(min(gathered.x, gathered.y), min(gathered.z, gathered.w));
    if (config_odd_width != 0u)
    {
        float a = src_depth_tex.SampleLevel(smp, src_uv + (float2(2.0f, 0.0f) * texel_size), 0.0f).x;
        float b = src_depth_tex.SampleLevel(smp, src_uv + (float2(2.0f, 1.0f) * texel_size), 0.0f).x;
        min_value = min(min_value, min(a, b));
    }
    if (config_odd_height != 0u)
    {
        float a_1 = src_depth_tex.SampleLevel(smp, src_uv + (float2(0.0f, 2.0f) * texel_size), 0.0f).x;
        float b_1 = src_depth_tex.SampleLevel(smp, src_uv + (float2(1.0f, 2.0f) * texel_size), 0.0f).x;
        min_value = min(min_value, min(a_1, b_1));
    }
    if ((config_odd_width != 0u) && (config_odd_height != 0u))
    {
        float a_2 = src_depth_tex.SampleLevel(smp, src_uv + (2.0f.xx * texel_size), 0.0f).x;
        min_value = min(min_value, a_2);
    }
    dst_depth_tex[int2(gl_GlobalInvocationID.xy)] = min_value.xxxx;
}

[numthreads(16, 16, 1)]
void main(SPIRV_Cross_Input stage_input)
{
    gl_GlobalInvocationID = stage_input.gl_GlobalInvocationID;
    comp_main();
}
    �	      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct DepthPyramidConfig
{
    uint input_width;
    uint input_height;
    uint odd_width;
    uint odd_height;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(16u, 16u, 1u);

struct spvDescriptorSetBuffer0
{
    constant DepthPyramidConfig* config [[id(0)]];
    texture2d<float> src_depth_tex [[id(1)]];
    texture2d<float, access::write> dst_depth_tex [[id(3)]];
};

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    constexpr sampler smp(mip_filter::nearest, compare_func::never, max_anisotropy(1), lod_clamp(0.0, 0.0));
    if (gl_GlobalInvocationID.x >= ((*spvDescriptorSet0.config).input_width / 2u))
    {
        return;
    }
    if (gl_GlobalInvocationID.y >= ((*spvDescriptorSet0.config).input_height / 2u))
    {
        return;
    }
    float2 texel_size = float2(1.0 / float((*spvDescriptorSet0.config).input_width), 1.0 / float((*spvDescriptorSet0.config).input_height));
    float2 src_uv = (float2(gl_GlobalInvocationID.xy * uint2(2u)) + float2(0.5)) * texel_size;
    float4 gathered = spvDescriptorSet0.src_depth_tex.gather(smp, src_uv, int2(0), component::x);
    float min_value = fast::min(fast::min(gathered.x, gathered.y), fast::min(gathered.z, gathered.w));
    if ((*spvDescriptorSet0.config).odd_width != 0u)
    {
        float a = spvDescriptorSet0.src_depth_tex.sample(smp, (src_uv + (float2(2.0, 0.0) * texel_size)), level(0.0)).x;
        float b = spvDescriptorSet0.src_depth_tex.sample(smp, (src_uv + (float2(2.0, 1.0) * texel_size)), level(0.0)).x;
        min_value = fast::min(min_value, fast::min(a, b));
    }
    if ((*spvDescriptorSet0.config).odd_height != 0u)
    {
        float a_1 = spvDescriptorSet0.src_depth_tex.sample(smp, (src_uv + (float2(0.0, 2.0) * texel_size)), level(0.0)).x;
        float b_1 = spvDescriptorSet0.src_depth_tex.sample(smp, (src_uv + (float2(1.0, 2.0) * texel_size)), level(0.0)).x;
        min_value = fast::min(min_value, fast::min(a_1, b_1));
    }
    if (((*spvDescriptorSet0.config).odd_width != 0u) && ((*spvDescriptorSet0.config).odd_height != 0u))
    {
        float a_2 = spvDescriptorSet0.src_depth_tex.sample(smp, (src_uv + (float2(2.0) * texel_size)), level(0.0)).x;
        min_value = fast::min(min_value, a_2);
    }
    spvDescriptorSet0.dst_depth_tex.write(float4(min_value), uint2(int2(gl_GlobalInvocationID.xy)));
}

    �      #     �              8        GLSL.std.450                     main    	                    G  	         H         #       H        #      H        #      H        #      G        G     "       G     !       G  G   "       G  G   !      G  K   "       G  K   !      G  �   "       G  �   !      G  �      G  �              !                                            ;     	      +     
                                                ;                       +                        +                +           +              (         )   (      +  (   ,     �?  7         +  (   =      ?,  )   >   =   =     B   (       	 E   (                            F       E   ;  F   G         I      J       I   ;  J   K         M   E   +     _      +  (   j      @+  (   k       ,  )   l   j   k   ,  )   w   j   ,   +     �      ,  )   �   k   j   ,  )   �   ,   j   ,  )   �   j   j    	 �   (                            �       �   ;  �   �         �         +     �      ,     �   �   �      ,  7   �         6               �     �  �       �  
   �   �  �   A        	   
   =           A              =           �              �              �         �           �     �  �   �     A        	      =           A     !          =     "   !   �     #   "      �     $      #   �  &       �  $   %   &   �  %   �  �   �  &   p  (   /      �  (   0   ,   /   p  (   3   "   �  (   4   ,   3   P  )   5   0   4   =     8   	   O  7   9   8   8          �  7   ;   9   �   p  )   <   ;   �  )   ?   <   >   �  )   A   ?   5   =  E   H   G   =  I   L   K   V  M   N   H   L   `  B   P   N   A      Q  (   T   P       Q  (   V   P        (   W      %   T   V   Q  (   Y   P      Q  (   \   P        (   ]      %   Y   \     (   ^      %   W   ]   A     `      _   =     a   `   �     b   a   
   �  d       �  b   c   d   �  c   V  M   h   H   L   �  )   n   l   5     )   o      2   ?   5   n   X  B   p   h   o      k   Q  (   q   p       V  M   u   H   L   �  )   y   w   5     )   z      2   ?   5   y   X  B   {   u   z      k   Q  (   |   {         (   �      %   q   |     (   �      %   ^   �   �  d   �  d   �  (   �   ^   &   �   c   A     �      �   =     �   �   �     �   �   
   �  �       �  �   �   �   �  �   V  M   �   H   L   �  )   �   �   5     )   �      2   ?   5   �   X  B   �   �   �      k   Q  (   �   �       V  M   �   H   L   �  )   �   �   5     )   �      2   ?   5   �   X  B   �   �   �      k   Q  (   �   �         (   �      %   �   �     (   �      %   �   �   �  �   �  �   �  (   �   �   d   �   �   �     �   b   �   �  �       �  �   �   �   �  �   V  M   �   H   L   �  )   �   �   5     )   �      2   ?   5   �   X  B   �   �   �      k   Q  (   �   �         (   �      %   �   �   �  �   �  �   �  (   �   �   �   �   �   =  �   �   �   |  �   �   9   P  B   �   �   �   �   �   c  �   �   �   �  �   �  �   �  8                    �                             DepthPyramidConfig               DepthPyramidConfig               DepthPyramidConfig.input_width           DepthPyramidConfig.input_height          DepthPyramidConfig.odd_width          DepthPyramidConfig.odd_height                               src_depth_tex              src_depth_tex                                     smp              smp                                     dst_depth_tex              dst_depth_tex                         main              �                             DepthPyramidConfig               DepthPyramidConfig               DepthPyramidConfig.input_width           DepthPyramidConfig.input_height          DepthPyramidConfig.odd_width          DepthPyramidConfig.odd_height                                   src_depth_tex              src_depth_tex                                       smp              smp                                                                              dst_depth_tex              dst_depth_tex                                     �                             DepthPyramidConfig               DepthPyramidConfig               DepthPyramidConfig.input_width           DepthPyramidConfig.input_height          DepthPyramidConfig.odd_width          DepthPyramidConfig.odd_height                               src_depth_tex              src_depth_tex                                     smp              smp                                     dst_depth_tex              dst_depth_tex                         main              �                             DepthPyramidConfig               DepthPyramidConfig               DepthPyramidConfig.input_width           DepthPyramidConfig.input_height          DepthPyramidConfig.odd_width          DepthPyramidConfig.odd_height                                   src_depth_tex              src_depth_tex                                       smp              smp                                                                              dst_depth_tex              dst_depth_tex                                     �                             DepthPyramidConfig               DepthPyramidConfig               DepthPyramidConfig.input_width           DepthPyramidConfig.input_height          DepthPyramidConfig.odd_width          DepthPyramidConfig.odd_height                               src_depth_tex              src_depth_tex                                     smp              smp                                     dst_depth_tex              dst_depth_tex                         main              �                             DepthPyramidConfig               DepthPyramidConfig               DepthPyramidConfig.input_width           DepthPyramidConfig.input_height          DepthPyramidConfig.odd_width          DepthPyramidConfig.odd_height                                   src_depth_tex              src_depth_tex                                       smp              smp                                                                              dst_depth_tex              dst_depth_tex                            depth_pyramid.comp