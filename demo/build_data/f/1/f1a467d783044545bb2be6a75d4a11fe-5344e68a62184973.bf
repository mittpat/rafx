                       �""D@"���Ȟn�Ӯ�T�      b      struct PerCullInfo
{
    uint was_culled;
};

struct IndirectCommand
{
    uint push_constant;
    uint index_count;
    uint instance_count;
    uint first_index;
    int vertex_offset;
    uint first_instance;
};

struct DrawData
{
    uint transform_index;
    uint material_index;
};

struct BoundingSphere
{
    float3 position;
    float radius;
};

static const uint3 gl_WorkGroupSize = uint3(1u, 64u, 1u);

cbuffer Config : register(b0, space0)
{
    row_major float4x4 config_view_matrix : packoffset(c0);
    row_major float4x4 config_proj_matrix : packoffset(c4);
    uint config_draw_data_count : packoffset(c8);
    uint config_indirect_first_command_index : packoffset(c8.y);
    uint config_depth_mip_slice_count : packoffset(c8.z);
    uint config_viewport_width : packoffset(c8.w);
    uint config_viewport_height : packoffset(c9);
    float config_z_near : packoffset(c9.y);
    uint config_write_debug_output : packoffset(c9.z);
};

RWByteAddressBuffer debug_output : register(u21, space0);
RWByteAddressBuffer all_indirect_commands : register(u3, space0);
RWByteAddressBuffer all_draw_data : register(u1, space0);
RWByteAddressBuffer all_bounding_spheres : register(u2, space0);
Texture2D<float4> depth_mip_slices[16] : register(t4, space0);
SamplerState smp : register(s20, space0);

static uint3 gl_GlobalInvocationID;
struct SPIRV_Cross_Input
{
    uint3 gl_GlobalInvocationID : SV_DispatchThreadID;
};

void comp_main()
{
    uint draw_index = gl_GlobalInvocationID.y;
    if (draw_index >= config_draw_data_count)
    {
        return;
    }
    if (config_write_debug_output != 0u)
    {
        uint indirect_index = config_indirect_first_command_index + draw_index;
        uint _56;
        debug_output.InterlockedAdd(0, 1u, _56);
        uint _69;
        debug_output.InterlockedAdd(8, all_indirect_commands.Load(indirect_index * 24 + 4) / 3u, _69);
    }
    uint transform_index = all_draw_data.Load(draw_index * 8 + 16);
    BoundingSphere _92;
    _92.position = asfloat(all_bounding_spheres.Load3(transform_index * 16 + 0));
    _92.radius = asfloat(all_bounding_spheres.Load(transform_index * 16 + 12));
    BoundingSphere bs;
    bs.position = _92.position;
    bs.radius = _92.radius;
    float3 center_vs = mul(float4(bs.position, 1.0f), config_view_matrix).xyz;
    float radius = bs.radius;
    if (radius < 0.0f)
    {
        return;
    }
    float _135;
    float _146;
    float _157;
    float2 min_uv;
    float2 max_uv;
    float max_ndc_z;
    int i = 0;
    for (;;)
    {
        if (i < 8)
        {
            if ((i & 1) != 0)
            {
                _135 = radius;
            }
            else
            {
                _135 = -radius;
            }
            if ((i & 2) != 0)
            {
                _146 = radius;
            }
            else
            {
                _146 = -radius;
            }
            if ((i & 4) != 0)
            {
                _157 = radius;
            }
            else
            {
                _157 = -radius;
            }
            float3 corner_vs = center_vs + float3(_135, _146, _157);
            float4 corner_clip = mul(float4(corner_vs, 1.0f), config_proj_matrix);
            float3 corner_ndc = corner_clip.xyz / corner_clip.w.xxx;
            if (corner_ndc.z < 0.0f)
            {
                return;
            }
            float2 corner_uv = (corner_ndc.xy * float2(0.5f, -0.5f)) + 0.5f.xx;
            if (i == 0)
            {
                min_uv = corner_uv;
                max_uv = corner_uv;
                max_ndc_z = corner_ndc.z;
            }
            else
            {
                min_uv = min(min_uv, corner_uv);
                max_uv = max(max_uv, corner_uv);
                max_ndc_z = max(max_ndc_z, corner_ndc.z);
            }
            i++;
            continue;
        }
        else
        {
            break;
        }
    }
    min_uv = clamp(min_uv, 0.0f.xx, 1.0f.xx);
    max_uv = clamp(max_uv, 0.0f.xx, 1.0f.xx);
    uint mip_slice = config_depth_mip_slice_count;
    float2 mip_slice_size;
    int2 min_sample_pixel;
    int2 max_sample_pixel;
    int i_1 = 0;
    for (;;)
    {
        if (uint(i_1) < config_depth_mip_slice_count)
        {
            mip_slice_size = float2(max(uint2(1u, 1u), uint2(config_viewport_width >> uint(i_1), config_viewport_height >> uint(i_1))));
            min_sample_pixel = int2(min_uv * mip_slice_size);
            max_sample_pixel = int2(max_uv * mip_slice_size);
            int2 pixel_dim = max_sample_pixel - min_sample_pixel;
            bool _285 = (max_sample_pixel.x - min_sample_pixel.x) < 2;
            bool _294;
            if (_285)
            {
                _294 = (max_sample_pixel.y - min_sample_pixel.y) < 2;
            }
            else
            {
                _294 = _285;
            }
            if (_294)
            {
                mip_slice = uint(i_1);
                break;
            }
            i_1++;
            continue;
        }
        else
        {
            break;
        }
    }
    if (mip_slice == config_depth_mip_slice_count)
    {
        return;
    }
    float2 gather_pixel = (float2(min_sample_pixel) + float2(max_sample_pixel)) / 2.0f.xx;
    float2 gather_uv = (gather_pixel + 0.5f.xx) / mip_slice_size;
    float4 gathered = depth_mip_slices[mip_slice].GatherRed(smp, gather_uv);
    bool use_both_samples_for_x = min_sample_pixel.x != max_sample_pixel.x;
    bool use_both_samples_for_y = min_sample_pixel.y != max_sample_pixel.y;
    float hiz_depth = gathered.w;
    if (use_both_samples_for_x)
    {
        hiz_depth = min(hiz_depth, gathered.z);
    }
    if (use_both_samples_for_y)
    {
        hiz_depth = min(hiz_depth, gathered.x);
    }
    if (use_both_samples_for_x && use_both_samples_for_y)
    {
        hiz_depth = min(hiz_depth, gathered.y);
    }
    if (max_ndc_z < hiz_depth)
    {
        uint indirect_index_1 = config_indirect_first_command_index + draw_index;
        all_indirect_commands.Store(indirect_index_1 * 24 + 8, 0u);
        if (config_write_debug_output != 0u)
        {
            uint _399;
            debug_output.InterlockedAdd(4, 1u, _399);
            uint _405;
            debug_output.InterlockedAdd(12, all_indirect_commands.Load(indirect_index_1 * 24 + 4) / 3u, _405);
        }
    }
}

[numthreads(1, 64, 1)]
void main(SPIRV_Cross_Input stage_input)
{
    gl_GlobalInvocationID = stage_input.gl_GlobalInvocationID;
    comp_main();
}
    �      #pragma clang diagnostic ignored "-Wunused-variable"

#include <metal_stdlib>
#include <simd/simd.h>
#include <metal_atomic>

using namespace metal;

struct Config
{
    float4x4 view_matrix;
    float4x4 proj_matrix;
    uint draw_data_count;
    uint indirect_first_command_index;
    uint depth_mip_slice_count;
    uint viewport_width;
    uint viewport_height;
    float z_near;
    uint write_debug_output;
};

struct PerCullInfo
{
    uint was_culled;
};

struct DebugOutput
{
    uint total_mesh_count;
    uint culled_mesh_count;
    uint total_primitive_count;
    uint culled_primitive_count;
    PerCullInfo per_cull_info[4000];
};

struct IndirectCommand
{
    uint index_count;
    uint instance_count;
    uint first_index;
    int vertex_offset;
    uint first_instance;
};

struct IndirectData
{
    IndirectCommand indirect_commands[1];
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

struct BoundingSphere
{
    float3 position;
    float radius;
};

struct BoundingSphere_1
{
    packed_float3 position;
    float radius;
};

struct AllBoundingSpheres
{
    BoundingSphere_1 bounding_spheres[1];
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(1u, 64u, 1u);

struct spvDescriptorSetBuffer0
{
    constant Config* config [[id(0)]];
    device AllDrawData* all_draw_data [[id(1)]];
    device AllBoundingSpheres* all_bounding_spheres [[id(2)]];
    device IndirectData* all_indirect_commands [[id(3)]];
    array<texture2d<float>, 16> depth_mip_slices [[id(4)]];
    device DebugOutput* debug_output [[id(21)]];
};

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    constexpr sampler smp(mip_filter::linear, compare_func::never, max_anisotropy(1));
    uint draw_index = gl_GlobalInvocationID.y;
    if (draw_index >= (*spvDescriptorSet0.config).draw_data_count)
    {
        return;
    }
    if ((*spvDescriptorSet0.config).write_debug_output != 0u)
    {
        uint indirect_index = (*spvDescriptorSet0.config).indirect_first_command_index + draw_index;
        uint _56 = atomic_fetch_add_explicit((device atomic_uint*)&(*spvDescriptorSet0.debug_output).total_mesh_count, 1u, memory_order_relaxed);
        uint _68 = atomic_fetch_add_explicit((device atomic_uint*)&(*spvDescriptorSet0.debug_output).total_primitive_count, (*spvDescriptorSet0.all_indirect_commands).indirect_commands[indirect_index].index_count / 3u, memory_order_relaxed);
    }
    uint transform_index = (*spvDescriptorSet0.all_draw_data).draw_data[draw_index].transform_index;
    BoundingSphere bs;
    bs.position = float3((*spvDescriptorSet0.all_bounding_spheres).bounding_spheres[transform_index].position);
    bs.radius = (*spvDescriptorSet0.all_bounding_spheres).bounding_spheres[transform_index].radius;
    float3 center_vs = ((*spvDescriptorSet0.config).view_matrix * float4(bs.position, 1.0)).xyz;
    float radius = bs.radius;
    if (radius < 0.0)
    {
        return;
    }
    float _135;
    float _146;
    float _157;
    float2 min_uv;
    float2 max_uv;
    float max_ndc_z;
    int i = 0;
    for (;;)
    {
        if (i < 8)
        {
            if ((i & 1) != 0)
            {
                _135 = radius;
            }
            else
            {
                _135 = -radius;
            }
            if ((i & 2) != 0)
            {
                _146 = radius;
            }
            else
            {
                _146 = -radius;
            }
            if ((i & 4) != 0)
            {
                _157 = radius;
            }
            else
            {
                _157 = -radius;
            }
            float3 corner_vs = center_vs + float3(_135, _146, _157);
            float4 corner_clip = (*spvDescriptorSet0.config).proj_matrix * float4(corner_vs, 1.0);
            float3 corner_ndc = corner_clip.xyz / float3(corner_clip.w);
            if (corner_ndc.z < 0.0)
            {
                return;
            }
            float2 corner_uv = (corner_ndc.xy * float2(0.5, -0.5)) + float2(0.5);
            if (i == 0)
            {
                min_uv = corner_uv;
                max_uv = corner_uv;
                max_ndc_z = corner_ndc.z;
            }
            else
            {
                min_uv = fast::min(min_uv, corner_uv);
                max_uv = fast::max(max_uv, corner_uv);
                max_ndc_z = fast::max(max_ndc_z, corner_ndc.z);
            }
            i++;
            continue;
        }
        else
        {
            break;
        }
    }
    min_uv = fast::clamp(min_uv, float2(0.0), float2(1.0));
    max_uv = fast::clamp(max_uv, float2(0.0), float2(1.0));
    uint mip_slice = (*spvDescriptorSet0.config).depth_mip_slice_count;
    float2 mip_slice_size;
    int2 min_sample_pixel;
    int2 max_sample_pixel;
    int i_1 = 0;
    for (;;)
    {
        if (uint(i_1) < (*spvDescriptorSet0.config).depth_mip_slice_count)
        {
            mip_slice_size = float2(max(uint2(1u), uint2((*spvDescriptorSet0.config).viewport_width >> uint(i_1), (*spvDescriptorSet0.config).viewport_height >> uint(i_1))));
            min_sample_pixel = int2(min_uv * mip_slice_size);
            max_sample_pixel = int2(max_uv * mip_slice_size);
            int2 pixel_dim = max_sample_pixel - min_sample_pixel;
            bool _285 = (max_sample_pixel.x - min_sample_pixel.x) < 2;
            bool _294;
            if (_285)
            {
                _294 = (max_sample_pixel.y - min_sample_pixel.y) < 2;
            }
            else
            {
                _294 = _285;
            }
            if (_294)
            {
                mip_slice = uint(i_1);
                break;
            }
            i_1++;
            continue;
        }
        else
        {
            break;
        }
    }
    if (mip_slice == (*spvDescriptorSet0.config).depth_mip_slice_count)
    {
        return;
    }
    float2 gather_pixel = (float2(min_sample_pixel) + float2(max_sample_pixel)) / float2(2.0);
    float2 gather_uv = (gather_pixel + float2(0.5)) / mip_slice_size;
    float4 gathered = spvDescriptorSet0.depth_mip_slices[mip_slice].gather(smp, gather_uv, int2(0), component::x);
    bool use_both_samples_for_x = min_sample_pixel.x != max_sample_pixel.x;
    bool use_both_samples_for_y = min_sample_pixel.y != max_sample_pixel.y;
    float hiz_depth = gathered.w;
    if (use_both_samples_for_x)
    {
        hiz_depth = fast::min(hiz_depth, gathered.z);
    }
    if (use_both_samples_for_y)
    {
        hiz_depth = fast::min(hiz_depth, gathered.x);
    }
    if (use_both_samples_for_x && use_both_samples_for_y)
    {
        hiz_depth = fast::min(hiz_depth, gathered.y);
    }
    if (max_ndc_z < hiz_depth)
    {
        uint indirect_index_1 = (*spvDescriptorSet0.config).indirect_first_command_index + draw_index;
        (*spvDescriptorSet0.all_indirect_commands).indirect_commands[indirect_index_1].instance_count = 0u;
        if ((*spvDescriptorSet0.config).write_debug_output != 0u)
        {
            uint _399 = atomic_fetch_add_explicit((device atomic_uint*)&(*spvDescriptorSet0.debug_output).culled_mesh_count, 1u, memory_order_relaxed);
            uint _405 = atomic_fetch_add_explicit((device atomic_uint*)&(*spvDescriptorSet0.debug_output).culled_primitive_count, (*spvDescriptorSet0.all_indirect_commands).indirect_commands[indirect_index_1].index_count / 3u, memory_order_relaxed);
        }
    }
}

    �      #                     GLSL.std.450                     main                  @      G           H            H         #       H               H           H        #   @   H              H        #   �   H        #   �   H        #   �   H        #   �   H        #   �   H        #   �   H        #   �   G        G     "       G     !       H  .       #       G  0         H  1       #       H  1      #      H  1      #      H  1      #      H  1      #      G  1      G  3   "       G  3   !      H  8       #       H  8      #      H  8      #      H  8      #      H  8      #      G  9         H  :       #       G  :      G  <   "       G  <   !      H  D       #       H  D      #      G  E         H  F       #       H  F      #      H  F      #      H  F      #      H  F      #      G  F      G  H   "       G  H   !      H  Q       #       H  Q      #      G  R         H  S       #       G  S      G  U   "       G  U   !      G  G  "       G  G  !      G  N  "       G  N  !      G  �             !                       	            
      	   ;  
         +                                                                                                  ;                       +                            +     !      +     $       +     )        .      +     /   �    0   .   /     1               0      2      1   ;  2   3      +     4         8                    9   8     :   9      ;      :   ;  ;   <      +     @        D           E   D     F               E      G      F   ;  G   H      +     I        M           Q   M        R   Q     S   R      T      S   ;  T   U         W      Q   +     ^         b         +     g     �?+     r         �         +     �      ?+     �      �,  �   �   �   �     �         ,  �   �         +     �      +     �                ,  �   >  �   �    	 C                           +     D       E  C  D     F      E  ;  F  G         I      C    L     M      L  ;  M  N        P  C  +     �  @   ,  	   �     �     *     �  )     �    �   �       �      �  ,  �     r   r   ,  �     g   g   6               �     �  �      �  $   �  �  �  A              =           A              =           �              �         �           �     �  �  �     A     "      !   =     #   "   �     %   #   $   �  '       �  %   &   '   �  &   A     *      )   =     +   *   �     -   +      A     5   3   4   �     6   5      $      A     7   3      A     >   <   4   -   4   =     ?   >   �     A   ?   @   �     B   7      $   A   �  '   �  '   A     K   H   I      4   =     L   K   A  W   X   U   4   L   =  Q   Y   X   Q  M   Z   Y       Q     ]   Y      A  b   c      4   =     d   c   Q     h   Z       Q     i   Z      Q     j   Z      P     k   h   i   j   g   �     l   d   k   O  M   m   l   l             �     s   ]   r   �  u       �  s   t   u   �  t   �  �  �  u   �  y   �  y   �     �  �  u     |   �  �   �  �  u     |   �  �   �  �  u     |   �     �  4   u   �   |   �        �  !   �  {   |       �     z   {   �  z   �     �   �  ^   �     �   �   4   �  �       �  �   �   �   �  �   �  �   �  �        �   ]   �  �   �  �   �     �  ]   �   �   �   �     �   �     �     �   �   4   �  �       �  �   �   �   �  �   �  �   �  �        �   ]   �  �   �  �   �     �  ]   �   �   �   �     �   �  I   �     �   �   4   �  �       �  �   �   �   �  �   �  �   �  �        �   ]   �  �   �  �   �     �  ]   �   �   �   P  M   �   �  �  �  �  M   �   m   �   A  b   �      ^   =     �   �   Q     �   �       Q     �   �      Q     �   �      P     �   �   �   �   g   �     �   �   �   O  M   �   �   �             Q     �   �      P  M   �   �   �   �   �  M   �   �   �   Q     �   �      �     �   �   r   �  �       �  �   �   �   �  �   �  {   �  �   O  �   �   �   �            �   �      2   �   �   >  �     �   �  4   �  �       �  �   �   �   �  �   �  �   �  �     �   �      %   �  �     �   �      (   �  �        �      (   �  �   �  �   �  �   �       �   �   �   �   �  �     �   �   �   �   �  �     �   �   �   �   �  |   �  |   �     �   �  ^   �  y   �  {   �     �  �  y   �  �   �  �      �  �  �  �  �  �    �   �      +   �        �   �      +   �      A     �      I   =     �   �   �  �   �  �   �     �  4   �  +  �   �  �   �  �  �    �   �    �  �  �    �   �    �  �  �    �   |     �   �  �     �   �   �   �  �   �       �  �   �   �   �  �   A     �      �   =     �   �   �     �   �   �  A     �      �   =        �   �          �  P  �     �       �        )   �     p  �       �  �     �     n        �  �     �     n        Q             Q             �           �            �        �        �    Q            Q     !       �     "    !  �     #  "     �    �    �     $    �   #    �  &      �  $  %  &  �  %  �  �   �  &  �  �   �  �   �     +  �  ^   �  �   �  �   �  �   �  �  �     %  �    �  �  �     %  �    �  �  �     %  �     �  �   �   �   %  �     /  �  �   �  1      �  /  0  1  �  0  �  �  �  1  o  �   5  �  o  �   7  �  �  �   8  5  7    �   ?     2   8  >  >  �  �   A  ?  �  A  I  J  G  �  =  C  K  J  =  L  O  N  V  P  Q  K  O  `     S  Q  A  4   Q     W  �      Q     Y  �      �     Z  W  Y  Q     ]  �     Q     _  �     �     `  ]  _  Q     c  S     �  f      �  Z  e  f  �  e  Q     i  S          j     %   c  i  �  f  �  f  �     �  c  1  j  e  �  m      �  `  l  m  �  l  Q     p  S           q     %   �  p  �  m  �  m  �     �  �  f  q  l  �     t  Z  `  �  v      �  t  u  v  �  u  Q     y  S          z     %   �  y  �  v  �  v  �        �  m  z  u  �     }  �     �        �  }  ~    �  ~  A     �     )   =     �  �  �     �  �     A     �  <   4   �  ^   >  �  $   �  �      �  %   �  �  �  �  A     �  3   ^   �     �  �     $      A     �  3   )   A     �  <   4   �  4   =     �  �  �     �  �  @   �     �  �     $   �  �  �  �  �  �    �    �  �  �  �  �  8                    �                             Config               Config 	              Config.view_matrix           Config.proj_matrix@          Config.draw_data_count�   #       Config.indirect_first_command_index�          Config.depth_mip_slice_count�          Config.viewport_width�          Config.viewport_height�          Config.z_near�          Config.write_debug_output�                                AllDrawData              all_draw_data                                      AllBoundingSpheres              all_bounding_spheres                                      IndirectData              all_indirect_commands                                     depth_mip_slices              depth_mip_slices                                     smp              smp                                      DebugOutput              debug_output            @             main              �                             Config               Config 	              Config.view_matrix           Config.proj_matrix@          Config.draw_data_count�   #       Config.indirect_first_command_index�          Config.depth_mip_slice_count�          Config.viewport_width�          Config.viewport_height�          Config.z_near�          Config.write_debug_output�    �                                AllDrawData              all_draw_data                                        AllBoundingSpheres              all_bounding_spheres                                        IndirectData              all_indirect_commands                                       depth_mip_slices              depth_mip_slices                                       smp              smp                                                                              DebugOutput              debug_output                                     �                             Config               Config 	              Config.view_matrix           Config.proj_matrix@          Config.draw_data_count�   #       Config.indirect_first_command_index�          Config.depth_mip_slice_count�          Config.viewport_width�          Config.viewport_height�          Config.z_near�          Config.write_debug_output�                                AllDrawData              all_draw_data                                      AllBoundingSpheres              all_bounding_spheres                                      IndirectData              all_indirect_commands                                     depth_mip_slices              depth_mip_slices                                     smp              smp                                      DebugOutput              debug_output            @             main              �                             Config               Config 	              Config.view_matrix           Config.proj_matrix@          Config.draw_data_count�   #       Config.indirect_first_command_index�          Config.depth_mip_slice_count�          Config.viewport_width�          Config.viewport_height�          Config.z_near�          Config.write_debug_output�    �                                AllDrawData              all_draw_data                                        AllBoundingSpheres              all_bounding_spheres                                        IndirectData              all_indirect_commands                                       depth_mip_slices              depth_mip_slices                                       smp              smp                                                                              DebugOutput              debug_output                                     �                             Config               Config 	              Config.view_matrix           Config.proj_matrix@          Config.draw_data_count�   #       Config.indirect_first_command_index�          Config.depth_mip_slice_count�          Config.viewport_width�          Config.viewport_height�          Config.z_near�          Config.write_debug_output�                                AllDrawData              all_draw_data                                      AllBoundingSpheres              all_bounding_spheres                                      IndirectData              all_indirect_commands                                     depth_mip_slices              depth_mip_slices                                     smp              smp                                      DebugOutput              debug_output            @             main              �                             Config               Config 	              Config.view_matrix           Config.proj_matrix@          Config.draw_data_count�   #       Config.indirect_first_command_index�          Config.depth_mip_slice_count�          Config.viewport_width�          Config.viewport_height�          Config.z_near�          Config.write_debug_output�    �                                AllDrawData              all_draw_data                                        AllBoundingSpheres              all_bounding_spheres                                        IndirectData              all_indirect_commands                                       depth_mip_slices              depth_mip_slices                                       smp              smp                                                                              DebugOutput              debug_output                            mesh_culling.comp