                       �""D@"���Ȟ��#�s�      �      struct LightBitfieldsData
{
    uint light_count[3072];
    uint bitfields[49152];
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

static const uint3 gl_WorkGroupSize = uint3(1024u, 1u, 1u);

RWByteAddressBuffer input_data : register(u0, space0);
RWByteAddressBuffer output_data : register(u1, space0);

static uint3 gl_GlobalInvocationID;
struct SPIRV_Cross_Input
{
    uint3 gl_GlobalInvocationID : SV_DispatchThreadID;
};

void comp_main()
{
    uint cluster_index = gl_GlobalInvocationID.x;
    uint cluster_first_u32_index = cluster_index * 16u;
    uint count = input_data.Load(cluster_index * 4 + 0);
    output_data.Store(cluster_index * 8 + 16, count);
    if (count == 0u)
    {
        output_data.Store(cluster_index * 8 + 20, 0u);
        return;
    }
    uint _62;
    output_data.InterlockedAdd(0, count, _62);
    uint list_start_index = _62;
    output_data.Store(cluster_index * 8 + 20, list_start_index);
    int written_light_count = 0;
    uint u32_index = 0u;
    for (;;)
    {
        if (u32_index < 16u)
        {
            uint u32_value = input_data.Load((cluster_first_u32_index + u32_index) * 4 + 12288);
            int lsb = int(firstbitlow(u32_value));
            for (;;)
            {
                if (lsb > (-1))
                {
                    u32_value &= uint(~(1 << lsb));
                    uint light_index = (32u * u32_index) + uint(lsb);
                    output_data.Store((list_start_index + uint(written_light_count)) * 4 + 24592, light_index);
                    written_light_count++;
                    lsb = int(firstbitlow(u32_value));
                    continue;
                }
                else
                {
                    break;
                }
            }
            u32_index++;
            continue;
        }
        else
        {
            break;
        }
    }
}

[numthreads(1024, 1, 1)]
void main(SPIRV_Cross_Input stage_input)
{
    gl_GlobalInvocationID = stage_input.gl_GlobalInvocationID;
    comp_main();
}
    �
      #pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wunused-variable"

#include <metal_stdlib>
#include <simd/simd.h>
#include <metal_atomic>

using namespace metal;

struct LightBitfieldsData
{
    uint light_count[3072];
    uint bitfields[49152];
};

struct LightBitfields
{
    LightBitfieldsData data;
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

struct LightBuildListsOutput
{
    LightBinningOutput data;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(1024u, 1u, 1u);

struct spvDescriptorSetBuffer0
{
    const device LightBitfields* input_data [[id(0)]];
    device LightBuildListsOutput* output_data [[id(1)]];
};

// Implementation of the GLSL findLSB() function
template<typename T>
inline T spvFindLSB(T x)
{
    return select(ctz(x), T(-1), x == T(0));
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint cluster_index = gl_GlobalInvocationID.x;
    uint cluster_first_u32_index = cluster_index * 16u;
    uint count = (*spvDescriptorSet0.input_data).data.light_count[cluster_index];
    (*spvDescriptorSet0.output_data).data.offsets[cluster_index].count = count;
    if (count == 0u)
    {
        (*spvDescriptorSet0.output_data).data.offsets[cluster_index].first_light = 0u;
        return;
    }
    uint _62 = atomic_fetch_add_explicit((device atomic_uint*)&(*spvDescriptorSet0.output_data).data.data_write_ptr, count, memory_order_relaxed);
    uint list_start_index = _62;
    (*spvDescriptorSet0.output_data).data.offsets[cluster_index].first_light = list_start_index;
    int written_light_count = 0;
    uint u32_index = 0u;
    for (;;)
    {
        if (u32_index < 16u)
        {
            uint u32_value = (*spvDescriptorSet0.input_data).data.bitfields[cluster_first_u32_index + u32_index];
            int lsb = int(spvFindLSB(u32_value));
            for (;;)
            {
                if (lsb > (-1))
                {
                    u32_value &= uint(~(1 << lsb));
                    uint light_index = (32u * u32_index) + uint(lsb);
                    (*spvDescriptorSet0.output_data).data.data[list_start_index + uint(written_light_count)] = light_index;
                    written_light_count++;
                    lsb = int(spvFindLSB(u32_value));
                    continue;
                }
                else
                {
                    break;
                }
            }
            u32_index++;
            continue;
        }
        else
        {
            break;
        }
    }
}

    �      #     �                 GLSL.std.450                     main                        G           G           G           H         #       H        #    0  H            H         #       G        G     "       G     !       H  #       #       H  #      #      G  $         G  &         H  '       #       H  '      #      H  '      #      H  '      #      H  '      #      H  '      #   `  H  (          H  (       #       G  (      G  *   "       G  *   !      G  v              !                       	            
      	   ;  
         +                        +           +                      +         �                                            ;                       +                           #           $   #      +     %        &      %     '               $   &     (   '      )      (   ;  )   *      +     +        0   +     5      +     ;      +     Y   ����+     b       +     h      +     u      ,  	   v   u   ;   ;   6               �     �  w       �     x   �  x   A              =           �              A      !               =     "   !   A      .   *      +         >  .   "   �  0   1   "      �  3       �  1   2   3   �  2   A      6   *      +      5   >  6      �  w   �  3   A      9   *         �     <   9   ;      "   A      ?   *      +      5   >  ?   <   �  C   �  C   �     }      3   t   F   �     �      3   �   F   �  0   I   }      �  E   F       �  I   D   E   �  D   �     M      }   A      N         5   M   =     O   N        R      I   O   �  S   �  S   �     �   �   D   p   T   �     �   O   D   `   T   �     ~   R   D   r   T   �  0   Z   ~   Y   �  U   T       �  Z   T   U   �  T   �     \   5   ~   �     ]   \   |     ^   ]   �     `   �   ^   �     d   b   }   |     f   ~   �     g   d   f   |     k   �   �     l   <   k   A      n   *      h   l   >  n   g   �     p   �   5        r      I   `   �  S   �  U   �  F   �  F   �     t   }   5   �  C   �  E   �  w   �  w   �  8                                                  LightBitfields        
       input_data                                      LightBuildListsOutput              output_data                         main                                            LightBitfields        
       input_data                                        LightBuildListsOutput              output_data                                                                   LightBitfields        
       input_data                                      LightBuildListsOutput              output_data                         main                                            LightBitfields        
       input_data                                        LightBuildListsOutput              output_data                                                                   LightBitfields        
       input_data                                      LightBuildListsOutput              output_data                         main                                            LightBitfields        
       input_data                                        LightBuildListsOutput              output_data                            lights_build_lists.comp