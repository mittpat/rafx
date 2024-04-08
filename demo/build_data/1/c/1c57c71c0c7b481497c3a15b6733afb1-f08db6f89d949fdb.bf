                       �""D@"���Ȟ�Q��'      D
      struct ClusterAABB
{
    float3 _min;
    float3 _max;
};

struct Light
{
    float3 position;
    float radius;
};

struct LightBitfieldsData
{
    uint light_count[3072];
    uint bitfields[49152];
};

static const uint3 gl_WorkGroupSize = uint3(64u, 16u, 1u);

RWByteAddressBuffer config : register(u0, space0);
RWByteAddressBuffer lights : register(u1, space0);
RWByteAddressBuffer bitfields : register(u2, space0);

static uint3 gl_GlobalInvocationID;
struct SPIRV_Cross_Input
{
    uint3 gl_GlobalInvocationID : SV_DispatchThreadID;
};

void comp_main()
{
    uint cluster_index = gl_GlobalInvocationID.x;
    ClusterAABB _34;
    _34._min = asfloat(config.Load3(cluster_index * 32 + 0));
    _34._max = asfloat(config.Load3(cluster_index * 32 + 16));
    ClusterAABB cluster;
    cluster._min = _34._min;
    cluster._max = _34._max;
    uint light_u32_index = gl_GlobalInvocationID.y;
    uint count = 0u;
    uint bitfield = 0u;
    Light light;
    int light_bit_index = 0;
    for (;;)
    {
        if (light_bit_index < 32)
        {
            uint light_index = (light_u32_index * 32u) + uint(light_bit_index);
            if (light_index >= lights.Load(0))
            {
                break;
            }
            Light _85;
            _85.position = asfloat(lights.Load3(light_index * 16 + 16));
            _85.radius = asfloat(lights.Load(light_index * 16 + 28));
            light.position = _85.position;
            light.radius = _85.radius;
            float3 light_min = light.position - light.radius.xxx;
            float3 light_max = light.position + light.radius.xxx;
            bool _110 = all(bool3(light_min.x <= cluster._max.x, light_min.y <= cluster._max.y, light_min.z <= cluster._max.z));
            bool _118;
            if (_110)
            {
                _118 = all(bool3(light_max.x >= cluster._min.x, light_max.y >= cluster._min.y, light_max.z >= cluster._min.z));
            }
            else
            {
                _118 = _110;
            }
            if (_118 == true)
            {
                count++;
                bitfield |= uint(1 << light_bit_index);
            }
            light_bit_index++;
            continue;
        }
        else
        {
            break;
        }
    }
    if (bitfield != 0u)
    {
        bitfields.Store(((cluster_index * 16u) + light_u32_index) * 4 + 12288, bitfield);
    }
    uint _153;
    bitfields.InterlockedAdd(cluster_index * 4 + 0, count, _153);
}

[numthreads(64, 16, 1)]
void main(SPIRV_Cross_Input stage_input)
{
    gl_GlobalInvocationID = stage_input.gl_GlobalInvocationID;
    comp_main();
}
    D      #pragma clang diagnostic ignored "-Wunused-variable"

#include <metal_stdlib>
#include <simd/simd.h>
#include <metal_atomic>

using namespace metal;

struct ClusterAABB
{
    float3 _min;
    float3 _max;
};

struct ClusterAABB_1
{
    float3 _min;
    float3 _max;
};

struct BinLightsConfig
{
    ClusterAABB_1 clusters[3072];
};

struct Light
{
    packed_float3 position;
    float radius;
};

struct LightsInputList
{
    uint light_count;
    char _m1_pad[12];
    Light lights[512];
};

struct Light_1
{
    float3 position;
    float radius;
};

struct LightBitfieldsData
{
    uint light_count[3072];
    uint bitfields[49152];
};

struct LightBitfields
{
    LightBitfieldsData data;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(64u, 16u, 1u);

struct spvDescriptorSetBuffer0
{
    const device BinLightsConfig* config [[id(0)]];
    const device LightsInputList* lights [[id(1)]];
    device LightBitfields* bitfields [[id(2)]];
};

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint cluster_index = gl_GlobalInvocationID.x;
    ClusterAABB cluster;
    cluster._min = (*spvDescriptorSet0.config).clusters[cluster_index]._min;
    cluster._max = (*spvDescriptorSet0.config).clusters[cluster_index]._max;
    uint light_u32_index = gl_GlobalInvocationID.y;
    uint count = 0u;
    uint bitfield = 0u;
    Light_1 light;
    int light_bit_index = 0;
    for (;;)
    {
        if (light_bit_index < 32)
        {
            uint light_index = (light_u32_index * 32u) + uint(light_bit_index);
            if (light_index >= (*spvDescriptorSet0.lights).light_count)
            {
                break;
            }
            light.position = float3((*spvDescriptorSet0.lights).lights[light_index].position);
            light.radius = (*spvDescriptorSet0.lights).lights[light_index].radius;
            float3 light_min = light.position - float3(light.radius);
            float3 light_max = light.position + float3(light.radius);
            bool _110 = all(light_min <= cluster._max);
            bool _118;
            if (_110)
            {
                _118 = all(light_max >= cluster._min);
            }
            else
            {
                _118 = _110;
            }
            if (_118 == true)
            {
                count++;
                bitfield |= uint(1 << light_bit_index);
            }
            light_bit_index++;
            continue;
        }
        else
        {
            break;
        }
    }
    if (bitfield != 0u)
    {
        (*spvDescriptorSet0.bitfields).data.bitfields[(cluster_index * 16u) + light_u32_index] = bitfield;
    }
    uint _153 = atomic_fetch_add_explicit((device atomic_uint*)&(*spvDescriptorSet0.bitfields).data.light_count[cluster_index], count, memory_order_relaxed);
}

    �	      #     �                 GLSL.std.450                     main               @         G           H         #       H        #      G            H            H         #       G        G     "       G     !       H  @       #       H  @      #      G  B         H  C          H  C       #       H  C         H  C      #      G  C      G  E   "       G  E   !      G  �         G  �         H  �       #       H  �      #    0  H  �          H  �       #       G  �      G  �   "       G  �   !      G  �              !                       	            
      	   ;  
         +                                                       +                                          ;                       +                        +     %      +     (      +     5         6   +     :         @         +     A        B   @   A     C      B      D      C   ;  D   E         F            Q      @     j   6      )  6   u     �         +     �    �    �      �     �   �   �     �   �      �      �   ;  �   �      +     �      +     �   @   ,  	   �   �   �   (   6               �     A              =           A                 =            Q     !           Q     $          A     )      (   =     *   )   �  /   �  /   �     �         �   2   �     �         �   2   �     �         �   2   �  6   7   �   5   �  1   2       �  7   0   1   �  0   �     ;   *   :   |     =   �   �     >   ;   =   A  F   G   E      =     H   G   �  6   I   >   H   �  K       �  I   J   K   �  J   �  1   �  K   A  Q   R   E   %   >   =  @   S   R   Q     T   S       Q     V   S      P     ^   V   V   V   �     _   T   ^   �     f   T   ^   �  j   k   _   $   �  6   l   k   �  n       �  l   m   n   �  m   �  j   r   f   !   �  6   s   r   �  n   �  n   �  6   t   l   K   s   m   �  6   v   t   u   �  x       �  v   w   x   �  w   �     z   �   %   �     |   %   �   |     }   |   �        �   }   �  x   �  x   �     �   �   n      w   �     �   �   n   z   w   �  2   �  2   �     �   �   %   �  /   �  1   �  6   �   �      �  �       �  �   �   �   �  �   �     �      �   �     �   �   *   A  F   �   �      %   �   >  �   �   �  �   �  �   A  F   �   �            �     �   �   (      �   �  8                                                  BinLightsConfig               config                                      LightsInputList              lights                                      LightBitfields       	       bitfields         @                main                                            BinLightsConfig               config                                        LightsInputList              lights                                        LightBitfields       	       bitfields                                                                   BinLightsConfig               config                                      LightsInputList              lights                                      LightBitfields       	       bitfields         @                main                                            BinLightsConfig               config                                        LightsInputList              lights                                        LightBitfields       	       bitfields                                                                   BinLightsConfig               config                                      LightsInputList              lights                                      LightBitfields       	       bitfields         @                main                                            BinLightsConfig               config                                        LightsInputList              lights                                        LightBitfields       	       bitfields                            lights_bin.comp