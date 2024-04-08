                       à®""D@"¯ÉÈ'ÚõrA            static const float2 _20[4] = { (-1.0f).xx, float2(-1.0f, 1.0f), float2(1.0f, -1.0f), 1.0f.xx };

Texture2D<float4> tex : register(t0, space0);
SamplerState smp : register(s1, space0);

static float2 inUV;
static float4 out_sdr;
static float4 out_bloom;

struct SPIRV_Cross_Input
{
    float2 inUV : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 out_sdr : SV_Target0;
    float4 out_bloom : SV_Target1;
};

uint2 spvTextureSize(Texture2D<float4> Tex, uint Level, out uint Param)
{
    uint2 ret;
    Tex.GetDimensions(Level, ret.x, ret.y, Param);
    return ret;
}

void frag_main()
{
    float3 c = tex.Sample(smp, inUV).xyz;
    out_sdr = float4(c, 1.0f);
    float luminance = dot(c, float3(0.2125999927520751953125f, 0.715200006961822509765625f, 0.072200000286102294921875f));
    float weight = 1.0f / (luminance + 1.0f);
    float3 color = c * weight;
    float weightSum = weight;
    uint _74_dummy_parameter;
    float2 tex_offset = 1.0f.xx / float2(int2(spvTextureSize(tex, uint(0), _74_dummy_parameter)));
    int i = 0;
    for (;;)
    {
        if (i < 4)
        {
            float3 c_1 = tex.Sample(smp, inUV + (_20[i] * tex_offset)).xyz;
            luminance = dot(c_1, float3(0.2125999927520751953125f, 0.715200006961822509765625f, 0.072200000286102294921875f));
            float weight_1 = 1.0f / (luminance + 1.0f);
            color += (c_1 * weight_1);
            weightSum += weight_1;
            i++;
            continue;
        }
        else
        {
            break;
        }
    }
    color /= weightSum.xxx;
    luminance = dot(color, float3(0.2125999927520751953125f, 0.715200006961822509765625f, 0.072200000286102294921875f));
    if (luminance > 1.0f)
    {
        out_bloom = float4(color, 1.0f);
    }
    else
    {
        out_bloom = float4(0.0f, 0.0f, 0.0f, 1.0f);
    }
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    inUV = stage_input.inUV;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_sdr = out_sdr;
    stage_output.out_bloom = out_bloom;
    return stage_output;
}
    R      #pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-braces"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

template<typename T, size_t Num>
struct spvUnsafeArray
{
    T elements[Num ? Num : 1];
    
    thread T& operator [] (size_t pos) thread
    {
        return elements[pos];
    }
    constexpr const thread T& operator [] (size_t pos) const thread
    {
        return elements[pos];
    }
    
    device T& operator [] (size_t pos) device
    {
        return elements[pos];
    }
    constexpr const device T& operator [] (size_t pos) const device
    {
        return elements[pos];
    }
    
    constexpr const constant T& operator [] (size_t pos) const constant
    {
        return elements[pos];
    }
    
    threadgroup T& operator [] (size_t pos) threadgroup
    {
        return elements[pos];
    }
    constexpr const threadgroup T& operator [] (size_t pos) const threadgroup
    {
        return elements[pos];
    }
};

struct spvDescriptorSetBuffer0
{
    texture2d<float> tex [[id(0)]];
};

constant spvUnsafeArray<float2, 4> _20 = spvUnsafeArray<float2, 4>({ float2(-1.0), float2(-1.0, 1.0), float2(1.0, -1.0), float2(1.0) });

struct main0_out
{
    float4 out_sdr [[color(0)]];
    float4 out_bloom [[color(1)]];
};

struct main0_in
{
    float2 inUV [[user(locn0)]];
};

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    constexpr sampler smp(mip_filter::linear, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    float3 c = spvDescriptorSet0.tex.sample(smp, in.inUV).xyz;
    out.out_sdr = float4(c, 1.0);
    float luminance = dot(c, float3(0.2125999927520751953125, 0.715200006961822509765625, 0.072200000286102294921875));
    float weight = 1.0 / (luminance + 1.0);
    float3 color = c * weight;
    float weightSum = weight;
    float2 tex_offset = float2(1.0) / float2(int2(spvDescriptorSet0.tex.get_width(), spvDescriptorSet0.tex.get_height()));
    int i = 0;
    for (;;)
    {
        if (i < 4)
        {
            float3 c_1 = spvDescriptorSet0.tex.sample(smp, (in.inUV + (_20[i] * tex_offset))).xyz;
            luminance = dot(c_1, float3(0.2125999927520751953125, 0.715200006961822509765625, 0.072200000286102294921875));
            float weight_1 = 1.0 / (luminance + 1.0);
            color += (c_1 * weight_1);
            weightSum += weight_1;
            i++;
            continue;
        }
        else
        {
            break;
        }
    }
    color /= float3(weightSum);
    luminance = dot(color, float3(0.2125999927520751953125, 0.715200006961822509765625, 0.072200000286102294921875));
    if (luminance > 1.0)
    {
        out.out_bloom = float4(color, 1.0);
    }
    else
    {
        out.out_bloom = float4(0.0, 0.0, 0.0, 1.0);
    }
    return out;
}

    <      #                   2        GLSL.std.450                     main    "   (              G     "       G     !       G     "       G     !      G  "          G  (          G                !                                         +     	        
      	            
   +          ¿,              +          ?,              ,              ,              ,  
                              	                                          ;                              ;                       !         ;  !   "        $            '      $   ;  '   (      +     1   Ð³Y>+     2   Y7?+     3   Ý=,     4   1   2   3      @           E          +  E   F         H   E      +  E   U        V   +  E   t      ;  '         +            ,  $                  6               ø     ;           >        =           =           V               =     #   "   W  $   %       #   O     &   %   %             Q     *   %       Q     +   %      Q     ,   %      P  $   -   *   +   ,      >  (   -        5   &   4        8   5           9      8        =   &   9   V     D         d     G   D   g  H   I   G   F   o     J   I        L      J   ù  O   ø  O   õ        =      o   P   õ        9      r   P   õ  E      F      u   P   ±  V   W      U   ö  Q   P       ú  W   P   Q   ø  P   V     [         A  @   ^         =     _   ^        b      2   _   L   #   W  $   c   [   b   O     d   c   c                  f   d   4        i   f           j      i        m   d   j        o      m        r      j     E   u      t   ù  O   ø  Q   P     x                 y      x        {   y   4   º  V   }   {      ÷         ú  }   ~      ø  ~   Q        y       Q        y      Q        y      P  $                  >        ù     ø     >        ù     ø     ý  8                                               tex               tex                                    smp              smp                            main                                          tex               tex                                      smp              smp                                                                                                      tex               tex                                    smp              smp                            main                                          tex               tex                                      smp              smp                                                                                                      tex               tex                                    smp              smp                            main                                          tex               tex                                      smp              smp                                                                  bloom_extract.frag