                       à®""D@"¯ÉÈ-ÚPÜV      '      static const float _18[5] = { 0.227026998996734619140625f, 0.19459460675716400146484375f, 0.121621601283550262451171875f, 0.054053999483585357666015625f, 0.01621600054204463958740234375f };

cbuffer Config : register(b2, space0)
{
    uint config_horizontal : packoffset(c0);
};

Texture2D<float4> tex : register(t0, space0);
SamplerState smp : register(s1, space0);

static float2 inUV;
static float4 out_blur;

struct SPIRV_Cross_Input
{
    float2 inUV : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 out_blur : SV_Target0;
};

uint2 spvTextureSize(Texture2D<float4> Tex, uint Level, out uint Param)
{
    uint2 ret;
    Tex.GetDimensions(Level, ret.x, ret.y, Param);
    return ret;
}

void frag_main()
{
    uint _37_dummy_parameter;
    float2 tex_offset = 1.0f.xx / float2(int2(spvTextureSize(tex, uint(0), _37_dummy_parameter)));
    float3 result = tex.Sample(smp, inUV).xyz * _18[0];
    if (config_horizontal != 0u)
    {
        int i = 1;
        for (;;)
        {
            if (i < 5)
            {
                result += (tex.Sample(smp, inUV + float2(tex_offset.x * float(i), 0.0f)).xyz * _18[i]);
                result += (tex.Sample(smp, inUV - float2(tex_offset.x * float(i), 0.0f)).xyz * _18[i]);
                i++;
                continue;
            }
            else
            {
                break;
            }
        }
    }
    else
    {
        int i_1 = 1;
        for (;;)
        {
            if (i_1 < 5)
            {
                result += (tex.Sample(smp, inUV + float2(0.0f, tex_offset.y * float(i_1))).xyz * _18[i_1]);
                result += (tex.Sample(smp, inUV - float2(0.0f, tex_offset.y * float(i_1))).xyz * _18[i_1]);
                i_1++;
                continue;
            }
            else
            {
                break;
            }
        }
    }
    out_blur = float4(result, 1.0f);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    inUV = stage_input.inUV;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_blur = out_blur;
    return stage_output;
}
    é      #pragma clang diagnostic ignored "-Wmissing-prototypes"
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

struct Config
{
    uint horizontal;
};

struct spvDescriptorSetBuffer0
{
    texture2d<float> tex [[id(0)]];
    constant Config* config [[id(2)]];
};

constant spvUnsafeArray<float, 5> _18 = spvUnsafeArray<float, 5>({ 0.227026998996734619140625, 0.19459460675716400146484375, 0.121621601283550262451171875, 0.054053999483585357666015625, 0.01621600054204463958740234375 });

struct main0_out
{
    float4 out_blur [[color(0)]];
};

struct main0_in
{
    float2 inUV [[user(locn0)]];
};

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    constexpr sampler smp(mip_filter::linear, compare_func::never, max_anisotropy(1));
    main0_out out = {};
    float2 tex_offset = float2(1.0) / float2(int2(spvDescriptorSet0.tex.get_width(), spvDescriptorSet0.tex.get_height()));
    float3 result = spvDescriptorSet0.tex.sample(smp, in.inUV).xyz * _18[0];
    if ((*spvDescriptorSet0.config).horizontal != 0u)
    {
        int i = 1;
        for (;;)
        {
            if (i < 5)
            {
                result += (spvDescriptorSet0.tex.sample(smp, (in.inUV + float2(tex_offset.x * float(i), 0.0))).xyz * _18[i]);
                result += (spvDescriptorSet0.tex.sample(smp, (in.inUV - float2(tex_offset.x * float(i), 0.0))).xyz * _18[i]);
                i++;
                continue;
            }
            else
            {
                break;
            }
        }
    }
    else
    {
        int i_1 = 1;
        for (;;)
        {
            if (i_1 < 5)
            {
                result += (spvDescriptorSet0.tex.sample(smp, (in.inUV + float2(0.0, tex_offset.y * float(i_1)))).xyz * _18[i_1]);
                result += (spvDescriptorSet0.tex.sample(smp, (in.inUV - float2(0.0, tex_offset.y * float(i_1)))).xyz * _18[i_1]);
                i_1++;
                continue;
            }
            else
            {
                break;
            }
        }
    }
    out.out_blur = float4(result, 1.0);
    return out;
}

    X
      #     º              2        GLSL.std.450                     main    /   ª           G     "       G     !       G     "       G     !      G  /          H  8       #       G  8      G  :   "       G  :   !      G  ª               !                              +             	            
      	   +        Äyh>+        ÏCG>+        ¿ù=+        ºg]=+        k×<,  	                                +          ? 	                                          ;                              ;                                 +      !         #            (            .         ;  .   /        1            4           8         9      8   ;  9   :         ;           >   +     ?       +      E      +      L      +     W          ©      1   ;  ©   ª      ,     ¹         6               ø     ;  
         >        =           =           V              d     "      g  #   $   "   !   o     %   $        '   ¹   %   V     -         =     0   /   W  1   2   -   0   O  (   3   2   2             A  4   5      !   =     6   5     (   7   3   6   A  ;   <   :   !   =     =   <   «  >   @   =   ?   ÷  B       ú  @   A   w   ø  A   ù  F   ø  F   õ  (   ¸   7   A   t   G   õ      ¶   E   A   v   G   ±  >   M   ¶   L   ö  H   G       ú  M   G   H   ø  G   V     P         Q     S   '       o     U   ¶        V   S   U   P     X   V   W        Y   0   X   W  1   Z   P   Y   O  (   [   Z   Z             A  4   ]      ¶   =     ^   ]     (   _   [   ^     (   a   ¸   _   V     d              l   0   X   W  1   m   d   l   O  (   n   m   m             =     q   ]     (   r   n   q     (   t   a   r         v   ¶   E   ù  F   ø  H   ù  B   ø  w   ù  y   ø  y   õ  (   µ   7   w   ¦   z   õ      ´   E   w   ¨   z   ±  >      ´   L   ö  {   z       ú     z   {   ø  z   V              Q        '      o        ´                 P        W              0      W  1            O  (                      A  4         ´   =             (              (      µ      V                      0      W  1            O  (                       =     £        (   ¤       £     (   ¦      ¤         ¨   ´   E   ù  y   ø  {   ù  B   ø  B   õ  (   ·   ¸   H   µ   {   Q     ¬   ·       Q     ­   ·      Q     ®   ·      P  1   ¯   ¬   ­   ®      >  ª   ¯   ı  8                                               tex               tex                                    smp              smp                                    Config              Config               Config.horizontal                       main                                          tex               tex                                      smp              smp                                                                            Config              Config               Config.horizontal                                                             tex               tex                                    smp              smp                                    Config              Config               Config.horizontal                       main                                          tex               tex                                      smp              smp                                                                            Config              Config               Config.horizontal                                                             tex               tex                                    smp              smp                                    Config              Config               Config.horizontal                       main                                          tex               tex                                      smp              smp                                                                            Config              Config               Config.horizontal                         bloom_blur.frag