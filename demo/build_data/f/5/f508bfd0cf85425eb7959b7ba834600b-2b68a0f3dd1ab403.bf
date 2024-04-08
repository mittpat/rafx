                       �""D@"���Ȟ������      <      struct Transform
{
    row_major float4x4 model_matrix;
};

struct DrawData
{
    uint transform_index;
    uint material_index;
};

cbuffer PerViewData : register(b0, space0)
{
    row_major float4x4 per_view_data_view : packoffset(c0);
    row_major float4x4 per_view_data_view_proj : packoffset(c4);
};

RWByteAddressBuffer all_transforms : register(u0, space1);
RWByteAddressBuffer all_draw_data : register(u1, space1);
cbuffer PushConstantData : register(b0, space2)
{
    uint push_constants_instance_offset : packoffset(c0);
};


static float4 out_color;

struct SPIRV_Cross_Output
{
    float4 out_color : SV_Target0;
};

void frag_main()
{
    out_color = 1.0f.xxxx;
}

SPIRV_Cross_Output main()
{
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_color = out_color;
    return stage_output;
}
    �      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct PerViewData
{
    float4x4 view;
    float4x4 view_proj;
};

struct Transform
{
    float4x4 model_matrix;
};

struct AllTransforms
{
    Transform transforms[1];
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

struct spvDescriptorSetBuffer0
{
    constant PerViewData* per_view_data [[id(0)]];
};

struct spvDescriptorSetBuffer1
{
    device AllTransforms* all_transforms [[id(0)]];
    device AllDrawData* all_draw_data [[id(1)]];
};

struct main0_out
{
    float4 out_color [[color(0)]];
};

fragment main0_out main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant spvDescriptorSetBuffer1& spvDescriptorSet1 [[buffer(1)]])
{
    main0_out out = {};
    out.out_color = float4(1.0);
    return out;
}

           #                      GLSL.std.450                     main    	           G  	               !                                        ;     	      +     
     �?,        
   
   
   
   6               �     >  	      �  8                   �                            PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data                            main              �                            PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@    �                                      AllTransforms              all_transforms                                      AllDrawData             all_draw_data                                    �                            PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data            ��������                 push_constants                                    main              �                            PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@    �                                      AllTransforms              all_transforms                                      AllDrawData             all_draw_data                                    �                            PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data                            main              �                            PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@    �                                      AllTransforms              all_transforms                                      AllDrawData             all_draw_data                            mesh_adv_wireframe.frag