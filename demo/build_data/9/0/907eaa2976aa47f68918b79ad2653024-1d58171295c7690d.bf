                       �""D@"���Ȟ,�bzjq١      �      struct TransformWithHistory
{
    row_major float4x4 current_model_matrix;
    row_major float4x4 previous_model_matrix;
};

struct DrawData
{
    uint transform_index;
    uint material_index;
};

cbuffer PerViewData : register(b0, space0)
{
    row_major float4x4 per_view_data_current_view_proj : packoffset(c0);
    row_major float4x4 per_view_data_current_view_proj_inv : packoffset(c4);
    row_major float4x4 per_view_data_previous_view_proj : packoffset(c8);
    uint per_view_data_viewport_width : packoffset(c12);
    uint per_view_data_viewport_height : packoffset(c12.y);
    float2 per_view_data_jitter_amount : packoffset(c12.z);
};

RWByteAddressBuffer all_transforms : register(u0, space1);
RWByteAddressBuffer all_draw_data : register(u1, space1);
cbuffer PushConstantData : register(b0, space2)
{
    uint push_constants_instance_offset : packoffset(c0);
};


static float4 in_old_position_clip;
static float4 in_new_position_clip;
static float2 out_velocity;

struct SPIRV_Cross_Input
{
    float4 in_old_position_clip : TEXCOORD0;
    float4 in_new_position_clip : TEXCOORD1;
};

struct SPIRV_Cross_Output
{
    float2 out_velocity : SV_Target0;
};

void frag_main()
{
    float2 old_position_ndc = in_old_position_clip.xy / abs(in_old_position_clip.w).xx;
    float2 new_position_ndc = in_new_position_clip.xy / abs(in_new_position_clip.w).xx;
    out_velocity = new_position_ndc - old_position_ndc;
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_old_position_clip = stage_input.in_old_position_clip;
    in_new_position_clip = stage_input.in_new_position_clip;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_velocity = out_velocity;
    return stage_output;
}
    �      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct PerViewData
{
    float4x4 current_view_proj;
    float4x4 current_view_proj_inv;
    float4x4 previous_view_proj;
    uint viewport_width;
    uint viewport_height;
    float2 jitter_amount;
};

struct TransformWithHistory
{
    float4x4 current_model_matrix;
    float4x4 previous_model_matrix;
};

struct AllTransforms
{
    TransformWithHistory transforms[1];
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
    float2 out_velocity [[color(0)]];
};

struct main0_in
{
    float4 in_old_position_clip [[user(locn0)]];
    float4 in_new_position_clip [[user(locn1)]];
};

fragment main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant spvDescriptorSetBuffer1& spvDescriptorSet1 [[buffer(1)]])
{
    main0_out out = {};
    float2 old_position_ndc = in.in_old_position_clip.xy / float2(abs(in.in_old_position_clip.w));
    float2 new_position_ndc = in.in_new_position_clip.xy / float2(abs(in.in_new_position_clip.w));
    out.out_velocity = new_position_ndc - old_position_ndc;
    return out;
}

    �      #     %                 GLSL.std.450                     main          !           G            G           G  !               !                              
                  
   ;                        +                       ;                        ;      !      6               �     =  
         O                     A              =                            P              �              =  
         O                     A              =                            P              �              �     $         >  !   $   �  8                   �                            PerViewData               PerViewData               PerViewData.current_view_proj    !       PerViewData.current_view_proj_inv@          PerViewData.previous_view_proj�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.jitter_amount�                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data                            main              �                            PerViewData               PerViewData               PerViewData.current_view_proj    !       PerViewData.current_view_proj_inv@          PerViewData.previous_view_proj�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.jitter_amount�    �                                      AllTransforms              all_transforms                                      AllDrawData             all_draw_data                                    �                            PerViewData               PerViewData               PerViewData.current_view_proj    !       PerViewData.current_view_proj_inv@          PerViewData.previous_view_proj�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.jitter_amount�                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data            ��������                 push_constants                                    main              �                            PerViewData               PerViewData               PerViewData.current_view_proj    !       PerViewData.current_view_proj_inv@          PerViewData.previous_view_proj�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.jitter_amount�    �                                      AllTransforms              all_transforms                                      AllDrawData             all_draw_data                                    �                            PerViewData               PerViewData               PerViewData.current_view_proj    !       PerViewData.current_view_proj_inv@          PerViewData.previous_view_proj�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.jitter_amount�                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data                            main              �                            PerViewData               PerViewData               PerViewData.current_view_proj    !       PerViewData.current_view_proj_inv@          PerViewData.previous_view_proj�          PerViewData.viewport_width�          PerViewData.viewport_height�          PerViewData.jitter_amount�    �                                      AllTransforms              all_transforms                                      AllDrawData             all_draw_data                            depth_velocity.frag