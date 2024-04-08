                       à®""D@"¯ÉÈ¥TIÝ4            cbuffer PerViewData : register(b0, space0)
{
    row_major float4x4 per_view_data_view : packoffset(c0);
    row_major float4x4 per_view_data_view_proj : packoffset(c4);
};


static float4 gl_Position;
static float4x4 in_model_matrix;
static float3 in_pos;

struct SPIRV_Cross_Input
{
    float3 in_pos : POSITION;
    float4 in_model_matrix_0 : MODELMATRIX0;
    float4 in_model_matrix_1 : MODELMATRIX1;
    float4 in_model_matrix_2 : MODELMATRIX2;
    float4 in_model_matrix_3 : MODELMATRIX3;
};

struct SPIRV_Cross_Output
{
    float4 gl_Position : SV_Position;
};

void vert_main()
{
    float4x4 model_view_proj = mul(in_model_matrix, per_view_data_view_proj);
    gl_Position = mul(float4(in_pos, 1.0f), model_view_proj);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_model_matrix[0] = stage_input.in_model_matrix_0;
    in_model_matrix[1] = stage_input.in_model_matrix_1;
    in_model_matrix[2] = stage_input.in_model_matrix_2;
    in_model_matrix[3] = stage_input.in_model_matrix_3;
    in_pos = stage_input.in_pos;
    vert_main();
    SPIRV_Cross_Output stage_output;
    stage_output.gl_Position = gl_Position;
    return stage_output;
}
    5      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct PerViewData
{
    float4x4 view;
    float4x4 view_proj;
};

struct spvDescriptorSetBuffer0
{
    constant PerViewData* per_view_data [[id(0)]];
};

struct main0_out
{
    float4 gl_Position [[position]];
};

struct main0_in
{
    float3 in_pos [[attribute(0)]];
    float4 in_model_matrix_0 [[attribute(1)]];
    float4 in_model_matrix_1 [[attribute(2)]];
    float4 in_model_matrix_2 [[attribute(3)]];
    float4 in_model_matrix_3 [[attribute(4)]];
};

vertex main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    main0_out out = {};
    float4x4 in_model_matrix = {};
    in_model_matrix[0] = in.in_model_matrix_0;
    in_model_matrix[1] = in.in_model_matrix_1;
    in_model_matrix[2] = in.in_model_matrix_2;
    in_model_matrix[3] = in.in_model_matrix_3;
    float4x4 model_view_proj = (*spvDescriptorSet0.per_view_data).view_proj * in_model_matrix;
    out.gl_Position = model_view_proj * float4(in.in_pos, 1.0);
    return out;
}

          #     +                 GLSL.std.450                      main          !   H            H         #       H               H           H        #   @   H              G        G     "       G     !       G           H                H              H              H              G        G  !               !                                                              ;                       +                                   ;                        +                                                   ;           +                                    ;      !      +     #     ?   )         6               ø     A              =           =                         =     "   !   Q     $   "       Q     %   "      Q     &   "      P     '   $   %   &   #        (      '   A  )   *         >  *   (   ý  8                                               PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                      main                                          PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                     in_model_matrix       MODELMATRIX0          in_model_matrix       MODELMATRIX1          in_model_matrix       MODELMATRIX2          in_model_matrix       MODELMATRIX3          in_pos       POSITION                                                 PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                      main                                          PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                     in_model_matrix       MODELMATRIX0          in_model_matrix       MODELMATRIX1          in_model_matrix       MODELMATRIX2          in_model_matrix       MODELMATRIX3          in_pos       POSITION                                                 PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                      main                                          PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                     in_model_matrix       MODELMATRIX0          in_model_matrix       MODELMATRIX1          in_model_matrix       MODELMATRIX2          in_model_matrix       MODELMATRIX3          in_pos       POSITION      
       depth.vert