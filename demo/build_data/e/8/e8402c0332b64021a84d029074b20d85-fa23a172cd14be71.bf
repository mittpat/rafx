                       ą®""D@"ÆÉČūXi"Ł=      '      struct DrawData
{
    uint transform_index;
    uint material_index;
};

struct Transform
{
    row_major float4x4 model_matrix;
};

RWByteAddressBuffer all_draw_data : register(u1, space1);
RWByteAddressBuffer all_transforms : register(u0, space1);
cbuffer PerViewData : register(b0, space0)
{
    row_major float4x4 per_view_data_view : packoffset(c0);
    row_major float4x4 per_view_data_view_proj : packoffset(c4);
};

cbuffer PushConstantData : register(b0, space2)
{
    uint push_constants_instance_offset : packoffset(c0);
};


static float4 gl_Position;
static float3 in_pos;

struct SPIRV_Cross_Input
{
    float3 in_pos : POSITION;
};

struct SPIRV_Cross_Output
{
    float4 gl_Position : SV_Position;
};

void vert_main()
{
    uint instance_index = push_constants_instance_offset;
    if (instance_index > all_draw_data.Load(0))
    {
        instance_index = 0u;
    }
    DrawData _41;
    _41.transform_index = all_draw_data.Load(instance_index * 8 + 16);
    _41.material_index = all_draw_data.Load(instance_index * 8 + 20);
    DrawData draw_data;
    draw_data.transform_index = _41.transform_index;
    draw_data.material_index = _41.material_index;
    float4x4 _61 = asfloat(uint4x4(all_transforms.Load4(draw_data.transform_index * 64 + 0), all_transforms.Load4(draw_data.transform_index * 64 + 16), all_transforms.Load4(draw_data.transform_index * 64 + 32), all_transforms.Load4(draw_data.transform_index * 64 + 48)));
    float4x4 model_matrix = _61;
    float4x4 model_view_proj = mul(model_matrix, per_view_data_view_proj);
    gl_Position = mul(float4(in_pos, 1.0f), model_view_proj);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_pos = stage_input.in_pos;
    vert_main();
    SPIRV_Cross_Output stage_output;
    stage_output.gl_Position = gl_Position;
    return stage_output;
}
          #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct DrawData
{
    uint transform_index;
    uint material_index;
};

struct DrawData_1
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
    DrawData_1 draw_data[1];
};

struct Transform
{
    float4x4 model_matrix;
};

struct AllTransforms
{
    Transform transforms[1];
};

struct PerViewData
{
    float4x4 view;
    float4x4 view_proj;
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
    float4 gl_Position [[position]];
};

struct main0_in
{
    float3 in_pos [[attribute(0)]];
};

vertex main0_out main0(main0_in in [[stage_in]], constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant spvDescriptorSetBuffer1& spvDescriptorSet1 [[buffer(1)]], uint gl_InstanceIndex [[instance_id]])
{
    main0_out out = {};
    uint instance_index = uint(int(gl_InstanceIndex));
    DrawData draw_data;
    draw_data.transform_index = (*spvDescriptorSet1.all_draw_data).draw_data[instance_index].transform_index;
    draw_data.material_index = (*spvDescriptorSet1.all_draw_data).draw_data[instance_index].material_index;
    float4x4 model_matrix = (*spvDescriptorSet1.all_transforms).transforms[draw_data.transform_index].model_matrix;
    float4x4 model_view_proj = (*spvDescriptorSet0.per_view_data).view_proj * model_matrix;
    out.gl_Position = model_view_proj * float4(in.in_pos, 1.0);
    return out;
}

    ¬      #     P                 GLSL.std.450                      main       <   @   G        +   H         #       H        #      G           H         #       H        #      H        #      H        #      H        #      G        G     "      G     !      H  &          H  &       #       H  &             G  '      @   H  (       #       G  (      G  *   "      G  *   !       H  1          H  1       #       H  1             H  1         H  1      #   @   H  1            G  1      G  3   "       G  3   !       H  :              H  :            H  :            H  :            G  :      G  @               !                       	             
      	   ;  
                                                            ;           +  	                     +  	          +  	           !         "   !        #   "        &   #     '   &     (   '      )      (   ;  )   *         -      #     1   #   #      2      1   ;  2   3      +     8        9   !   8     :   "   !   9   9      ;      :   ;  ;   <        >   !         ?      >   ;  ?   @      +  !   B     ?   H      "      L         +     M       6               ų     =  	         |           A                 A  L   N      M   =     O   N   A  -   .   *      O      =  #   /   .   A  -   4   3      =  #   5   4     #   7   5   /   =  >   A   @   Q  !   C   A       Q  !   D   A      Q  !   E   A      P  "   F   C   D   E   B     "   G   7   F   A  H   I   <      >  I   G   ż  8                                               PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data                            main                                          PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                                          AllTransforms              all_transforms                                      AllDrawData             all_draw_data                         in_pos       POSITION                                                 PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data            ’’’’’’’’                 push_constants                                    main                                          PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                                          AllTransforms              all_transforms                                      AllDrawData             all_draw_data                         in_pos       POSITION                                                 PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                               AllTransforms              all_transforms                                    AllDrawData             all_draw_data                            main                                          PerViewData               PerViewData               PerViewData.view           PerViewData.view_proj@                                          AllTransforms              all_transforms                                      AllDrawData             all_draw_data                         in_pos       POSITION             mesh_adv_wireframe.vert