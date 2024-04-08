                       �""D@"���Ȟ�d}w�0�f          #version 100
precision mediump float;
precision highp int;

struct UniformData_UniformBlock
{
    highp vec4 uniform_color;
};

uniform UniformData_UniformBlock UniformData;

varying highp vec4 interface_var_0;

void main()
{
    gl_FragData[0] = UniformData.uniform_color;
}

          #version 300 es
precision mediump float;
precision highp int;

layout(std140) uniform UniformData
{
    highp vec4 uniform_color;
} uniform_data;

layout(location = 0) out highp vec4 out_color;
in highp vec4 interface_var_0;

void main()
{
    out_color = uniform_data.uniform_color;
}

    8      cbuffer UniformData : register(b0, space0)
{
    float4 uniform_data_uniform_color : packoffset(c0);
};


static float4 out_color;
static float4 in_color;

struct SPIRV_Cross_Input
{
    float4 in_color : TEXCOORD0;
};

struct SPIRV_Cross_Output
{
    float4 out_color : SV_Target0;
};

void frag_main()
{
    out_color = uniform_data_uniform_color;
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_color = stage_input.in_color;
    frag_main();
    SPIRV_Cross_Output stage_output;
    stage_output.out_color = out_color;
    return stage_output;
}
    �      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct UniformData
{
    float4 uniform_color;
};

struct spvDescriptorSetBuffer0
{
    constant UniformData* uniform_data [[id(0)]];
};

struct main0_out
{
    float4 out_color [[color(0)]];
};

fragment main0_out main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]])
{
    main0_out out = {};
    out.out_color = (*spvDescriptorSet0.uniform_data).uniform_color;
    return out;
}

    �      #                      GLSL.std.450                     main    
                   glsl\shader.frag     }    �     // OpModuleProcessed entry-point main
// OpModuleProcessed client vulkan100
// OpModuleProcessed target-env vulkan1.0
// OpModuleProcessed entry-point main
#line 1
#version 450

// @[internal_buffer]
layout (set = 0, binding = 0) uniform UniformData {
    vec4 uniform_color;
} uniform_data;

layout (location = 0) in vec4 in_color;

layout (location = 0) out vec4 out_color;

void main() {
    //out_color = in_color;
    out_color = uniform_data.uniform_color;
}
     
 GL_GOOGLE_cpp_style_line_directive    GL_GOOGLE_include_directive      main      
   out_color        UniformData          uniform_color        uniform_data         in_color    G  
          H         #       G        G     "       G     !       G                 !                               	         ;  	   
                          ;                       +                                    ;                      6               �                 A              =           >  
      �  8                   �                            UniformData               UniformData               UniformData.uniform_color                       main              �                            UniformData               UniformData               UniformData.uniform_color                                 �                            UniformData               UniformData               UniformData.uniform_color                       main              �                            UniformData               UniformData               UniformData.uniform_color                                 �                            UniformData               UniformData               UniformData.uniform_color                       main              �                            UniformData               UniformData               UniformData.uniform_color                                 �                            UniformData               UniformData               UniformData.uniform_color                       main              �                            UniformData               UniformData               UniformData.uniform_color                                 �                            UniformData               UniformData               UniformData.uniform_color                       main              �                            UniformData               UniformData               UniformData.uniform_color                       shader.frag