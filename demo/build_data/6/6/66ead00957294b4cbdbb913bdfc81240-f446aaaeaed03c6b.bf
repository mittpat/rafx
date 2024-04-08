                       �""D@"���Ȟ`����    �       #version 100

varying vec4 interface_var_0;
attribute vec4 in_color;
attribute vec4 pos;

void main()
{
    interface_var_0 = in_color;
    gl_Position = pos;
    gl_Position.z = 2.0 * gl_Position.z - gl_Position.w;
    gl_Position.y = -gl_Position.y;
}

          #version 300 es

out vec4 interface_var_0;
layout(location = 1) in vec4 in_color;
layout(location = 0) in vec4 pos;

void main()
{
    interface_var_0 = in_color;
    gl_Position = pos;
    gl_Position.z = 2.0 * gl_Position.z - gl_Position.w;
    gl_Position.y = -gl_Position.y;
}

    �      static float4 gl_Position;
static float4 out_color;
static float4 in_color;
static float4 pos;

struct SPIRV_Cross_Input
{
    float4 pos : POSITION;
    float4 in_color : COLOR;
};

struct SPIRV_Cross_Output
{
    float4 out_color : TEXCOORD0;
    float4 gl_Position : SV_Position;
};

void vert_main()
{
    out_color = in_color;
    gl_Position = pos;
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_color = stage_input.in_color;
    pos = stage_input.pos;
    vert_main();
    SPIRV_Cross_Output stage_output;
    stage_output.gl_Position = gl_Position;
    stage_output.out_color = out_color;
    return stage_output;
}
    �      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct main0_out
{
    float4 out_color [[user(locn0)]];
    float4 gl_Position [[position]];
};

struct main0_in
{
    float4 pos [[attribute(0)]];
    float4 in_color [[attribute(1)]];
};

vertex main0_out main0(main0_in in [[stage_in]])
{
    main0_out out = {};
    out.out_color = in.in_color;
    out.gl_Position = in.pos;
    return out;
}

    $      #                      GLSL.std.450              	        main    
                 glsl\shader.vert     �    �     // OpModuleProcessed entry-point main
// OpModuleProcessed client vulkan100
// OpModuleProcessed target-env vulkan1.0
// OpModuleProcessed entry-point main
#line 1
#version 450

// These "semantic" annotations are matched with vertex data, allowing pipelines to be produced as needed for whatever
// data is in use at runtime

// @[semantic("POSITION")]
layout (location = 0) in vec4 pos;
// @[semantic("COLOR")]
layout (location = 1) in vec4 in_color;

layout (location = 0) out vec4 out_color;

void main() {
    out_color = in_color;
    gl_Position = pos;
}
   
 GL_GOOGLE_cpp_style_line_directive    GL_GOOGLE_include_directive      main      
   out_color        in_color         gl_PerVertex             gl_Position         gl_PointSize            gl_ClipDistance         gl_CullDistance               pos G  
          G           H                H              H              H              G        G                 !                               	         ;  	   
                  ;                        +                                                   ;                       +            ;                      6               �                 =           >  
                  =           A  	            >        �  8                                       main                      in_color       COLOR          pos       POSITION                                         main                      in_color       COLOR          pos       POSITION                                         main                      in_color       COLOR          pos       POSITION                                         main                      in_color       COLOR          pos       POSITION                                         main                      in_color       COLOR          pos       POSITION           shader.vert