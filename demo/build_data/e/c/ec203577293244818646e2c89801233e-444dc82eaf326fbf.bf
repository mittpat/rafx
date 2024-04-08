                       �""D@"���ȞbRp�"N�            static float4 gl_Position;
static float2 in_pos;

struct SPIRV_Cross_Input
{
    float2 in_pos : POSITION;
};

struct SPIRV_Cross_Output
{
    float4 gl_Position : SV_Position;
};

void vert_main()
{
    float2 clip_space = (in_pos * 2.0f) - 1.0f.xx;
    gl_Position = float4(clip_space.x, -clip_space.y, 0.0f, 1.0f);
}

SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
{
    in_pos = stage_input.in_pos;
    vert_main();
    SPIRV_Cross_Output stage_output;
    stage_output.gl_Position = gl_Position;
    return stage_output;
}
    �      #include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct main0_out
{
    float4 gl_Position [[position]];
};

struct main0_in
{
    float2 in_pos [[attribute(0)]];
};

vertex main0_out main0(main0_in in [[stage_in]])
{
    main0_out out = {};
    float2 clip_space = (in.in_pos * 2.0) - float2(1.0);
    out.gl_Position = float4(clip_space.x, -clip_space.y, 0.0, 1.0);
    return out;
}

    �      #     +                 GLSL.std.450                      main          G            H                H              H              H              G             !                               
         ;  
         +           @+          �?                        +                      +                                                   ;                       +            +     $          &         ,     *         6               �     =           �              �           *   Q                Q     "              #   "   P     %       #   $      A  &   '         >  '   %   �  8                                       main                      in_pos       POSITION                                         main                      in_pos       POSITION                                         main                      in_pos       POSITION             shadow_atlas_clear_tiles.vert