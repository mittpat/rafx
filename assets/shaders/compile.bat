@echo off
setlocal
cd /D "%~dp0"
SET PATH=%PATH%;c:\dev\sdk\shaderc

glslc sprite.vert -o sprite.vert.spv
glslc sprite.frag -o sprite.frag.spv

glslc mesh.vert -o mesh.vert.spv
glslc mesh.frag -o mesh.frag.spv

glslc mesh_shadow_map.vert -o mesh_shadow_map.vert.spv

glslc debug.vert -o debug.vert.spv
glslc debug.frag -o debug.frag.spv

glslc bloom_extract.vert -o bloom_extract.vert.spv
glslc bloom_extract.frag -o bloom_extract.frag.spv

glslc bloom_blur.vert -o bloom_blur.vert.spv
glslc bloom_blur.frag -o bloom_blur.frag.spv

glslc bloom_combine.vert -o bloom_combine.vert.spv
glslc bloom_combine.frag -o bloom_combine.frag.spv

glslc imgui.vert -o imgui.vert.spv
glslc imgui.frag -o imgui.frag.spv