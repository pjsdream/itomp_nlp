#version 430 core

in vec4 shader_color;
in vec3 shader_normal;

layout (location = 0) out vec4 color;

void main()
{
    color = shader_color;
}
