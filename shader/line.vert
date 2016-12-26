#version 430 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec4 color;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out vec4 shader_color;

void main()
{
    gl_Position = projection * view * vec4(position, 1.f);
    shader_color = color;
}
