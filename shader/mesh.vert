#version 430 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec4 color;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out vec4 shader_color;
out vec3 shader_normal;

void main()
{
    //gl_Position = projection * modelview * vec4(position, 1.f);
    gl_Position = projection * view * vec4(position, 1.f);
    shader_normal = normal;
    shader_color = color;
}
