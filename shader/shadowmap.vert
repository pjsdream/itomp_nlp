#version 430 core

in vec3 position;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

out vec4 pass_position;

void main()
{
    gl_Position = projection * view * model * vec4(position, 1.0f);
    pass_position = gl_Position;
}
