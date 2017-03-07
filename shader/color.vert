#version 430 core

in vec3 position;
in vec3 color;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

out vec3 surface_color;

void main()
{
    const vec4 world_position = model * vec4(position, 1.0);
	
    surface_color = color;

    gl_Position = projection * view * world_position;
}
