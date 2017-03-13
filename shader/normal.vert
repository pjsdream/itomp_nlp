#version 430 core

in vec3 position;
in vec3 normal;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

out vec3 surface_position;
out vec3 surface_normal;

void main()
{
    const vec4 world_position = model * vec4(position, 1.0);

    surface_position = vec3(world_position);
    surface_normal = mat3(model) * normal;

    gl_Position = projection * view * world_position;
}
