#version 430 core

in vec3 position;
in vec3 normal;
in vec2 texture_coords;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;
uniform mat4 light_projection_view;

out vec3 surface_position;
out vec3 surface_normal;
out vec2 pass_texture_coords;
out vec3 surface_position_light;

void main()
{
    const vec4 world_position = model * vec4(position, 1.0);
	
    surface_position = vec3(world_position);
    surface_normal = mat3(model) * normal;
    pass_texture_coords = texture_coords;
    surface_position_light = vec3(light_projection_view * world_position);

    gl_Position = projection * view * world_position;
}
