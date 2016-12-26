#version 430 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec4 color;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

out gl_PerVertex
{
    vec4 gl_Position;
};

out vec4 surface_color;
out vec3 frag_position;
out vec3 frag_normal;

void main(void)
{
    surface_color = color;

    vec4 object_pos = vec4(position, 1.f);
    vec4 world_pos = model_matrix * object_pos;
    frag_position = world_pos.xyz;
    frag_normal = mat3(model_matrix) * normal;

    gl_Position = (projection_matrix * view_matrix) * world_pos;
}
