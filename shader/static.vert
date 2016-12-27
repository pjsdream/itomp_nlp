#version 430 core

in vec3 position;
in vec2 texture_coords;

uniform mat4 transformation_matrix;
uniform mat4 projection_matrix;

out vec2 pass_texture_coords;

void main()
{
    gl_Position = projection_matrix * transformation_matrix * vec4(position, 1.0);
    pass_texture_coords = texture_coords;
}
