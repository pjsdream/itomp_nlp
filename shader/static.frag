#version 430 core

in vec2 pass_texture_coords;

out vec4 out_color;

uniform sampler2D texture_sampler;

void main()
{
    out_color = texture(texture_sampler, pass_texture_coords);
}
