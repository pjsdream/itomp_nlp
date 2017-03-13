#version 430 core

in vec3 surface_color;

out vec4 out_color;

void main()
{
    out_color = vec4(surface_color, 1.0);
}
