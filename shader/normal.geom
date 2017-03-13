#version 430

layout (points) in;
layout (line_strip, max_vertices = 2) out;

in vec3 surface_position[];
in vec3 surface_normal[];

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

uniform float line_length;

void main()
{
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();

    gl_Position = projection * view * vec4(surface_position[0] + line_length * surface_normal[0], 1.0);
    EmitVertex();
}
