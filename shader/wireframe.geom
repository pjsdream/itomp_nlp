#version 430

layout (triangles) in;
layout (line_strip, max_vertices = 4) out;

in vec3 surface_position[];

void main()
{
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();

    gl_Position = gl_in[1].gl_Position;
    EmitVertex();

    gl_Position = gl_in[2].gl_Position;
    EmitVertex();

    gl_Position = gl_in[0].gl_Position;
    EmitVertex();
}
