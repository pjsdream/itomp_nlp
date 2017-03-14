#version 430 core

in vec4 pass_position;

out vec4 out_color;

void main()
{
    float depth = pass_position.z;
    float grey = depth;

    out_color = vec4(vec3(grey), 1.0f);
}
