#version 430 core

in vec4 frag_position;

uniform vec3 light_position;
uniform float far_plane;

void main()
{
    // get distance between fragment and light source
    float light_distance = length(frag_position.xyz - light_position);

    // map to [0;1] range by dividing by far_plane
    light_distance /= far_plane;

    // Write this as modified depth
    gl_FragDepth = light_distance;
}