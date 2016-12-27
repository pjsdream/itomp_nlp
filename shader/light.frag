#version 430 core

in vec2 pass_texture_coords;
in vec3 surface_position;
in vec3 surface_normal;

uniform vec3 light_position;
uniform vec3 light_ambient_color;
uniform vec3 light_diffuse_color;
uniform vec3 light_specular_color;
uniform float light_ambient;
uniform vec3 eye_position;

uniform vec3 material_ambient_color;
uniform vec3 material_specular_color;

uniform sampler2D texture_sampler;

const float shininess = 10.0;

out vec4 out_color;

void main()
{
    vec3 N = normalize(surface_normal);
    vec3 L = normalize(light_position - surface_position);
    vec3 R = - L + 2.0 * dot(L, N) * N;
    vec3 V = normalize(eye_position - surface_position);

    float NdotL = dot(N, L);
    float light_diffuse = clamp(NdotL, 0, 1);

    float VdotR = dot(V, R);
    float light_specular = pow( clamp(VdotR, 0, 1), shininess );

    vec3 material_diffuse_color = vec3( texture(texture_sampler, pass_texture_coords) );

    vec3 phong_color = light_ambient_color * material_ambient_color * light_ambient
                     + light_diffuse_color * material_diffuse_color * light_diffuse
                     + light_specular_color * material_specular_color * light_specular;

    out_color = vec4(phong_color, 1.0);
}
