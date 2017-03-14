#version 430 core

in vec3 surface_position;
in vec3 surface_normal;
in vec2 pass_texture_coords;

uniform bool light_use[8];
uniform vec3 light_position[8];
uniform vec4 light_diffuse_color[8];
uniform vec4 light_specular_color[8];
uniform vec3 eye_position;
uniform mat4 light_projection_view[8];

uniform vec4 material_diffuse_color;
uniform vec4 material_specular_color;
uniform float shininess;

uniform bool has_texture;
uniform sampler2D texture_sampler;

uniform sampler2D shadow_map[8];

const float light_ambient = 0.;

out vec4 out_color;

float shadowCalculation(vec4 position_light, int light_index)
{
    vec3 projCoords = position_light.xyz / position_light.w;
    // Transform to [0,1] range
    projCoords = projCoords * 0.5f + 0.5f;
    // Get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;

    // Check whether current frag pos is in shadow
    const float bias = max(0.05f * (1.f - dot(normalize(surface_normal), normalize(light_position[light_index]))), 0.005f);

    float shadow = 0.0;
    if (projCoords.z <= 1.0f)
    {
        vec2 texelSize = 1.0 / textureSize(shadow_map[light_index], 0);
        for(int x = -1; x <= 1; ++x)
        {
            for(int y = -1; y <= 1; ++y)
            {
                float pcfDepth = texture(shadow_map[light_index], projCoords.xy + vec2(x, y) * texelSize).r; 
                shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;        
            }    
        }
        shadow /= 9.0;
    }

    return shadow;
}

void main()
{
    vec4 material_final_diffuse_color;
    if (has_texture)
        material_final_diffuse_color = texture(texture_sampler, pass_texture_coords);
    else
        material_final_diffuse_color = material_diffuse_color;

    vec3 N = normalize(surface_normal);
    vec3 V = normalize(eye_position - surface_position);

    vec4 total_color = vec4(0, 0, 0, 0);

    // TODO: (ambient color) = light_ambient_color * material_ambient_color * light_ambient;
    // For now, it is preset
    total_color = material_final_diffuse_color * 0.2;

    for (int i=0; i<8; i++)
    {
        if (light_use[i])
        {
            // This is for point light:
            // vec3 L = normalize(light_position[i] - surface_position);

            // This is for directional light:
            vec3 L = normalize(light_position[i]);

            vec3 R = - L + 2.0 * dot(L, N) * N;

            float NdotL = dot(N, L);
            float light_diffuse = clamp(NdotL, 0, 1);

            float VdotR = dot(V, R);
            float light_specular = pow( clamp(VdotR, 0, 1), shininess );

            vec4 phong_color = light_diffuse_color[i] * material_final_diffuse_color * light_diffuse
                             + light_specular_color[i] * material_specular_color * light_specular;

            float shadow = shadowCalculation(light_projection_view[i] * vec4(surface_position, 1.0f), i);

            total_color += (1.f - shadow) * phong_color;
        }
    }

    out_color = vec4(total_color.xyz, 1.0);


/*
    vec3 N = normalize(surface_normal);
    vec3 L = normalize(light_position - surface_position);
    vec3 R = - L + 2.0 * dot(L, N) * N;
    vec3 V = normalize(eye_position - surface_position);

    float NdotL = dot(N, L);
    float light_diffuse = clamp(NdotL, 0, 1);

    float VdotR = dot(V, R);
    float light_specular = pow( clamp(VdotR, 0, 1), shininess );

    vec4 material_final_diffuse_color;
    if (has_texture)
        material_final_diffuse_color = texture(texture_sampler, pass_texture_coords);
    else
        material_final_diffuse_color = material_diffuse_color ;

    vec4 phong_color = light_ambient_color * material_ambient_color * light_ambient
                     + light_diffuse_color * material_final_diffuse_color * light_diffuse
                     + light_specular_color * material_specular_color * light_specular;

    out_color = vec4(phong_color.xyz, 1.0);
*/
}
