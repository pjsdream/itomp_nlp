#version 430 core

in vec3 surface_position;
in vec3 surface_normal;
in vec2 pass_texture_coords;

// OIT buffers
layout (binding = 0, r32ui) uniform uimage2D head_pointer_image;
layout (binding = 1, rgba32ui) uniform writeonly uimageBuffer list_buffer;

layout (binding = 0, offset = 0) uniform atomic_uint list_counter;

struct DirectionalLight
{
    bool use;

    vec3 position;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

struct PointLight
{
    bool use;

    vec3 position;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;

    vec3 attenuation;
};

struct Material
{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;

    float alpha; // opaqueness

    bool has_texture;
    sampler2D diffuse_texture;
};

uniform DirectionalLight directional_lights[8];
uniform PointLight point_lights[8];

uniform vec3 eye_position;
uniform Material material;

void main()
{
    // OIT variables
    uint index;
    uint old_head;
    uvec4 item;

    index = atomicCounterIncrement(list_counter);
    old_head = imageAtomicExchange(head_pointer_image, ivec2(gl_FragCoord.xy), uint(index));

    vec3 material_final_ambient;
    vec3 material_final_diffuse;

    if (material.has_texture)
    {
        material_final_diffuse = texture(material.diffuse_texture, pass_texture_coords).rgb;
        material_final_ambient = material_final_diffuse * material.ambient;
    }
    else
    {
        material_final_diffuse = material.diffuse;
        material_final_ambient = material.ambient;
    }

    vec3 N = normalize(surface_normal);
    vec3 V = normalize(eye_position - surface_position);

    vec3 total_color = vec3(0, 0, 0);

    for (int i=0; i<8; i++)
    {
        if (directional_lights[i].use)
        {
            vec3 L = normalize(directional_lights[i].position);

            vec3 R = - L + 2.0 * dot(L, N) * N;

            float NdotL = dot(N, L);
            float VdotR = dot(V, R);

            float diffuse_strength = clamp(NdotL, 0, 1);
            float specular_strength = pow( clamp(VdotR, 0, 1), material.shininess );

            vec3 ambient = directional_lights[i].ambient * material_final_ambient;
            vec3 diffuse = directional_lights[i].diffuse * material_final_diffuse * diffuse_strength;
            vec3 specular = directional_lights[i].specular * material.specular * specular_strength;

            total_color += ambient + diffuse + specular;
        }
    }

    for (int i=0; i<8; i++)
    {
        if (point_lights[i].use)
        {
            vec3 L = point_lights[i].position - surface_position;
            float d = length(L);
            L /= d;

            vec3 R = - L + 2.0 * dot(L, N) * N;

            float NdotL = dot(N, L);
            float VdotR = dot(V, R);

            float diffuse_strength = clamp(NdotL, 0, 1);
            float specular_strength = pow( clamp(VdotR, 0, 1), material.shininess );

            vec3 ambient = point_lights[i].ambient * material_final_diffuse;
            vec3 diffuse = point_lights[i].diffuse * material_final_diffuse * diffuse_strength;
            vec3 specular = point_lights[i].specular * material.specular * specular_strength;

            float attenuation = 1.f / (point_lights[i].attenuation.x + point_lights[i].attenuation.y * d + point_lights[i].attenuation.z * d * d);

            total_color += (ambient + diffuse + specular) * attenuation;
        }
    }

    // OIT buffer
    vec4 modulator = vec4(total_color, material.alpha);

    item.x = old_head;
    item.y = packUnorm4x8(modulator);
    item.z = floatBitsToUint(gl_FragCoord.z);
    item.w = 0;

    imageStore(list_buffer, int(index), item);
}
