#version 430 core

layout (early_fragment_tests) in;

layout (binding = 0, r32ui) uniform uimage2D head_pointer_image;
layout (binding = 1, rgba32ui) uniform writeonly uimageBuffer list_buffer;

layout (binding = 0, offset = 0) uniform atomic_uint list_counter;

layout (location = 0) out vec4 color;

in vec3 frag_position;
in vec3 frag_normal;
in vec4 surface_color;

uniform vec3 eye_position;

const int num_lights = 4;
const vec3 light_position[num_lights] =
{
	vec3(-10,   0, 10),
	vec3(  0, -10, 10),
	vec3( 10,   0, 10),
	vec3(  0,  10, 10)
};

const float ambient_constant = 0.1;
const float diffuse_constant = 0.5;
const float spec_constant = 0.1;
const float shininess = 0.;

void main(void)
{
    uint index;
    uint old_head;
    uvec4 item;
    vec4 frag_color;

    index = atomicCounterIncrement(list_counter);

    old_head = imageAtomicExchange(head_pointer_image, ivec2(gl_FragCoord.xy), uint(index));

	vec3 V = normalize(eye_position - frag_position);
	vec3 N = normalize(frag_normal);

	vec3 ambient_color = surface_color.rgb * ambient_constant;
	vec3 diffuse_color = surface_color.rgb * diffuse_constant;
	const vec3 spec_color = vec3(1.0);

	vec4 modulator = vec4(ambient_color.xyz, surface_color.a);

	for (int i=0; i<num_lights; i++)
	{
		vec3 L = normalize(light_position[i] - frag_position); // point source
		// vec3 L = normalize(light_position[i]); // parallel source
		vec3 H = normalize(L + V);

		float NdotL = dot(N, L);
		float NdotH = dot(N, H);

		if (NdotL > 0.)
		{
			float spec = pow(max(NdotH, 0.), shininess) * spec_constant;
			modulator.rgb += NdotL * diffuse_color + spec * spec_color;
		}
	}

    item.x = old_head;
    item.y = packUnorm4x8(modulator);
    item.z = floatBitsToUint(gl_FragCoord.z);
    item.w = 0;

    imageStore(list_buffer, int(index), item);

    frag_color = modulator;

    color = frag_color;
}
