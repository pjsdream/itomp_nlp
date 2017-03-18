#version 430 core

/*
 * OpenGL Programming Guide - Order Independent Transparency Example
 *
 * This is the resolve shader for order independent transparency.
 */

// The per-pixel image containing the head pointers
layout (binding = 0, r32ui) uniform uimage2D head_pointer_image;
// Buffer containing linked lists of fragments
layout (binding = 1, rgba32ui) uniform uimageBuffer list_buffer;

// This is the output color
layout (location = 0) out vec4 color;

// This is the maximum number of overlapping fragments allowed
#define MAX_FRAGMENTS 40

// Temporary array used for sorting fragments
uvec4 fragment_list[MAX_FRAGMENTS];

// opaque textures
uniform sampler2D opaque_color;
uniform sampler2D opaque_depth;
//layout (binding = 2, rgba8) uniform image2D opaque_color;
//layout (binding = 3, r32ui) uniform image2D opaque_depth;

void main(void)
{
    uint current_index;
    uint fragment_count = 0;

    // opaque fragment from texture
    uvec4 opaque_fragment;
    opaque_fragment.y = packUnorm4x8(vec4(texture(opaque_color, vec2(gl_FragCoord.x / 800, gl_FragCoord.y / 600)).xyz, 1.f));
    opaque_fragment.z = floatBitsToUint(texture(opaque_depth, vec2(gl_FragCoord.x / 800, gl_FragCoord.y / 600)).r);
    //opaque_fragment.y = packUnorm4x8(vec4(imageLoad(opaque_color, ivec2(gl_FragCoord).xy).rgb, 1.f));
    //opaque_fragment.z = floatBitsToUint(imageLoad(opaque_depth, ivec2(gl_FragCoord).xy).r);
    fragment_list[fragment_count] = opaque_fragment;
    fragment_count++;

    current_index = imageLoad(head_pointer_image, ivec2(gl_FragCoord).xy).x;

    while (current_index != 0 && fragment_count < MAX_FRAGMENTS)
    {
        uvec4 fragment = imageLoad(list_buffer, int(current_index));
        fragment_list[fragment_count] = fragment;
        current_index = fragment.x;
        fragment_count++;
    }

    uint i, j;

    if (fragment_count > 1)
    {
        for (i = 0; i < fragment_count - 1; i++)
        {
            for (j = i + 1; j < fragment_count; j++)
            {
                uvec4 fragment1 = fragment_list[i];
                uvec4 fragment2 = fragment_list[j];

                float depth1 = uintBitsToFloat(fragment1.z);
                float depth2 = uintBitsToFloat(fragment2.z);

                if (depth1 < depth2)
                {
                    fragment_list[i] = fragment2;
                    fragment_list[j] = fragment1;
                }
            }
        }

    }


    vec4 final_color = vec4(1.f);

    for (i = 0; i < fragment_count; i++)
    {
        vec4 modulator = unpackUnorm4x8(fragment_list[i].y);
        vec4 additive_component = unpackUnorm4x8(fragment_list[i].w);

        final_color = mix(final_color, modulator, modulator.a) + additive_component;
    }

    color = final_color;
    //color = vec4(float(fragment_count) / float(MAX_FRAGMENTS));
    //color = vec4(vec3(texture(opaque_color, vec2(gl_FragCoord.x / 800, gl_FragCoord.y / 600)).r), 1.f);
    //color = vec4(vec3(texture(opaque_depth, vec2(gl_FragCoord.x / 800, gl_FragCoord.y / 600)).r), 1.f);


/*
    vec4 final_color = vec4(1.0);
    float last_alpha = 1.f;

    for (i = 0; i < fragment_count; i++)
    {
        vec4 modulator = unpackUnorm4x8(fragment_list[i].y);
        vec4 additive_component = unpackUnorm4x8(fragment_list[i].w);

        final_color = mix(final_color, modulator, modulator.a);// + additive_component;
        last_alpha = modulator.a;
    }

    color = vec4(last_alpha, 0, 0, 1.f);
*/
}
