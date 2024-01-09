#version 460 core

struct ps_input
{
    vec4 position;
    vec2 tex_coords;
};

uniform sampler2D image_texture;

in ps_input vs_output;
out vec4 frag_color;

void main()
{
    frag_color = texture(image_texture, vs_output.tex_coords);
}
