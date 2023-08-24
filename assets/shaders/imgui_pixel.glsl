#version 460 core

struct ps_input
{
	vec2 tex_coords;
	vec4 color;
};

layout(location=0) in ps_input vs_output;
out vec4 frag_color;

uniform sampler2D font_texture;

void main()
{
	frag_color = vs_output.color * texture(font_texture, vs_output.tex_coords);
}
