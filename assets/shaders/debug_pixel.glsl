#version 460 core

struct ps_input
{
	vec4 position;
	vec4 color;
};

layout(location=0) in ps_input vs_output;
out vec4 frag_color;

void main()
{
	frag_color = vs_output.color;
}
