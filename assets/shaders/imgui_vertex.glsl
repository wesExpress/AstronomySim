#version 460 core

layout(location=0) in vec3 position;
layout(location=1) in vec2 tex_coords;
layout(location=2) in vec4 color;

struct ps_input
{
	vec2 tex_coords;
	vec4 color;
};

layout(location=0) out ps_input vs_output;

layout (std140, binding=0) uniform uni
{
	mat4 proj;
};

void main()
{
	vs_output.tex_coords = tex_coords;

	vs_output.color = color;

	gl_Position = proj * vec4(position, 1);
}
