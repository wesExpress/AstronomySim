#version 460 core

layout(location=0)  in vec3 position;
layout(location=1)  in vec2 tex_coords;
layout(location=2)  in vec3 normal;
layout(location=3)  in mat4 obj_model;
layout(location=7)  in mat4 obj_normal;
layout(location=11) in vec4 color;

struct ps_input
{
	vec4 position;
	vec2 tex_coords;
	vec3 normal;
	vec4 color;
	vec3 frag_pos;
};

layout(location=0) out ps_input vs_output;

layout (std140, binding=1) uniform uni
{
	mat4 view_proj;
	vec3 view_pos;
};

void main()
{
	vs_output.position = obj_model * vec4(position, 1);
	vs_output.frag_pos = vs_output.position.xyz;
	vs_output.position = view_proj * vs_output.position;

	vs_output.tex_coords = tex_coords;

	vs_output.normal = (obj_normal * vec4(normal, 0)).xyz;

	vs_output.color = color;

	gl_Position = vs_output.position;
}
