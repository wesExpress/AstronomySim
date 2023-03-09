#version 330 core

// vertex
layout (location = 0) in vec3 pos;
layout (location = 1) in vec2 tex_coords;
// instance
layout (location = 2)  in mat4 obj_model;
layout (location = 6) in vec4 obj_diffuse;

struct ps_input
{
	vec4  position;
	vec2  tex_coords;
	vec4  obj_diffuse;
	float depth;
};

layout (std140) uniform scene_uni
{
	mat4  view_proj;
	float fcoef_inv;
};

out ps_input vs_output;

void main()
{
	vs_output.position = view_proj * obj_model * vec4(pos, 1);

	vs_output.tex_coords = tex_coords;

	vs_output.obj_diffuse = obj_diffuse;

	vs_output.depth = log2(1.0f + vs_output.position.w) * fcoef_inv;

	gl_Position = vs_output.position;
}
