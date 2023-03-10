#version 330 core

// vertex
layout (location = 0) in vec3 pos;
layout (location = 2) in vec2 tex_coords;
// instance
layout (location = 3)  in mat4 obj_model;
layout (location = 11) in vec4 obj_diffuse;
layout (location = 12) in float brightness;

struct ps_input
{
	vec4  position;
	vec2  tex_coords;
	vec4  obj_color;
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
	vs_output.position =  view_proj * obj_model * vec4(pos, 1);
	vs_output.tex_coords = tex_coords;

	vs_output.obj_color = obj_diffuse * brightness;

	vs_output.depth = 1.0f + vs_output.position.w;

	gl_Position = vs_output.position;
}
