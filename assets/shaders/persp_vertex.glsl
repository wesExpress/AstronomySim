#version 330 core

// vertex
layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 tex_coords;
// instance
layout (location = 3)  in mat4 obj_model;
layout (location = 7)  in mat4 obj_norm;
layout (location = 11) in vec4 obj_diffuse;
layout (location = 12) in vec4 obj_specular;

struct ps_input
{
	vec4  position;
	vec3  normal;
	vec2  tex_coords;
	vec3  frag_pos;
	vec4  obj_diffuse;
	vec4  obj_specular;
	float depth;
};

layout (std140) uniform scene_uni
{
	mat4  view_proj;
	vec3  view_pos;
	float fcoef_inv;
};

out ps_input vs_output;

void main()
{
	vs_output.position = obj_model * vec4(pos, 1);
	vs_output.frag_pos = vs_output.position.xyz;
	vs_output.position = view_proj * vs_output.position;

	vs_output.normal = (obj_norm * vec4(normal, 0)).xyz;
	
	vs_output.tex_coords = tex_coords;

	vs_output.obj_diffuse = obj_diffuse;
	vs_output.obj_specular = obj_specular;

	vs_output.depth = 1.0f + vs_output.position.w;

	gl_Position = vs_output.position;
}
