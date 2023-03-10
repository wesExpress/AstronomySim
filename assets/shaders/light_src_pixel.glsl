#version 330 core

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

in ps_input vs_output;
out vec4 FragColor;

void main()
{
	FragColor = vs_output.obj_diffuse;
	gl_FragDepth = log2(vs_output.depth) * fcoef_inv;
}
