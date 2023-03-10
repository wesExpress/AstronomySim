#version 330 core

struct ps_input
{
	vec4  position;
	vec4  obj_color;
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
	FragColor = vs_output.obj_color;
	gl_FragDepth = log2(vs_output.depth) * fcoef_inv;
}
