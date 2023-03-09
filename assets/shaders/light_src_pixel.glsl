#version 330 core

struct ps_input
{
	vec4  position;
	vec2  tex_coords;
	vec4  obj_diffuse;
	float depth;
};

in ps_input vs_output;
out vec4 FragColor;

void main()
{
	FragColor = vs_output.obj_diffuse;
	gl_FragDepth = vs_output.depth;
}
