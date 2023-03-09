#version 330 core

struct ps_input
{
	vec4  position;
	vec4  obj_color;
	float depth;
};

in ps_input vs_output;
out vec4 FragColor;

void main()
{
	FragColor = vs_output.obj_color;
	gl_FragDepth = vs_output.depth;
}
