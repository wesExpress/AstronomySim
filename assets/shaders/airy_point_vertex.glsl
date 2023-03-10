#version 330 core

// vertex
layout (location = 0) in vec3 pos;
layout (location = 2) in vec4 obj_color;
layout (location = 3) in float brightness;

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

out ps_input vs_output;

void main()
{
	vs_output.position =  view_proj * vec4(pos, 1);

	vs_output.obj_color = obj_color * brightness;

	vs_output.depth = 1.0f + vs_output.position.w;

	gl_Position = vs_output.position;
}
