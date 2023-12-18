#version 460 core

struct ps_input
{
	vec4 position;
	vec2 tex_coords;
	vec3 normal;
	vec4 color;
	vec3 frag_pos;
};

layout (std140, binding=1) uniform uni
{
	mat4 view_proj;
	vec3 view_pos;
};

layout(location=0) in ps_input vs_output;
out vec4 frag_color;

uniform sampler2D default_texture;

const vec3 light_pos = { 15,2,15 };
const vec4 light_ambient = { 0.33f,0.33f,0.33f,1.0f };
const vec4 light_diffuse = { 1,1,1,1 };
const vec4 light_specular = { 1,1,1,1 };

void main()
{
	const vec3 norm_normal = normalize(vs_output.normal);
	const vec3 view_dir = normalize(view_pos - vs_output.frag_pos);
	const vec3 light_dir = normalize(light_pos - vs_output.frag_pos);
	const vec3 reflect_dir = reflect(-light_dir, norm_normal);

	const float diff = max(dot(vs_output.normal, light_dir), 0.0f);
	const float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 64);

	vec4 obj_color = vs_output.color * texture(default_texture, vs_output.tex_coords);

	const float distance = length(light_pos - vs_output.frag_pos);
	const float atten = 1.0f / (1.0f + 0.01f * distance + 0.0001f * distance * distance);

	vec4 ambient  = vec4(vs_output.color.xyz * light_ambient.xyz * atten, 1);
	vec4 diffuse  = vec4(vs_output.color.xyz * diff * light_diffuse.xyz * atten, 1);
	vec4 specular = vec4(vs_output.color.xyz * spec * light_specular.xyz * atten, 1);

	frag_color = (ambient + diffuse + specular) * obj_color;
}
