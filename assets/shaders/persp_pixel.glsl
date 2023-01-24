#version 330 core

struct ps_input
{
	vec4 position;
	vec3 normal;
	vec2 tex_coords;
	vec3 frag_pos;
	vec4 obj_diffuse;
	vec4 obj_specular;
};

layout (std140) uniform scene_uni
{
	mat4  view_proj;
	vec4  light_color;
	vec4  ambient_color;
	vec3  light_pos;
	float padding;
	vec3  view_pos;
};

uniform sampler2D default_texture;

in ps_input vs_output;
out vec4 color;

void main()
{
	vec3 norm_normal = normalize(vs_output.normal);
	vec3 light_dir = normalize(light_pos - vs_output.frag_pos);
	vec3 view_dir = normalize(view_pos - vs_output.frag_pos);
	vec3 reflect_dir = reflect(-light_dir, norm_normal);

	float diff = max(dot(norm_normal, light_dir), 0);
	float spec = pow(max(dot(view_dir, reflect_dir), 0), 36);
	
	vec4 texture_color = texture(default_texture, vs_output.tex_coords);

	vec4 diffuse = diff * light_color * vs_output.obj_diffuse * texture_color;
	vec4 specular = spec * light_color * vs_output.obj_specular * texture_color;

	color = (ambient_color * vs_output.obj_diffuse * texture_color + diffuse + specular);
}
