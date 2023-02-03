#version 330 core

struct ps_input
{
	vec4  position;
	vec3  normal;
	vec2  tex_coords;
	vec3  frag_pos;
	vec4  obj_diffuse;
	vec4  obj_specular;
	float logz;
};

#define MAX_LIGHTS 5
struct point_light
{
	vec4 position;

	vec4 ambient;
	vec4 diffuse;
	vec4 specular;

	float constant;
	float lnear;
	float quadratic;
	float padding;
};

layout (std140) uniform scene_uni
{
	mat4  view_proj;
	vec3  view_pos;
	float fcoef_inv;
};

layout (std140) uniform lights_uni
{
	point_light point_lights[MAX_LIGHTS];
	uint        num_point_lights;
};

uniform sampler2D default_texture;

in ps_input vs_output;
out vec4 color;

vec3 calc_point_light(point_light light, vec3 normal, vec3 frag_pos, vec3 view_dir, vec3 diffuse_color, vec3 specular_color, vec3 texture_color)
{
	vec3 light_pos = light.position.xyz;
	vec3 light_dir = normalize(light_pos - frag_pos);
	vec3 reflect_dir = reflect(-light_dir, normal);

	float diff = max(dot(normal, light_dir), 0.0f);
	float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 4);

	float distance    = length(light_pos - frag_pos);
	float attenuation = 1.0f / (light.constant + light.lnear * distance + light.quadratic * distance * distance);

	vec3 ambient  = light.ambient.xyz * diffuse_color;
	vec3 diffuse  = light.diffuse.xyz * diff * diffuse_color;
	vec3 specular = light.specular.xyz * spec * specular_color;

	return (ambient + diffuse + specular) * texture_color * attenuation;
}

void main()
{
	vec3 norm_normal = normalize(vs_output.normal);
	vec3 view_dir = normalize(view_pos - vs_output.frag_pos);

	vec4 texture_color = texture(default_texture, vs_output.tex_coords);

	color = vec4(0);
	for(uint i=0u; i<num_point_lights; i++)
	{
		vec3 point_color = calc_point_light(point_lights[i], norm_normal, vs_output.frag_pos, view_dir, texture_color.xyz, vs_output.obj_diffuse.xyz, vs_output.obj_specular.xyz);
		color += vec4(point_color, 1);
	}

	gl_FragDepth = log2(vs_output.logz) * fcoef_inv;
}
