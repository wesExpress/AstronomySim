#include <metal_stdlib>

using namespace metal;

struct vertex_in
{
	packed_float3 position;
	packed_float3 normal;
	packed_float2 tex_coords;
};

struct vertex_inst
{
	float4x4      obj_model;
	float4x4      obj_normal;
	float4        object_diffuse;
	float4        object_specular;
};

struct vertex_out
{
	float4 position [[position]];
	float3 normal;
	float2 tex_coords;
	float3 frag_pos;
	float4 obj_diffuse;
	float4 obj_specular;
	float  depth;
};

struct fragment_out
{
	float4 color [[color(0)]];
	float  depth [[depth(any)]];
};

struct scene_uniform
{
	float4x4      view_proj;
	packed_float3 view_pos;
	float         fcoef_inv;
};

vertex vertex_out vertex_main(const device vertex_in* vertices [[buffer(0)]], const device vertex_inst* instance_data [[buffer(1)]], constant scene_uniform& scene_uni [[buffer(2)]], uint vid [[vertex_id]], uint instid [[instance_id]])
{
	vertex_out v_out;
	vertex_in v_in = vertices[vid];
	vertex_inst v_inst = instance_data[instid];

	v_out.position = v_inst.obj_model * float4(v_in.position, 1);
	v_out.frag_pos = v_out.position.xyz;
	v_out.position = scene_uni.view_proj * v_out.position;
	v_out.normal = (v_inst.obj_normal * float4(v_in.normal, 1)).xyz;
	
	v_out.tex_coords = v_in.tex_coords;
	v_out.obj_diffuse = v_inst.object_diffuse;
	v_out.obj_specular = v_inst.object_specular;

	v_out.depth = log2(1.0f + v_out.position.w) * scene_uni.fcoef_inv;

	return v_out;
}

#define MAX_LIGHTS 10
struct point_light
{
	float4 position;

	float4 ambient;
	float4 diffuse;
	float4 specular;

	float4 params;
};

struct blackbody
{
	float4 position;
	float4 color;
	float  brightness;
	packed_float3 padding;
};

struct lights_uniform
{
	point_light point_lights[MAX_LIGHTS];
	blackbody   blackbodies[MAX_LIGHTS];

	uint        num_point_lights;
	uint        num_blackbodies;
};

#define INV_4PI 0.0795774715459f

float3 calc_point_light(point_light light, float3 normal, float3 frag_pos, float3 view_dir, float3 diffuse_color, float3 specular_color)
{
	float3 light_pos   = light.position.xyz;
	float3 light_dir   = normalize(light_pos - frag_pos);
	float3 reflect_dir = reflect(-light_dir, normal);

	float diff = max(dot(normal, light_dir), 0.0f);
	float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 4);

	float distance    = length(light_pos - frag_pos);
	float attenuation = 1.0f / (light.params.x + light.params.y * distance + light.params.z * distance * distance);

	float3 ambient  = light.ambient.xyz  * diffuse_color;
	float3 diffuse  = light.diffuse.xyz  * diff * diffuse_color;
	float3 specular = light.specular.xyz * spec * specular_color;

	return (ambient + diffuse + specular) * attenuation;
}

float3 calc_blackbody_light(blackbody bb, float3 normal, float3 frag_pos, float3 view_dir, float3 diffuse_color, float3 specular_color)
{
	float3 light_pos   = bb.position.xyz;
	float3 light_dir   = normalize(light_pos - frag_pos);
	float3 reflect_dir = reflect(-light_dir, normal);

	float diff = max(dot(normal, light_dir), 0.0f);
	float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 4);

	float3 diffuse  = bb.color.xyz * diff * diffuse_color;
	float3 specular = bb.color.xyz * spec * specular_color;

	return (diffuse + specular) * bb.brightness / 2048.0f;
}

fragment fragment_out fragment_main(vertex_out v_in [[stage_in]], texture2d<float> obj_texture [[texture(0)]], constant scene_uniform& scene_uni [[buffer(0)]], constant lights_uniform& lights_uni [[buffer(1)]], sampler samplr [[sampler(0)]])
{
	fragment_out out;

	float3 norm_normal = normalize(v_in.normal);
	float3 view_dir = normalize(scene_uni.view_pos.xyz - v_in.frag_pos);

	out.color = 0;

	for(uint i=0; i<lights_uni.num_point_lights; i++)
	{
		float3 color = calc_point_light(lights_uni.point_lights[i], norm_normal, v_in.frag_pos, view_dir, v_in.obj_diffuse.xyz, v_in.obj_specular.xyz);
		out.color += float4(color, 1);
	}

	for(uint j=0; j<lights_uni.num_blackbodies; j++)
	{
		float3 color = calc_blackbody_light(lights_uni.blackbodies[j], norm_normal, v_in.frag_pos, view_dir, v_in.obj_diffuse.xyz, v_in.obj_specular.xyz);
		out.color += float4(color, 1);
	}

	out.color *= obj_texture.sample(samplr, v_in.tex_coords);
	out.depth = v_in.depth;

	return out;
}