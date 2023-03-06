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
	float4 object_diffuse;
	float4 object_specular;
};

struct scene_uniform
{
	float4x4 view_proj;
	float4   light_color;
	float4   light_ambient;
	float4   light_pos;
	float4   view_pos;
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
	v_out.object_diffuse = v_inst.object_diffuse;
	v_out.object_specular = v_inst.object_specular;

	return v_out;
}

fragment float4 fragment_main(vertex_out v_in [[stage_in]], texture2d<float> diffuse_map [[texture(0)]], texture2d<float> specular_map [[texture(1)]], constant scene_uniform& scene_uni [[buffer(0)]], sampler samplr [[sampler(0)]])
{
	float3 norm_normal = normalize(v_in.normal);
	float3 light_dir = normalize(scene_uni.light_pos.xyz - v_in.frag_pos);
	float3 view_dir = normalize(scene_uni.view_pos.xyz - v_in.frag_pos);
	float3 reflect_dir = reflect(-light_dir, norm_normal);

	float diff = max(dot(norm_normal, light_dir), 0.0f);
	float spec = pow(max(dot(view_dir, reflect_dir), 0.0f), 4);

	float4 ambient = scene_uni.light_ambient * diffuse_map.sample(samplr, v_in.tex_coords);
	float4 diffuse = scene_uni.light_color * diff * diffuse_map.sample(samplr, v_in.tex_coords);
	float4 specular = scene_uni.light_color * spec * specular_map.sample(samplr, v_in.tex_coords);

	ambient *= v_in.object_diffuse;
	diffuse *= v_in.object_diffuse;
	specular *= v_in.object_specular;

	return ambient + diffuse + specular;
}