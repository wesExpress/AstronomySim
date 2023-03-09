#include <metal_stdlib>

using namespace metal;

struct vertex_in
{
	packed_float3 position;
	packed_float2 tex_coords;
};

struct vertex_inst
{
	float4x4 obj_model;
	float4   object_diffuse;
};

struct vertex_out
{
	float4 position [[position]];
	float2 tex_coords;
	float4 obj_diffuse;
	float  depth;
};

struct fragment_out
{
	float4 color [[color(0)]];
	float  depth [[depth(any)]];
};

struct scene_uniform
{
	float4x4 view_proj;
	float    fcoef_inv;
};

vertex vertex_out vertex_main(const device vertex_in* vertices [[buffer(0)]], const device vertex_inst* instance_data [[buffer(1)]], constant scene_uniform& scene_uni [[buffer(2)]], uint vid [[vertex_id]], uint instid [[instance_id]])
{
	vertex_out v_out;
	vertex_in v_in = vertices[vid];
	vertex_inst v_inst = instance_data[instid];

	v_out.position = v_inst.obj_model * float4(v_in.position, 1);
	v_out.position = scene_uni.view_proj * v_out.position;
	
	v_out.tex_coords = v_in.tex_coords;
	v_out.obj_diffuse = v_inst.object_diffuse;

	v_out.depth = log2(1.0f + v_out.position.w) * scene_uni.fcoef_inv;

	return v_out;
}

fragment fragment_out fragment_main(vertex_out v_in [[stage_in]])
{
	fragment_out out;

	out.color = v_in.obj_diffuse;
	out.depth = v_in.depth;

	return out;
}