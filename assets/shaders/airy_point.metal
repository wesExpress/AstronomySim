#include <metal_stdlib>

using namespace metal;

struct vertex_in
{
	packed_float3 position;
	packed_float4 obj_color;
	float  brightness;
};

struct vertex_inst
{
	float brightness;
};

struct vertex_out
{
	float4 position [[position]];
	float4 obj_color;
	float  logz;
	float  brightness;
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

vertex vertex_out vertex_main(const device vertex_in* vertices [[buffer(0)]], constant scene_uniform& scene_uni [[buffer(2)]], uint vid [[vertex_id]])
{
	vertex_out v_out;
	vertex_in v_in = vertices[vid];

	v_out.position = scene_uni.view_proj * float4(v_in.position, 1);;
	
	v_out.obj_color = v_in.obj_color;

	v_out.logz = 1.0f + v_out.position.w;
	v_out.brightness = v_in.brightness;

	return v_out;
}

fragment fragment_out fragment_main(vertex_out v_in [[stage_in]], texture2d<float> obj_texture [[texture(0)]], constant scene_uniform& scene_uni [[buffer(0)]], sampler samplr [[sampler(0)]])
{
	fragment_out out;

	out.color = v_in.obj_color * v_in.brightness;
	out.depth = log2(v_in.logz) * scene_uni.fcoef_inv;

	return out;
}