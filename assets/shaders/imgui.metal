#include <metal_stdlib>

using namespace metal;

struct vertex_in
{
	packed_float3 position;
	packed_float2 tex_coords;
    packed_float4 color;
};

struct vertex_out
{
	float4 position [[position]];
	float2 tex_coords;
	float4 color;
};

struct fragment_out
{
	float4 color [[color(0)]];
};

struct proj_uniform
{
    float4x4 proj;
};

vertex vertex_out vertex_main(
	const device vertex_in* vertices [[buffer(0)]], 
    constant proj_uniform& uni [[buffer(1)]],
    uint vid [[vertex_id]])
{
	vertex_out v_out;
    vertex_in v_in = vertices[vid];

	v_out.position = float4(v_in.position, 1);
    v_out.position = uni.proj * v_out.position;

	v_out.tex_coords = v_in.tex_coords;
    v_out.color = v_in.color;

	return v_out;
}

fragment fragment_out fragment_main(vertex_out v_in [[stage_in]], texture2d<float> font_texture [[texture(0)]],  sampler samplr [[sampler(0)]])
{
	fragment_out out;

	out.color = v_in.color * font_texture.sample(samplr, v_in.tex_coords);

	return out;
}