#include <metal_stdlib>

using namespace metal;

struct vertex_in
{
	packed_float3 position;
};

struct vertex_out
{
	float4 position [[position]];
};

struct fragment_out
{
	float4 color [[color(0)]];
};

vertex vertex_out vertex_main(const device vertex_in* vertices [[buffer(0)]], uint vid [[vertex_id]])
{
	vertex_out v_out;

	v_out.position = float4(vertices[vid].position, 1);

	return v_out;
}

fragment fragment_out fragment_main(vertex_out v_in [[stage_in]])
{
	fragment_out out;

	out.color = float4(1,1,1,1);

	return out;
}