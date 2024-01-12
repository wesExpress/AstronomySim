#include <metal_stdlib>
using namespace metal;

struct vertex_in
{
    packed_float3 position;
    packed_float2 tex_coords;
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

struct instance
{
    float4x4 model;
    float4   color;
};

struct uniform_data
{
    float4x4 view_proj;
};

vertex vertex_out vertex_main(
	const device vertex_in* vertices [[buffer(0)]], 
    const device instance* instances [[buffer(1)]], 
    constant uniform_data& uni       [[buffer(2)]],
    uint vid [[vertex_id]],
    uint instid [[instance_id]])
{
	vertex_out v_out;
    vertex_in  v_in   = vertices[vid];
    instance   v_inst = instances[instid];

	v_out.position = v_inst.model * float4(v_in.position, 1);
    v_out.position = uni.view_proj * v_out.position;

	v_out.tex_coords = v_in.tex_coords;
    v_out.color = v_inst.color;

	return v_out;
}

fragment fragment_out fragment_main(
    vertex_out v_in [[stage_in]],
    texture2d<float> obj_texture [[texture(0)]],  
    sampler samplr [[sampler(0)]])
{
	fragment_out out;

	out.color = v_in.color * obj_texture.sample(samplr, v_in.tex_coords);

	return out;
}