#include <metal_stdlib>

using namespace metal;

struct vertex_in
{
    packed_float2 position;
    packed_float2 tex_coords;
};

struct vertex_out
{
    float4 position [[position]];
    float2 tex_coords;
};

struct fragment_out
{
    float4 color [[color(0)]];
};

vertex vertex_out vertex_main(
    const device vertex_in* vertices [[buffer(0)]],
    uint vid [[vertex_id]]) 
{
    vertex_out v_out;

    v_out.position = float4(vertices[vid].position, 0,1);
    v_out.tex_coords = vertices[vid].tex_coords;
    //v_out.tex_coords.y = 1 - v_out.tex_coords.y;

    return v_out;
}

fragment fragment_out fragment_main(
    vertex_out v_in [[stage_in]], 
    texture2d<float> texture [[texture(0)]], 
    sampler samplr [[sampler(0)]])
{
    fragment_out out;

    out.color = texture.sample(samplr, v_in.tex_coords);

    return out;
}