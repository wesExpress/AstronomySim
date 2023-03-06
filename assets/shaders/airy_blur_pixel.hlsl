struct PS_INPUT
{
	float4 position  : SV_POSITION;
	float2 tex_coords : TEXCOORDS1;
};

cbuffer lens_params : register(b0)
{
	float offset;
}

SamplerState sample_state;
Texture2D    t;

float kernel[] = {
	1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f,
	2.0f / 16.0f, 4.0f / 16.0f, 2.0f / 16.0f,
	1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f
};

float4 p_main(PS_INPUT input) : SV_Target
{
	float3 color = float3(0,0,0);

	color += kernel[0] * t.Sample(sample_state, input.tex_coords + float2(-offset, offset)).xyz;
	color += kernel[1] * t.Sample(sample_state, input.tex_coords + float2(0, offset)).xyz;
	color += kernel[2] * t.Sample(sample_state, input.tex_coords + float2(offset, offset)).xyz;
	color += kernel[3] * t.Sample(sample_state, input.tex_coords + float2(-offset, 0)).xyz;
	color += kernel[4] * t.Sample(sample_state, input.tex_coords + float2(0, 0)).xyz;
	color += kernel[5] * t.Sample(sample_state, input.tex_coords + float2(offset, 0)).xyz;
	color += kernel[6] * t.Sample(sample_state, input.tex_coords + float2(-offset, -offset)).xyz;
	color += kernel[7] * t.Sample(sample_state, input.tex_coords + float2(0, -offset)).xyz;
	color += kernel[8] * t.Sample(sample_state, input.tex_coords + float2(offset, -offset)).xyz;

	//return t.Sample(sample_state, input.tex_coords);
	return float4(color, 1);
}