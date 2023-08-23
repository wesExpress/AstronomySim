struct PS_INPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEXCOORDS1;
	float4 color      : COLOR1;
};

SamplerState sample_state;
Texture2D tex : register(t0);

float4 p_main(PS_INPUT input) : SV_Target
{
	return input.color * tex.Sample(sample_state, input.tex_coords);
}