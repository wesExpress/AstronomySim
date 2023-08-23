struct PS_INPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEXCOORD1;
	float4 color      : COLOR1;
};

cbuffer scene_cb : register(b0)
{
	matrix proj;
};

SamplerState sample_state;
Texture2D font_texture : register(t0);

float4 p_main(PS_INPUT input) : SV_Target
{
	float4 tex_color = font_texture.Sample(sample_state, input.tex_coords);
	return input.color * tex_color;
}