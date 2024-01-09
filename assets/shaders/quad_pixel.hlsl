struct INPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEX_COORDS1;
};

SamplerState sample_state;
Texture2D    tex : register(t0);

float4 p_main(INPUT input) : SV_Target
{
	//return float4(1,0,1,1);
	return tex.Sample(sample_state, input.tex_coords);
}