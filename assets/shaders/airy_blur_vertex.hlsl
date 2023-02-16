struct VS_INPUT
{
	float2 position   : POSITION;
	float2 tex_coords : TEXCOORDS0;
};

struct PS_INPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEXCOORDS1;
};

cbuffer lens_params : register(b0)
{
	float offset;
}

PS_INPUT v_main(VS_INPUT input)
{
	PS_INPUT output = (PS_INPUT)0;

	output.position = float4(input.position.x, input.position.y, 0, 1);
	output.tex_coords = input.tex_coords;

	return output;
}