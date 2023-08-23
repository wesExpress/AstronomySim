struct VS_INPUT
{
	float3 position   : POSITION;
	float2 tex_coords : TEXCOORDS0;
	matrix model      : MODEL;
	float4 color      : COLOR0;
};

struct PS_INPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEXCOORDS1;
	float4 color      : COLOR1;
};

cbuffer uni : register(b0)
{
	matrix view_proj;
};

PS_INPUT v_main(VS_INPUT input)
{
	PS_INPUT output = (PS_INPUT)0;
	
	output.position = mul(float4(input.position, 1), input.model);
	output.position = mul(output.position, view_proj);

	output.tex_coords = input.tex_coords;

	output.color = input.color;

	return output;
}