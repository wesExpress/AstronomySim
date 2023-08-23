struct VS_INPUT
{
	float3 position   : POSITION;
	float2 tex_coords : TEXCOORD0;
	float4 color      : COLOR0;
};

struct PS_INPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEXCOORD1;
	float4 color      : COLOR1;
};

cbuffer uni : register(b0)
{
	matrix proj;
}

PS_INPUT v_main(VS_INPUT input)
{
	PS_INPUT output = (PS_INPUT)0;
	
	output.position   = mul(float4(input.position, 1), proj);
	output.color      = input.color;
	output.tex_coords = input.tex_coords;

	return output;
}