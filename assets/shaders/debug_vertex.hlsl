struct VS_INPUT
{
	float3 position : POSITION;
	matrix model    : OBJ_MODEL;
	float4 color    : COLOR0;
};

struct VS_OUTPUT
{
	float4 position : SV_Position;
	float4 color    : COLOR1;
};

cbuffer uni : register(b0)
{
	matrix view_proj;
};

VS_OUTPUT v_main(VS_INPUT input)
{
	VS_OUTPUT output = (VS_OUTPUT)0;

	output.position = mul(float4(input.position, 1.0f), input.model);
	output.position = mul(output.position, view_proj);

	output.color    = input.color;

	return output;
}