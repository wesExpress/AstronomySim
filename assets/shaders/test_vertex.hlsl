struct VS_INPUT
{
	float3 position   : POSITION;
	float2 tex_coords : TEXCOORDS0;
	float3 normal     : NORMAL0;
	matrix obj_model  : OBJ_MODEL;
	matrix obj_norm   : OBJ_NORM;
	float4 color      : COLOR0;
};

struct PS_INPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEXCOORDS1;
	float3 normal     : NORMAL1;
	float4 color      : COLOR1;
	float3 frag_pos   : FRAG_POS0;
};

cbuffer uni : register(b0)
{
	matrix view_proj;
	float3 view_pos;
};

PS_INPUT v_main(VS_INPUT input)
{
	PS_INPUT output = (PS_INPUT)0;
	
	output.position = mul(float4(input.position, 1), input.obj_model);
	output.frag_pos  = output.position.xyz;
	output.position = mul(output.position, view_proj);

	output.tex_coords = input.tex_coords;
	
	output.normal = mul(float4(input.normal, 0), input.obj_norm).xyz;

	output.color = input.color;

	return output;
}