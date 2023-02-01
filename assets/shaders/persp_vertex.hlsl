struct VS_INPUT
{
	float3 position     : POSITION;
	float3 normal       : NORMAL0;
	float2 tex_coords   : TEXCOORD0;
	matrix obj_model    : OBJ_MODEL;      
	matrix obj_norm     : OBJ_NORM;
	float4 obj_diffuse  : OBJ_DIFFUSE0;
	float4 obj_specular : OBJ_SPECULAR0;
};

struct PS_INPUT
{
	float4 position     : SV_Position;
	float3 normal       : NORMAL1;
	float2 tex_coords   : TEXCOORD1;
	float3 frag_pos     : FRAG_POS;
	float4 obj_diffuse  : OBJ_DIFFUSE1;
	float4 obj_specular : OBJ_SPECULAR1;
	float  logz         : LOGZ;
};

cbuffer scene_cb : register(b0)
{
	matrix view_proj;
	float3 view_pos;
	float  fcoef_inv;
};

PS_INPUT v_main(VS_INPUT input)
{
	PS_INPUT output = (PS_INPUT)0;

	output.position = mul(float4(input.position, 1), input.obj_model);
	output.frag_pos = output.position.xyz;
	output.position = mul(output.position, view_proj);

	output.normal = mul(float4(input.normal, 1), input.obj_norm).xyz;

	output.tex_coords = input.tex_coords;

	output.obj_diffuse = input.obj_diffuse;
	output.obj_specular = input.obj_specular;

	output.logz = 1.0f + output.position.w;

	return output;
}