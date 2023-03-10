struct VS_INPUT
{
	float3 position     : POSITION;
	float2 tex_coords   : TEXCOORD0;
	matrix obj_model    : OBJ_MODEL;      
	float4 obj_diffuse  : OBJ_DIFFUSE0;
	float  brightness   : BRIGHTNESS0;
};

struct PS_INPUT
{
	float4 position     : SV_Position;
	float2 tex_coords   : TEXCOORD1;
	float4 obj_diffuse  : OBJ_DIFFUSE1;
	float  depth        : DEPTH;
	float  brightness   : BRIGHTNESS1;
};

cbuffer scene_cb : register(b0)
{
	matrix view_proj;
	float  fcoef_inv;
};

PS_INPUT v_main(VS_INPUT input)
{
	PS_INPUT output = (PS_INPUT)0;

	output.position = mul(float4(input.position, 1), input.obj_model);
	output.position = mul(output.position, view_proj);

	output.tex_coords = input.tex_coords;

	output.obj_diffuse = input.obj_diffuse;

	output.depth = 1.0f + output.position.w;
	output.brightness = input.brightness;

	return output;
}