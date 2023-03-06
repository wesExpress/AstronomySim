struct VS_INPUT
{
	float3 position   : POSITION;
	float4 obj_color  : OBJ_COLOR0;
	float  brightness : BRIGHTNESS0;
};

struct PS_INPUT
{
	float4 position   : SV_Position;
	float4 obj_color  : OBJ_COLOR1;
	float  logz       : LOGZ;
	float  brightness : BRIGHTNESS1;
};

cbuffer scene_cb : register(b0)
{
	matrix view_proj;
	float  fcoef_inv;
};

PS_INPUT v_main(VS_INPUT input)
{
	PS_INPUT output = (PS_INPUT)0;

	output.position = mul(float4(input.position, 1), view_proj);

	output.obj_color = input.obj_color;

	output.logz = 1.0f + output.position.w;
	output.brightness = input.brightness;

	return output;
}