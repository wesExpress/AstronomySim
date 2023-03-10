struct PS_INPUT
{
	float4 position   : SV_Position;
	float4 obj_color  : OBJ_COLOR1;
	float  depth      : DEPTH;
	float  brightness : BRIGHTNESS1;
};

struct PS_OUTPUT
{
	float4 color : SV_Target;
	float  depth : SV_Depth;
};

cbuffer scene_cb : register(b0)
{
	matrix view_proj;
	float  fcoef_inv;
};

PS_OUTPUT p_main(PS_INPUT input)
{
	PS_OUTPUT output = (PS_OUTPUT)0;

	output.color = input.obj_color * input.brightness;
	output.depth = log2(input.depth) * fcoef_inv;

	return output;
}