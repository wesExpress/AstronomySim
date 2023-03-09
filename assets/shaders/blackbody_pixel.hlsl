struct PS_INPUT
{
	float4 position     : SV_Position;
	float2 tex_coords   : TEXCOORD1;
	float4 obj_diffuse  : OBJ_DIFFUSE1;
	float  depth        : DEPTH;
	float  brightness   : BRIGHTNESS1;
};

struct PS_OUTPUT
{
	float4 color : SV_Target;
	float  depth : SV_Depth;
};

PS_OUTPUT p_main(PS_INPUT input)
{
	PS_OUTPUT output = (PS_OUTPUT)0;

	output.color = input.obj_diffuse * input.brightness;
	output.depth = depth;

	return output;
}