struct INPUT
{
	float3 pos   : POSITION;
	float4 color : COLOR0;
};

struct OUTPUT
{
	float4 position : SV_POSITION;
	float4 color    : COLOR1;
};

OUTPUT v_main(INPUT input)
{
	OUTPUT output = (OUTPUT)0;

	output.position = float4(input.pos, 1);
	output.color    = input.color;

	return output;
}