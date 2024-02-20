struct INPUT
{
	float4 position : SV_Position;
	float4 color    : COLOR1;
};

float4 p_main(INPUT input) : SV_Target
{
	return input.color;
}