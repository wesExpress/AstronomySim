struct INPUT
{
	float2 pos        : POSITION;
	float2 tex_coords : TEX_COORDS0;
};

struct OUTPUT
{
	float4 position   : SV_Position;
	float2 tex_coords : TEX_COORDS1;
};

OUTPUT v_main(INPUT input)
{
	OUTPUT output = (OUTPUT)0;

	output.position   = float4(input.pos, 0, 1);
	output.tex_coords = input.tex_coords;

	return output;
}