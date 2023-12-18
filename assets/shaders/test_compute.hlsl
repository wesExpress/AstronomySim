struct INPUT
{
	float x;
	float y;
	float z;
};

struct OUTPUT
{
	float x;
	float y;
	float z;
};

cbuffer globals : register(b0)
{
	uint array_len;
};

RWStructuredBuffer<INPUT> input_array : register(t0);

[numthreads(8,8,1)]
void c_main(uint3 id : SV_DispatchTreadID)
{

}