StructuredBuffer<float> in_a : register(t0);
StructuredBuffer<float> in_b : register(t1);

RWStructuredBuffer<float> buffer_out : register(u0);

[numthreads(8,1,1)]
void c_main(uint3 id : SV_DispatchThreadID)
{
	buffer_out[id.x] = in_a[id.x] + in_b[id.x];
}