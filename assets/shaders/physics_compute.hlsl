struct transform_elem
{
	float4 pos;
};

struct physics_elem
{
	float4 vel;
	float4 force;
};

RWStructuredBuffer<transform_elem> transforms : register(u0);
RWStructuredBuffer<physics_elem>   physics    : register(u1);

#define FIXED_DT   0.00833f
#define BLOCK_SIZE 128

[numthreads(BLOCK_SIZE,1,1)]
void c_main(uint3 id : SV_DispatchThreadID)
{
	const float dt_m = FIXED_DT / physics[id.x].vel.w;

	physics[id.x].vel.xyz    += physics[id.x].force.xyz * dt_m;
	transforms[id.x].pos.xyz += physics[id.x].vel.xyz * FIXED_DT;
}
