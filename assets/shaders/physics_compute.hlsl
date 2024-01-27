struct transform_elem
{
	float3 pos;
	float padding;
};

struct physics_elem
{
	float3 vel;
	float  mass;

	float3 force;
	float  inv_mass;

	float3 accel;
	float  padding;
};

RWStructuredBuffer<transform_elem> transforms : register(u0);
RWStructuredBuffer<physics_elem>   physics    : register(u1);

#define ARRAY_LENGTH 20000

#define FIXED_DT     0.00833f
#define BLOCK_SIZE   256

[numthreads(BLOCK_SIZE,1,1)]
void c_main(uint3 id : SV_DispatchThreadID)
{
	const float dt_m_half = FIXED_DT * physics[id.x].inv_mass * 0.5f;

	physics[id.x].vel    += physics[id.x].accel * FIXED_DT * 0.5f;
	transforms[id.x].pos += physics[id.x].vel * FIXED_DT;
	
	physics[id.x].accel   = physics[id.x].force * physics[id.x].inv_mass;
	physics[id.x].vel    += physics[id.x].accel * FIXED_DT * 0.5f;
}
