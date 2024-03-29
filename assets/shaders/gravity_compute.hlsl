struct transform_elem
{
	float3 pos;
	float  padding;
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

#include "../../app/simulation_defines.h"

groupshared float3 shared_pos[BLOCK_SIZE];
groupshared float shared_mass[BLOCK_SIZE];

[numthreads(BLOCK_SIZE,1,1)]
void c_main(uint3 group_id : SV_GroupID, uint3 dispatch_id : SV_DispatchThreadID, uint3 group_thread_id : SV_GroupThreadID, uint group_index : SV_GroupIndex)
{
	const uint dimx = uint(ceil((float)ARRAY_LENGTH / (float)BLOCK_SIZE));

	float3 r, f;
	float  distance_2, distance_6, inv_dis;
    float  grav;

	f.x = f.y = f.z = 0;

	const float3 p = transforms[dispatch_id.x].pos;
	const float  m = physics[dispatch_id.x].mass;

	[loop]
	for(uint tile=0; tile < dimx; tile++)
	{
		const uint id = tile * BLOCK_SIZE + group_index;

		shared_pos[group_index]  = transforms[id].pos;
		shared_mass[group_index] = physics[id].mass;

		GroupMemoryBarrierWithGroupSync();

		for(uint j=0; j<BLOCK_SIZE; j++)
		{
			r = shared_pos[j] - p;

			distance_2 = dot(r, r);

        	// softening
        	distance_2 += SOFTENING_2;

        	distance_6 = distance_2 * distance_2 * distance_2;
        	distance_6 = sqrt(distance_6);
        	inv_dis    = 1.f / distance_6;

        	grav  = shared_mass[j] * m;
        	grav *= G;
        	grav *= inv_dis;

			f += grav * r;   
		}
		GroupMemoryBarrierWithGroupSync();
	}

	physics[dispatch_id.x].force += f;
}