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

#define OBJECT_COUNT 10000
#define BLOCK_SIZE   128

#define G            6.67e-11f
#define SOFTENING_2  0.01f

[numthreads(BLOCK_SIZE,1,1)]
void c_main(uint3 group_id : SV_GroupID, uint3 dispatch_id : SV_DispatchThreadID, uint3 group_thread_id : SV_GroupThreadID, uint group_index : SV_GroupIndex)
{
	const uint dimx = uint(ceil(OBJECT_COUNT / BLOCK_SIZE));

	float3 r, f;
	float  distance_2, distance_6, inv_dis;
    float  grav;

	f.x = f.y = f.z = 0;

	//GroupMemoryBarrierWithGroupSync();
#if 1
    for(uint j=0; j<OBJECT_COUNT; j++)
    {
		r = transforms[j].pos.xyz - transforms[dispatch_id.x].pos.xyz;

		distance_2  = r.x * r.x;
        distance_2 += r.y * r.y;
        distance_2 += r.z * r.z;

        // softening
        distance_2 += SOFTENING_2;

        distance_6 = distance_2 * distance_2 * distance_2;
        distance_6 = sqrt(distance_6);
        inv_dis = 1.f / distance_6;

        grav  = physics[j].vel.w * physics[dispatch_id.x].vel.w;
        grav *= G;
        grav *= inv_dis;
		
		//f += grav * r;
		f = (j==dispatch_id.x) ? f : (f + grav * r);   
    }
#else
	for(uint tile=0; tile < dimx; tile++)
	{
		uint start = tile * BLOCK_SIZE + group_index;

		for(uint j=0; j<BLOCK_SIZE; j++)
		{
			uint counter = tile + j;

			r = transforms[counter].pos.xyz - transforms[dispatch_id.x].pos.xyz;

        	distance_2  = r.x * r.x;
        	distance_2 += r.y * r.y;
        	distance_2 += r.z * r.z;

        	// softening
        	distance_2 += SOFTENING_2;

        	distance_6 = distance_2 * distance_2 * distance_2;
        	distance_6 = sqrt(distance_6);
        	inv_dis = 1.f / distance_6;

        	grav  = physics[counter].vel.w * physics[dispatch_id.x].vel.w;
        	grav *= G;
        	grav *= inv_dis;

			f = (j==dispatch_id.x) ? f : (f + grav * r);   
		}
	}
#endif
	//GroupMemoryBarrierWithGroupSync();

	physics[dispatch_id.x].force.xyz += f;

}