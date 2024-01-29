#include <metal_stdlib>
using namespace metal;

struct transform_elem
{
    float4 pos;
};

struct physics_elem
{
    float4 vel;

    float4 force;

    float4 accel;
};

#define ARRAY_LENGTH 30000
#define BLOCK_SIZE   256

#define FIXED_DT     0.00833f

kernel void physics_update(device transform_elem* transform[[buffer(0)]],
                           device physics_elem*   physics[[buffer(1)]],
                           uint index             [[thread_position_in_grid]])
{
    if(index<ARRAY_LENGTH)
    {
        physics[index].vel.xyz   += physics[index].accel.xyz * FIXED_DT * 0.5f;
        transform[index].pos.xyz += physics[index].vel.xyz * FIXED_DT;

        physics[index].accel.xyz  = physics[index].force.xyz * physics[index].force.w;
        physics[index].vel.xyz   += physics[index].accel.xyz * FIXED_DT * 0.5f;
    }
}