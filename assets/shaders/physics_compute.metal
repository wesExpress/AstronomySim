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
};

#define FIXED_DT     0.00833f
#define OBJECT_COUNT 16000

kernel void physics_update(device transform_elem* transform[[buffer(0)]],
                           device physics_elem*   physics[[buffer(1)]],
                           uint index             [[thread_position_in_grid]])
{
    if(index>=OBJECT_COUNT) return;

    const float dt_m = FIXED_DT / physics[index].vel.w;

    physics[index].vel.xyz += physics[index].force.xyz * dt_m;

    transform[index].pos.xyz += physics[index].vel.xyz * FIXED_DT;
}