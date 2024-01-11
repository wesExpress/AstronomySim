#include <metal_stdlib>
using namespace metal;

#define FIXED_DT     0.01666f

kernel void physics_update(constant const float* force_x [[buffer(0)]],
                           constant const float* force_y [[buffer(1)]],
                           constant const float* force_z [[buffer(2)]],
                           constant const float* m       [[buffer(3)]],
                           device float* vel_x           [[buffer(4)]],
                           device float* vel_y           [[buffer(5)]],
                           device float* vel_z           [[buffer(6)]],
                           device float* x               [[buffer(7)]],
                           device float* y               [[buffer(8)]],
                           device float* z               [[buffer(9)]],
                           uint index                    [[thread_position_in_grid]])
{
    const float dt_m = FIXED_DT / m[index];

    vel_x[index] += force_x[index] * dt_m;
    vel_y[index] += force_y[index] * dt_m;
    vel_z[index] += force_z[index] * dt_m;

    x[index] += vel_x[index] * FIXED_DT;
    y[index] += vel_y[index] * FIXED_DT;
    z[index] += vel_z[index] * FIXED_DT;
}