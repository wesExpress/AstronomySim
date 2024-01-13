#include <metal_stdlib>
using namespace metal;

#define OBJECT_COUNT 1 << 14
#define G            6.67e-11f
#define MIN_RADIUS   0.5f

kernel void gravity_calc(constant const float* x [[buffer(0)]],
                         constant const float* y [[buffer(1)]],
                         constant const float* z [[buffer(2)]],
                         constant const float* m [[buffer(3)]],
                         device float* force_x   [[buffer(4)]],
                         device float* force_y   [[buffer(5)]],
                         device float* force_z   [[buffer(6)]],
                         uint index              [[thread_position_in_grid]])
{
    float r_x, r_y, r_z, mag, distance_2;
    float f;

    float f_x = 0;
    float f_y = 0;
    float f_z = 0;

    const float x_i = x[index];
    const float y_i = y[index];
    const float z_i = z[index];
    const float m_i = m[index];

    for(uint i=0; i<index; i++)
    {
        // distance
        r_x = x[i] - x_i;
        r_y = y[i] - y_i;
        r_z = z[i] - z_i;

        distance_2 =  r_x * r_x;
        distance_2 += r_y * r_y;
        distance_2 += r_z * r_z;
        mag = sqrt(distance_2);
        distance_2 = 1.f / distance_2;

        f  = m[i] * m_i;
        f *= G;
        f *= distance_2;

        mag = 1.f / mag;

        r_x *= mag;
        r_y *= mag;
        r_z *= mag;

        if (mag < MIN_RADIUS) continue;

        f_x += f * r_x;
        f_y += f * r_y;
        f_z += f * r_z; 
    }

    for(uint i=index+1; i<OBJECT_COUNT; i++)
    {
        // distance
        r_x = x[i] - x_i;
        r_y = y[i] - y_i;
        r_z = z[i] - z_i;

        distance_2  = r_x * r_x;
        distance_2 += r_y * r_y;
        distance_2 += r_z * r_z;
        mag = sqrt(distance_2);
        distance_2 = 1.f / distance_2;

        f  = m[i] * m_i;
        f *= G;
        f *= distance_2;

        mag = 1.f / mag;

        r_x *= mag;
        r_y *= mag;
        r_z *= mag;

        if (mag < MIN_RADIUS) continue;

        f_x += f * r_x;
        f_y += f * r_y;
        f_z += f * r_z; 
    }

    force_x[index] = f_x;
    force_y[index] = f_y;
    force_z[index] = f_z;
}