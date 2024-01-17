#include <metal_stdlib>
using namespace metal;

//#define USE_ASTRO_UNITS

#define OBJECT_COUNT 1 << 14

#ifndef USE_ASTRO_UNITS
    #define G            6.67e-11f
#else
    #define G            4.3e-3f
#endif

#define SOFTENING_2  0.1f

kernel void gravity_calc(constant const float* x [[buffer(0)]],
                         constant const float* y [[buffer(1)]],
                         constant const float* z [[buffer(2)]],
                         constant const float* m [[buffer(3)]],
                         device float* force_x   [[buffer(4)]],
                         device float* force_y   [[buffer(5)]],
                         device float* force_z   [[buffer(6)]],
                         uint index              [[thread_position_in_grid]])
{
    float r_x, r_y, r_z, distance_2, distance_6, inv_dis;
    float grav;

    float f_x = 0;
    float f_y = 0;
    float f_z = 0;

    const float x_i = x[index];
    const float y_i = y[index];
    const float z_i = z[index];
    const float m_i = m[index];

    for(uint j=0; j<OBJECT_COUNT; j++)
    {
        r_x = x[j] - x_i;
        r_y = y[j] - y_i;
        r_z = z[j] - z_i;

        distance_2  = r_x * r_x;
        distance_2 += r_y * r_y;
        distance_2 += r_z * r_z;

        // softening
        distance_2 += SOFTENING_2;

        distance_6 = distance_2 * distance_2 * distance_2;
        distance_6 = sqrt(distance_6);
        inv_dis = 1.f / distance_6;

        grav  = m[j] * m_i;
        grav *= G;
        grav *= inv_dis;

        f_x = (j==index) ? f_x : (f_x + grav * r_x);
        f_y = (j==index) ? f_y : (f_y + grav * r_y);
        f_z = (j==index) ? f_z : (f_z + grav * r_z);        
    }

    force_x[index] += f_x;
    force_y[index] += f_y;
    force_z[index] += f_z;
}

// CUDA Code
#if 0 
__device__ float3 bodyBodyInteraction(float4 bi, float4 bj, float3 ai) 
{   
    float3 r;   // r_ij  [3 FLOPS]   
    r.x = bj.x - bi.x;   
    r.y = bj.y - bi.y;   
    r.z = bj.z - bi.z;   
    
    // distSqr = dot(r_ij, r_ij) + EPS^2  [6 FLOPS]    
    float distSqr = r.x * r.x + r.y * r.y + r.z * r.z + EPS2;   
    
    // invDistCube =1/distSqr^(3/2)  [4 FLOPS (2 mul, 1 sqrt, 1 inv)]    
    float distSixth = distSqr * distSqr * distSqr;   
    float invDistCube = 1.0f/sqrtf(distSixth);   
    
    // s = m_j * invDistCube [1 FLOP]    
    float s = bj.w * invDistCube;   
    
    // a_i =  a_i + s * r_ij [6 FLOPS]   
    ai.x += r.x * s;   
    ai.y += r.y * s;   
    ai.z += r.z * s;   
    
    return ai;
} 

__device__ float3 tile_calculation(float4 myPosition, float3 accel) 
{   
    int i;  

    extern __shared__ float4[] shPosition; 

    for (i = 0; i < blockDim.x; i++) 
    {     
        accel = bodyBodyInteraction(myPosition, shPosition[i], accel);   
    }   

    return accel; 
} 

__global__ void calculate_forces(void *devX, void *devA) 
{   
    extern __shared__ float4[] shPosition;   
    float4 *globalX = (float4 *)devX;   
    float4 *globalA = (float4 *)devA;   
    float4 myPosition;   
    int i, tile;   

    float3 acc = {0.0f, 0.0f, 0.0f};   

    int gtid = blockIdx.x * blockDim.x + threadIdx.x;   

    myPosition = globalX[gtid];   

    for (i = 0, tile = 0; i < N; i += p, tile++) 
    {     
        int idx = tile * blockDim.x + threadIdx.x;     
        
        shPosition[threadIdx.x] = globalX[idx];    
        __syncthreads();     
        acc = tile_calculation(myPosition, acc);     
        __syncthreads();   
    }   
    
    // Save the result in global memory for the integration step.    
    float4 acc4 = {acc.x, acc.y, acc.z, 0.0f};   

    globalA[gtid] = acc4; 
} 
#endif