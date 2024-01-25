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

//#define USE_ASTRO_UNITS

#define OBJECT_COUNT 10000

#ifndef USE_ASTRO_UNITS
    #define G            6.67e-11f
#else
    #define G            4.3e-3f
#endif

#define SOFTENING_2  0.05f

kernel void gravity_calc(device transform_elem* transform[[buffer(0)]],
                         device physics_elem*   physics[[buffer(1)]],
                         uint index            [[thread_position_in_grid]])
{
    if(index>=OBJECT_COUNT) return;

    float3 r, f;
    float distance_2, distance_6, inv_dis;
    float grav;

    f.x = f.y = f.z = 0;

    const float3 p_i = transform[index].pos.xyz;
    const float  m_i = physics[index].vel.w;

    for(uint j=0; j<OBJECT_COUNT; j++)
    {
        r = transform[j].pos.xyz - p_i;

        distance_2 = dot(r,r);

        // softening
        distance_2 += SOFTENING_2;

        distance_6 = distance_2 * distance_2 * distance_2;
        distance_6 = sqrt(distance_6);
        inv_dis = 1.f / distance_6;

        grav  = physics[j].vel.w * m_i;
        grav *= G;
        grav *= inv_dis;

        f = (j==index) ? f : (f + grav * r);  
    }

    physics[index].force.xyz += f;
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