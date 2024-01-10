#include <metal_stdlib>
using namespace metal;

struct stuff_data
{
    float offset_a;
    float offset_b;
    float scale_a;
    float scale_b;
};

#if 0
kernel void add_arrays(device const float* inA [[buffer(0)]],
                       device const float* inB [[buffer(1)]],
                       constant stuff_data& stuff [[buffer(2)]],
                       device float* result [[buffer(3)]],
                       uint index [[thread_position_in_grid]])
{
    // the for-loop is replaced with a collection of threads, each of which
    // calls this function.
    result[index] = (inA[index] + stuff.offset_a) * stuff.scale_a + (inB[index] + stuff.offset_b) * stuff.scale_b;
}
#else
kernel void add_arrays(device const float* inA [[buffer(0)]],
                       device const float* inB [[buffer(1)]],
                       device float* result [[buffer(2)]],
                       uint index [[thread_position_in_grid]])
{
    // the for-loop is replaced with a collection of threads, each of which
    // calls this function.
    result[index] = inA[index] + inB[index];
}
#endif