#include <metal_stdlib>
using namespace metal;

struct stuff_data
{
    float offset_a;
    float offset_b;
    float scale_a;
    float scale_b;
};

kernel void add_arrays(device const float* inA [[buffer(0)]],
                       device const float* inB [[buffer(1)]],
                       constant stuff_data& stuff [[buffer(2)]],
                       device float* result [[buffer(3)]],
                       uint index [[thread_position_in_grid]])
{
    // the for-loop is replaced with a collection of threads, each of which
    // calls this function.
    result[index] = inA[index] * stuff.scale_a + inB[index] * stuff.scale_b + stuff.offset_a + stuff.offset_b;
}