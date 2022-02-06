#include "filter.hpp"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/fill.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/sequence.h>
#include <assert.h>

/**
 * \brief templated kernel used for coloring points in the point cloud
 * \param cloud: cloud of points that will be colored
 * \param pred: predicate used for evaluating points. If pred returns false point is colored
 * \param color: color the point will be assigned
 */
template<typename T>
__global__ void colorKernel(GPU_Cloud cloud, T pred, float color) {
    int pointIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if (pointIdx >= cloud.size) return;

    if (!pred(cloud.data[pointIdx])) {
        cloud.data[pointIdx].w = color;
    }
}

template<typename T>
void Filter(GPU_Cloud& cloud, T& pred, FilterOp operation, float color) {
    // Ensure pred is of the correct type
    assert((std::is_base_of<IPredicateFunctor, T>::value));

    if (cloud.size == 0) return;

    if (operation == FilterOp::REMOVE) {
        // Create thrust vector with all cloud point in it
        thrust::device_vector<float4> buffer(cloud.data, cloud.data + cloud.size);

        // Copy from the temp buffer back into the cloud only the points that pass the predicate 
        float4* end = thrust::copy_if(thrust::device, buffer.begin(), buffer.end(), cloud.data, pred);

        // Clear the remainder of the cloud of points that failed predicate
        thrust::fill(thrust::device, end, cloud.data + cloud.size, float4{0, 0, 0, 0});

        //update the cloud size
        cloud.size = end - cloud.data;
    } else if (operation == FilterOp::COLOR) {
        // Color in the points
        colorKernel<T><<<ceilDiv(cloud.size, MAX_THREADS), MAX_THREADS>>>(cloud, pred, color);
    }

//    printf("Cloud Size: %i\n", cloud.size);
}

// Have to explicitly instantiate all possible template types
// Needed to compile template with cuda, so it had to be put in the .cu file
// Only way to put in .cu file is to explicitly instantiate, so linker knows which types are possible
template void Filter<WithinBounds>(GPU_Cloud& cloud, WithinBounds& pred, FilterOp operation, float color);

template void Filter<NotInPlane>(GPU_Cloud& cloud, NotInPlane& pred, FilterOp operation, float color);
