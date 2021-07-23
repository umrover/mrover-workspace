#include "common.cuh"
#include <iostream>
#include <sl/Camera.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>


#include <pcl/common/time.h>

#include <vector>
#include <algorithm>

//Cuda error checking function
bool checkStatus(cudaError_t status) {
	if (status != cudaSuccess) {
		printf("%s \n", cudaGetErrorString(status));
		return true;
	}
    return false;
}

//ceiling division
int ceilDiv(int x, int y) {
    return (x + y - 1) / y;
}


//This function convert a RGBA color packed into a packed RGBA PCL compatible format
inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}

void getRawCloud(GPU_Cloud &pc, sl::Mat zed_cloud) {
    sl::float4* ptr = zed_cloud.getPtr<sl::float4>(sl::MEM::GPU);
    pc.data = (float4*)ptr;
    pc.size = zed_cloud.getWidth() * zed_cloud.getHeight();
}

GPU_Cloud_F4 createCloud(int size) {
    GPU_Cloud_F4 g;
    cudaMalloc(&g.data, sizeof(sl::float4)*size);
    g.size = size;
    return g;
}

/*
__global__ void copyKernel(GPU_Cloud_F4 to, GPU_Cloud_F4 from) {
    int pointIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if(pointIdx >= from.size) return;
    to.data[pointIdx] = from.data[pointIdx];
}
*/
__global__ void copyKernel(GPU_Cloud to, GPU_Cloud from) {
    int pointIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if(pointIdx >= from.size) return;
    if(pointIdx == 0) printf("We're using the right one\n");
    __syncthreads();
    to.data[pointIdx] = from.data[pointIdx];
}

__global__ void removeJunkKernel(GPU_Cloud_F4 cloud, int start, int maxSize) {
    int pointIdx = start + threadIdx.x + blockIdx.x * blockDim.x;
    if(pointIdx >= maxSize) return;
    cloud.data[pointIdx].x = 0;
    cloud.data[pointIdx].y = 0;
    cloud.data[pointIdx].z = 0;
    cloud.data[pointIdx].w = VIEWER_BGR_COLOR;

}
/*
void copyCloud(GPU_Cloud_F4 &to, GPU_Cloud_F4 &from) {
    to.size = from.size;
    copyKernel<<<ceilDiv(from.size, MAX_THREADS), MAX_THREADS>>>(to, from);
    checkStatus(cudaDeviceSynchronize());
}
*/
void copyCloud(GPU_Cloud &to, GPU_Cloud &from) {
    to.size = from.size;
    printf("Size moved\n");
    copyKernel<<<ceilDiv(from.size, MAX_THREADS), MAX_THREADS>>>(to, from);
    printf("From moved\n");
    checkStatus(cudaDeviceSynchronize());
}

void clearStale(GPU_Cloud_F4 &cloud, int maxSize) {
    removeJunkKernel<<<ceilDiv(maxSize-cloud.size, MAX_THREADS), MAX_THREADS>>>(cloud, cloud.size, maxSize);
    checkStatus(cudaDeviceSynchronize());
}

__device__ __forceinline__ float atomicMinFloat (float * addr, float value) {
    float old;
    old = (value >= 0) ? __int_as_float(atomicMin((int *)addr, __float_as_int(value))) :
         __uint_as_float(atomicMax((unsigned int *)addr, __float_as_uint(value)));

    return old;
}

__device__ __forceinline__ float atomicMaxFloat (float * addr, float value) {
    float old;
    old = (value >= 0) ? __int_as_float(atomicMax((int *)addr, __float_as_int(value))) :
         __uint_as_float(atomicMin((unsigned int *)addr, __float_as_uint(value)));

    return old;
}

template<typename T>
__global__ void colorKernel(GPU_Cloud cloud, T pred, float color) {
    int pointIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if (pointIdx >= cloud.size) return;

    if (!pred(cloud.data[pointIdx])) {
        cloud.data[pointIdx].w = color;
    }
}

template<typename T>
void Filter(GPU_Cloud &cloud, T &pred, FilterOp operation, float color) {
    // Ensure pred is of the correct type
    assert((std::is_base_of<IPredicateFunctor, T>::value));
    
    if(cloud.size == 0) return;

    if (operation == FilterOp::REMOVE) {
        // Create thrust vector with all cloud point in it
        thrust::device_vector<float4> buffer(cloud.data, cloud.data+cloud.size);

        // Copy from the temp buffer back into the cloud only the points that pass the predicate 
        float4* end = thrust::copy_if(thrust::device, buffer.begin(), buffer.end(), cloud.data, pred);

        // Clear the remainder of the cloud of points that failed predicate
        thrust::fill(thrust::device, end, cloud.data+cloud.size, float4{0, 0, 0, 0});

        //update the cloud size
        cloud.size = end - cloud.data;
    }
    else if (operation == FilterOp::COLOR) {
        // Color in the points
        colorKernel<T><<<ceilDiv(cloud.size, MAX_THREADS), MAX_THREADS>>>(cloud, pred, color);
    }
    
    printf("Cloud Size: %i\n", cloud.size);
}

// Have to explicitly instantiate all possible template types
// Needed to compile template with cuda, so it had to be put in the .cu file
// Only way to put in .cu file is to explicitly instantiate, so linker knows which types are possible
template void Filter<WithinBounds>(GPU_Cloud &cloud, WithinBounds &pred, FilterOp operation, float color);
template void Filter<InPlane>(GPU_Cloud &cloud, InPlane &pred, FilterOp operation, float color);
