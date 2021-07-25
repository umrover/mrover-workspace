#include "common.hpp"
#include <iostream>
#include <sl/Camera.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>


#include <pcl/common/time.h>

#include <vector>
#include <algorithm>

// TODO: add comments explaining the purpose of these functions
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

GPU_Cloud createCloud(int size) {
    GPU_Cloud g;
    cudaMalloc(&g.data, sizeof(float4)*size);
    g.size = size;
    return g;
}

__global__ void copyKernel(GPU_Cloud to, GPU_Cloud from) {
    int pointIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if(pointIdx >= from.size) return;
    if(pointIdx == 0) printf("We're using the right one\n");
    __syncthreads();
    to.data[pointIdx] = from.data[pointIdx];
}

__global__ void removeJunkKernel(GPU_Cloud cloud, int start, int maxSize) {
    int pointIdx = start + threadIdx.x + blockIdx.x * blockDim.x;
    if(pointIdx >= maxSize) return;
    cloud.data[pointIdx].x = 0;
    cloud.data[pointIdx].y = 0;
    cloud.data[pointIdx].z = 0;
    cloud.data[pointIdx].w = VIEWER_BGR_COLOR;

}

void copyCloud(GPU_Cloud &to, GPU_Cloud &from) {
    to.size = from.size;
    printf("Size moved\n");
    copyKernel<<<ceilDiv(from.size, MAX_THREADS), MAX_THREADS>>>(to, from);
    printf("From moved\n");
    checkStatus(cudaDeviceSynchronize());
}

void clearStale(GPU_Cloud &cloud, int maxSize) {
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
