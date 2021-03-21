#include "test-filter.hpp"
#include <iostream>

#include "common.hpp"

//Cuda kernel that turns pixels blue in parallel on the GPU!
__global__ void blueKernel(GPU_Cloud_F4 cloud) {
    int idx = threadIdx.x + blockIdx.x * 1024;
    if(idx >= cloud.size) return;
    
    //Set the color of all points to yellow
    cloud.data[idx].w = 9.18340948595e-41;
}

//Constructor for test filter, take in a ZED GPU cloud and convert to a struct that can be passed to CUDA Kernels
TestFilter::TestFilter() {
    
}

//Run the filter on the point cloud
void TestFilter::run(GPU_Cloud_F4 pc) {
    blueKernel<<<ceilDiv(pc.size, 1024), 1024>>>(pc);
    checkStatus(cudaGetLastError());
    cudaDeviceSynchronize();
}