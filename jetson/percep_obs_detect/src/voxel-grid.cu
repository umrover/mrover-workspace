#include "voxel-grid.hpp"

using namespace std;

/* --- Kernels --- */
/**
* \brief adds offsets to extrema to make sure all are evenly spaced
* \param extrema: array of min and max indices of points for each axis
* \param pc: GPU point cloud
* \return void
*/
__global__ void makeCubeKernel(GPU_Cloud_F4 pc, pair<float,float>* extrema, float* partitionLength) {
    if(threadIdx.x >= 6) return; // Only need 6 threads

    // Variable Declarations
    int idx = threadIdx.x;
    int axis = idx/2;
    sl::float4 pt;
    __shared__ float dif[3];
    enum axis{x=0, y=1, z=2};

    // Calculate differences between mins and maxes
    if(idx % 2 == 0) { // If even
        pt = pc.data[(int)extrema[axis].first];
        
        //Find differences between extremes on each axis
        if(axis == 0) dif[axis] = pc.data[(int)extrema[axis].second].x - pt.x;
        else if(axis == 1) dif[axis] = pc.data[(int)extrema[axis].second].y - pt.y;
        else dif[axis] = pc.data[(int)extrema[axis].second].z - pt.z;
    }
    else { // If odd process maxes
        pt = pc.data[(int)extrema[axis].second];
    }

    __syncthreads();

    // Obnoxiously long system for making sure all mins and maxes have same difference
    
    // If z is largest difference add offset to other values
    if(dif[z] >= dif[y] && dif[z] >= dif[x]) {

        if(idx % 2 == 0) { // If even process mins    
            if(axis == x) extrema[axis].first = pt.x - ((dif[z]-dif[x])/2) - 1;

            else if(axis == y) extrema[axis].first = pt.y - ((dif[z]-dif[y])/2) - 1;

            else extrema[axis].first = pt.z - 1;
        }
        else { // If odd process maxes
            if(axis == x) extrema[axis].second = pt.x + ((dif[z]-dif[x])/2) + 1;

            else if(axis == y) extrema[axis].second = pt.y + ((dif[z]-dif[y])/2) + 1;

            else extrema[axis].second = pt.z + 1;
        }
    }

    // If y is largest distance add offset to other values
    else if(dif[y] >= dif[z] && dif[y] >= dif[x]) {
        
        if(idx % 2 == 0) { // If even process mins
            if(axis == x) extrema[axis].first = pt.x - ((dif[y]-dif[x])/2) - 1;

            else if(axis == y) extrema[axis].first = pt.y - 1;

            else extrema[axis].first = pt.z - ((dif[y]-dif[z])/2) - 1;
        }
        else { // If odd process maxes
            if(axis == x) extrema[axis].second = pt.x + ((dif[y]-dif[x])/2) + 1;

            else if(axis == y) extrema[axis].second = pt.y + 1;

            else extrema[axis].second = pt.z + ((dif[y]-dif[z])/2) + 1;
        }
    }

    // If x is largest distance add offset to other values
    else {
        
        if(idx % 2 == 0) { // If even process mins
            if(axis == x) extrema[axis].first = pt.x - 1;

            else if(axis == y) extrema[axis].first = pt.y - ((dif[x]-dif[y])/2) - 1;

            else extrema[axis].first = pt.z - ((dif[x]-dif[z])/2) - 1;
        }
        else { // If odd process maxes
            if(axis == x) extrema[axis].second = pt.x + 1;

            else if(axis == y) extrema[axis].second = pt.y + ((dif[x]-dif[y])/2) + 1;

            else extrema[axis].second = pt.z + ((dif[x]-dif[z])/2) + 1;
        }
    }
    
    // Need to grab partition lengt, but could use a better spot
    if(threadIdx.x == 0) *partitionLength = (extrema[0].second-extrema[0].first);

    return;   
}

__global__ void hashToBinsKernel(GPU_Cloud_F4 pc, Bins bins, pair<float,float>* extrema, int partitions) {
    int ptIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if(ptIdx >= pc.size) return;

    int binNum = hashToBin(pc.data[ptIdx], extrema, partitions);
    
    pc.data[ptIdx].w = atomicAdd(&bins.data[binNum],1);
}

__global__ void sortCloud(GPU_Cloud_F4 pc, Bins bins, pair<float,float>* extrema, int partitions) {
    int ptIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if(ptIdx >= pc.size) return;

    sl::float4 pt = pc.data[ptIdx]; // Is it unsafe to read and write so closely?
    
    __syncthreads();
    int offset = pt.w;
    pt.w = hashToBin(pc.data[ptIdx], extrema, partitions);
    //printf("PtVoxMain: (%f, %f, %f) Bin: %f\n", pt.x, pt.y, pt.z, pt.w);
    pc.data[offset + bins.data[int(pt.w)]] = pt; // Move point to sorted location in cloud

    /*
    if(ptIdx == 0) {
        for(int i = 0; i < pc.size; i++) {
            printf("PtVox: (%f, %f, %f)\n", pc.data[i].x, pc.data[i].y, pc.data[i].z);
        }
    }
    */
    
}

/* --- Host Functions --- */

VoxelGrid::VoxelGrid(int partitions) : partitions{partitions} {}

Bins VoxelGrid::run(GPU_Cloud_F4 &pc) {

    // Create place to store maxes
    thrust::pair< thrust::device_ptr<sl::float4>, thrust::device_ptr<sl::float4>> extrema[3];

    // Find 6 maxes of Point Cloud
    extrema[x] = thrust::minmax_element(thrust::device_ptr<sl::float4>(pc.data), 
                                        thrust::device_ptr<sl::float4>(pc.data) + pc.size, 
                                        CompareFloat4(Axis::X));
    extrema[y] = thrust::minmax_element(thrust::device_ptr<sl::float4>(pc.data), 
                                        thrust::device_ptr<sl::float4>(pc.data) + pc.size, 
                                        CompareFloat4(Axis::Y));
    extrema[z] = thrust::minmax_element(thrust::device_ptr<sl::float4>(pc.data), 
                                        thrust::device_ptr<sl::float4>(pc.data) + pc.size, 
                                        CompareFloat4(Axis::Z));    

    pair<float,float> extremaVals[3] = {
        {extrema[x].first - thrust::device_ptr<sl::float4>(pc.data), extrema[x].second - thrust::device_ptr<sl::float4>(pc.data)},
        {extrema[y].first - thrust::device_ptr<sl::float4>(pc.data), extrema[y].second - thrust::device_ptr<sl::float4>(pc.data)},
        {extrema[z].first - thrust::device_ptr<sl::float4>(pc.data), extrema[z].second - thrust::device_ptr<sl::float4>(pc.data)}
    };

    // Adjust extrema to form a cube
    checkStatus(cudaMalloc(&extremaValsGPU, sizeof(pair<float,float>)*3));
    checkStatus(cudaMemcpy(extremaValsGPU, extremaVals, sizeof(pair<float,float>)*3, cudaMemcpyHostToDevice));

    float* partitionLength;
    cudaMalloc(&partitionLength, sizeof(float));

    makeCubeKernel<<<1,MAX_THREADS>>>(pc, extremaValsGPU, partitionLength);
    checkStatus(cudaGetLastError());
    cudaDeviceSynchronize();
    cudaMemcpy(&bins.partitionLength, partitionLength, sizeof(float), cudaMemcpyDeviceToHost);
    bins.partitionLength = bins.partitionLength/partitions;

    // Initialize bins info
    bins.size = partitions*partitions*partitions+1; // +1 makes sure have total num of points
    bins.partition = partitions;
    checkStatus(cudaMalloc(&bins.data, sizeof(int) * bins.size));
    thrust::fill(thrust::device, bins.data, bins.data + bins.size, 0);

    // Hash each point to a bin
    hashToBinsKernel<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, bins, extremaValsGPU, partitions);
    checkStatus(cudaGetLastError());
    cudaDeviceSynchronize();

    // Make data contain starting values of points in point cloud
    thrust::exclusive_scan(thrust::device, bins.data, bins.data + bins.size, bins.data);
    
    // Sorts GPU cloud points into groups based on bins they hashed to
    sortCloud<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, bins, extremaValsGPU, partitions);
    checkStatus(cudaGetLastError());
    cudaDeviceSynchronize();
    
    return bins;
}
