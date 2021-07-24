#include "plane-ransac.hpp"
#include "filter.hpp"
#include <stdlib.h>
#include <unistd.h>

// TODO: move this into common
__device__ int ceilDivGPU(int a, int b) {
    return (a + b - 1) / b;
}

/* 
LAUNCH:
    - [Block] # iterations [aka, randomly selected models] to try
    - [Thread] MAX_THREADS

REQUIRES:
    - GPU point cloud
    - A buffer to write inlier counts for each attempted model
    - A buffer that tells the kernel what the randomly selected points were for each model
    - Threshold distance for a pt to be considered an inlier
EFFECTS:

    [Block]: 
    Each block represents an "iteration" of the traditional RANSAC algorithm. 
    That is, every block has a different set of randomly chosen 3 points that define the
    model (a plane is minimally defined by 3 points). The threads in the block then check
    every point in the point cloud against the model for that block.

    [Thread]:
    Threads are used to decide how many points are inliers to the model. If
    thread max = 1024 and there are 2048 points, each thread will process 2
    points. Each thread will write the number of inliers in the set of points it evaluated
    to its specific spot in shared memory. Threads are synced, and then threads will 
    participate in a parallel reduction to give the total number of inliers for that block/model, 
    which will be returned from the kernel in the inlierCounts buffer. 
*/
__global__ void ransacKernel(GPU_Cloud pc, float* inlierCounts, int* modelPoints, float threshold, float3 axis, float epsilon) { 
    __shared__ float inlierField[MAX_THREADS];
    inlierField[threadIdx.x] = 0;

    int iteration = blockIdx.x; //which "iteration" 
    float inliers = 0; //number of inliers in this thread

    // select 3 random points from the cloud as the model that this particular block will evaluate
    int randIdx0 = modelPoints[iteration*3 + 0];
    int randIdx1 = modelPoints[iteration*3 + 1];
    int randIdx2 = modelPoints[iteration*3 + 2];

    if(randIdx0 >= pc.size || randIdx1 >= pc.size || randIdx2 >= pc.size) {
        inlierCounts[iteration] = 0;
        return;
    }

    float3 modelPt0 = make_float3(pc.data[randIdx0].x, pc.data[randIdx0].y, pc.data[randIdx0].z);
    float3 modelPt1 = make_float3(pc.data[randIdx1].x, pc.data[randIdx1].y, pc.data[randIdx1].z);
    float3 modelPt2 = make_float3(pc.data[randIdx2].x, pc.data[randIdx2].y, pc.data[randIdx2].z);    

    // Create a plane from the 3 points
    Plane plane(modelPt0, modelPt1, modelPt2);

    //check that n dot desired axis is less than epsilon, if so, return here 
    if(abs(dot(normalize(plane.normal), normalize(axis))) < epsilon) {
        if(threadIdx.x == 0) inlierCounts[iteration] = 0; //make it -1 to show invalid model?
        return;
    }

    // Construct predicate to chek if a point is an inlier in the plane
    NotInPlane pred(plane.normal, modelPt1, threshold);

    // figure out how many points each thread must compute distance for and determine if each is inlier/outlier
    int pointsPerThread = ceilDivGPU(pc.size, MAX_THREADS);
    for(int i = 0; i < pointsPerThread; i++) {
        // select a point index or return if this isn't a valid point
        int pointIdx = threadIdx.x * pointsPerThread + i;
        if(pointIdx >= pc.size) continue; //TODO Should this be return??? 
        
        // point in the point cloud that could be an inlier or outlier
        float4 curPt = make_float4(pc.data[pointIdx].x, pc.data[pointIdx].y, pc.data[pointIdx].z, 0);
        
        //add a 1 if inlier in plane, 0 if not 
        inliers += (pred(curPt)) ? 0 : 1; //very probalmatic line, how can we reduce these checks
    }
    
    //parallel reduction to get an aggregate sum of the number of inliers for this model
    //this is all equivalent to sum(inlierField), but it does it in parallel
    inlierField[threadIdx.x] = inliers;
    __syncthreads();
    int aliveThreads = (blockDim.x) / 2;
	while (aliveThreads > 0) {
		if (threadIdx.x < aliveThreads) {
            inliers += inlierField[aliveThreads + threadIdx.x];
			if (threadIdx.x >= (aliveThreads) / 2) inlierField[threadIdx.x] = inliers;
		}
		__syncthreads();
		aliveThreads /= 2;
	}

    //at the final thread, write to global memory
    if(threadIdx.x == 0) {
        inlierCounts[iteration] = inliers;
    } 
}

//to avoid kernel launch time, this could actually be appended to the bottom of the ransacKernel,
//after a syncthreads() call. But for now it will be left seperate for the purpose of clarity.
//kernel launch time is likely in tens of microseconds. TODO test to confirm this theory
/* 
LAUNCH:
    - [Block] 1
    - [Thread] Number of attempted models ("iterations")

REQUIRES:
    - Buffer with inlier counts for each attempted model in RANSAC
    - Output in memory the 3 points of the selected model

EFFECTS:
    - Selects the optimal model (the one with the greatest inlier count)
    - Outputs the points of this model 
*/
// optimalMOdel out = { p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z}
__global__ void selectOptimalRansacModel(GPU_Cloud pc, float* inlierCounts, int* modelPoints, Plane& optimalModelOut, int iterations, int* optimalModelIndex) {
    
    __shared__ float inlierCountsLocal[MAX_THREADS];
    __shared__ int modelIndiciesLocal[MAX_THREADS];
    
    //TODO: This can easily index out of bounds if threadIdx.x > numPoints in the PC
    //another problem:  we must initalize the inlierCountsLocal with low valeus that wont be chosen
    
    // Populate the locally defined arrays
    float inliers = (threadIdx.x < iterations) ? inlierCounts[threadIdx.x] : 0;
    int optimalModel = threadIdx.x;
    inlierCountsLocal[threadIdx.x] = inliers;
    modelIndiciesLocal[threadIdx.x] = optimalModel;
    __syncthreads();

    // Parallel reduction to determine the model with the largest number of inliers
    int aliveThreads = (blockDim.x) / 2;
	while (aliveThreads > 0) {
		if (threadIdx.x < aliveThreads) {
            int temp = max(inlierCountsLocal[aliveThreads + threadIdx.x], inliers);
            if(temp > inliers) {
                inliers = temp;
                optimalModel = modelIndiciesLocal[aliveThreads + threadIdx.x];
            }

			if (threadIdx.x >= (aliveThreads) / 2) {
                modelIndiciesLocal[threadIdx.x] = optimalModel;
                inlierCountsLocal[threadIdx.x] = inliers;
            }
		}
		__syncthreads();
		aliveThreads /= 2;
	}

    //at the final thread, write to global memory
    if(threadIdx.x < 3) {
        float3 pt = make_float3(pc.data[ modelPoints[modelIndiciesLocal[0]*3 + threadIdx.x] ].x, pc.data[ modelPoints[modelIndiciesLocal[0]*3 + threadIdx.x] ].y, pc.data[ modelPoints[modelIndiciesLocal[0]*3 + threadIdx.x] ].z);

        // Set output model
        optimalModelOut[threadIdx.x] = pt;
    }
    
    __syncthreads();

    if(threadIdx.x == 0) {
        // Find normal to the plane
        optimalModelOut.ComputeNormal();

        printf("winner model inlier count: %f \n", inlierCountsLocal[0]);
        
        //check here if the inlier counts local is 0, if so return -1 instead
        *optimalModelIndex = (inlierCountsLocal[0] > 1.0) ? modelIndiciesLocal[0] : -1;
    }
}

RansacPlane::RansacPlane(float3 axis, float epsilon, int iterations, float threshold, int pcSize, float removalRadius)
    : pc(pc), axis(axis), epsilon(epsilon), iterations(iterations), threshold(threshold), removalRadius(removalRadius)  {
    
    //Set up buffers needed for RANSAC
    cudaMalloc(&inlierCounts, sizeof(float) * iterations); 
    cudaMalloc(&modelPoints, sizeof(int) * iterations * 3);
    cudaMalloc(&selection, sizeof(Plane));
    cudaMalloc(&optimalModelIndex, sizeof(int));

    //Generate random numbers in CPU to use in RANSAC kernel
    int* randomNumsCPU = (int*) malloc(sizeof(int) * iterations* 3);

    for(int i = 0; i < iterations; i++) {
        int a = 0;
        int b = 0;
        int c = 0;
        while(a == b || b == c || a == c) {
            a = rand() % pcSize;
            b = rand() % pcSize;
            c = rand() % pcSize;
        }
    
        randomNumsCPU[i*3] = a;
        randomNumsCPU[i*3 + 1] = b;
        randomNumsCPU[i*3 + 2] = c; 
    }

    cudaMemcpy(modelPoints, randomNumsCPU, sizeof(int) * iterations * 3, cudaMemcpyHostToDevice);
    free(randomNumsCPU);

    // Generate a buffer for retreiving the selected model from CUDA Kernels
    selectedModel = (Plane*) malloc(sizeof(Plane)); 
}

Plane RansacPlane::computeModel(GPU_Cloud &pc) {
    if(pc.size == 0) return {make_float3(0,0,0), make_float3(0,0,0), make_float3(0,0,0)};

    // Copy vars locally
    this->pc = pc;
    int blocks = iterations;
    int threads = MAX_THREADS;
    
    // Get a list of models and corresponding inlier count
    ransacKernel<<<blocks, threads>>>(pc, inlierCounts, modelPoints, threshold, axis, cos(epsilon*3.1415/180));
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());

    // Choose the model with the greatest inlier count
    selectOptimalRansacModel<<<1, MAX_THREADS>>>(pc, inlierCounts, modelPoints, *selection, iterations, optimalModelIndex);
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());   

    // Copy selected plane to CPU
    cudaMemcpy(selectedModel, selection, sizeof(Plane), cudaMemcpyDeviceToHost);
    
    // Filter out all the points in the plane
    NotInPlane predicate(selectedModel->normal, selectedModel->p1, threshold);
    Filter<NotInPlane>(pc, predicate, FilterOp::REMOVE, 0);
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());

    return *selectedModel;
}

RansacPlane::~RansacPlane() {
    cudaFree(inlierCounts);
    cudaFree(modelPoints);
    cudaFree(selection);
    cudaFree(optimalModelIndex);
    free(selectedModel);
}