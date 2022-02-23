#include "plane-ransac.hpp"
#include <stdlib.h>
#include <unistd.h>
#include <thrust/extrema.h>


#define DEBUG

// does ceiling division
__device__ int ceilDivGPU(int a, int b) {
    return (a + b - 1) / b;
}

/**
* Launch with iteration [aka, number of candidate models] many blocks and MAX_THREADS 
* \brief Runs through RANSAC 'iterations' and computes the number of inliers for each iteration's candidate 
* \param pc Point cloud to process 
* \param candidatePlanePoints Array len(iterations) of randomly generated candidate planes. Each elt is an int3 containing 
*  the INDICIES of three points from the point cloud data array which form a candidate plane
* \param threshold How far a point may be from the candidate plane to be considered an inlier
* \param axis The axis normal to the plane we are looking for
* \param epsilon How far off the given axis a candidate plane can be without being rejected 
* \param inlierCountOut An output array len(iterations) where the number of inliers for each candidate tried is stored
*/

/* 
    Each block represents an "iteration" of the traditional RANSAC algorithm. 
    The threads in the block then check every point in the point cloud against 
    the candidate for that iteration. Each thread is responsible for 
    total_points/num_threads many points. The thread computes how many of its 
    own points are inliers. It then participates in parallel reduction to give
    the total number of inliers for that iteration in the output buffer
*/

__global__ void ransacKernel(GPU_Cloud pc, int3* candidatePlanePoints, float threshold, float3 axis, float epsilon, int* inlierCountsOut) {
    // Set up internal inlier counts for each thread of this block to write to 
    __shared__ float inlierCountInternal[MAX_THREADS];
    inlierCountInternal[threadIdx.x] = 0;

    // Which "iteration" of RANSAC
    int iteration = blockIdx.x;
    int3 candidateModelIndicies = candidatePlanePoints[iteration];

    // Number of inliers in this thread
    float inliers = 0;

    // Verify that this candidate plane is actually valid and doesn't consist of a point that goes out of bounds
    if (candidateModelIndicies.x >= pc.size || candidateModelIndicies.y >= pc.size || candidateModelIndicies.z >= pc.size) {
        // Mark as an invalid model so its not chosen 
        inlierCountsOut[iteration] = -1;
        return;
    }

    float3 modelPt0 = make_float3(pc.data[candidateModelIndicies.x]);
    float3 modelPt1 = make_float3(pc.data[candidateModelIndicies.y]);
    float3 modelPt2 = make_float3(pc.data[candidateModelIndicies.z]);

    // Create the candidate plane from the 3 points
    Plane plane(modelPt0, modelPt1, modelPt2);

    // Verify that the candidate plane's normal is within epsilon of desired axis
    if (abs(dot(normalize(plane.normal), normalize(axis))) < epsilon) {
        // Mark as an invalid model so its not chosen 
        inlierCountsOut[iteration] = 0;
        return;
    }

    // Construct predicate to chek if a point is an inlier in the plane
    NotInPlane pred(plane.normal, modelPt1, threshold);

    // Figure out how many points each thread must check 
    int pointsPerThread = ceilDivGPU(pc.size, MAX_THREADS);
    for (int i = 0; i < pointsPerThread; i++) {
        // Select a point index or return if this isn't a valid point
        int pointIdx = threadIdx.x * pointsPerThread + i;
        if (pointIdx >= pc.size) continue;

        // Point in the point cloud that could be an inlier or outlier
        float4 curPt = pc.data[pointIdx];

        // Add a 1 if inlier in plane, 0 if not 
        inliers += (pred(curPt)) ? 0 : 1;
    }

    // Load the inliers for this thread into the shared memory that all threads can access
    inlierCountInternal[threadIdx.x] = inliers;
    __syncthreads();

    // Parallel reduction to get an aggregate sum of the number of inliers for this model
    // This is all equivalent to sum(inlierCountInternal), but it does it in parallel
    int aliveThreads = (blockDim.x) / 2;
    while (aliveThreads > 0) {
        if (threadIdx.x < aliveThreads) {
            inliers += inlierCountInternal[aliveThreads + threadIdx.x];
            if (threadIdx.x >= (aliveThreads) / 2) inlierCountInternal[threadIdx.x] = inliers;
        }
        __syncthreads();
        aliveThreads /= 2;
    }

    // At the final thread, write the number of inliers for this iteration's model to global memory
    if (threadIdx.x == 0) {
        inlierCountsOut[iteration] = inliers;
    }
}

/**
 * \brief Updates the plane selection from the cloud using the given model index
 */
__global__ void getOptimalModelPoints(GPU_Cloud pc, Plane& selection, int optimalCandidateIndex, int3* candidatePlanePoints, int* maxInliersCount) {
    // Each thread is responsible for one point in the output candidate plane, there will be 3 threads
    int candidatePtNum = threadIdx.x;

    // Copy the point from the cloud into the output plane object
    float4 point;
    if (candidatePtNum == 0) point = pc.data[candidatePlanePoints[optimalCandidateIndex].x];
    if (candidatePtNum == 1) point = pc.data[candidatePlanePoints[optimalCandidateIndex].y];
    if (candidatePtNum == 2) point = pc.data[candidatePlanePoints[optimalCandidateIndex].z];

    selection[candidatePtNum] = make_float3(point);

    // Use one thread to compute the normal
    __syncthreads();
    if (threadIdx.x == 0) {
        selection.ComputeNormal();

#ifdef DEBUG
//        printf("Winner model inlier count: %d \n", *maxInliersCount);
#endif
    }
}

void RansacPlane::selectOptimalModel() {
    // Get a pointer to the elt in inlierCounts that is the greatest
    int* maxCount = thrust::max_element(thrust::device, inlierCounts, inlierCounts + iterations);
    // Pointer arithmetic gives us the optimal model index with most inliers
    int optIdx = maxCount - inlierCounts;
    // Now launch a kernel to construct the Plane object 'selection'
    getOptimalModelPoints<<<1, 3>>>(pc, *selection, optIdx, candidatePlanePoints, maxCount);
    checkStatus(cudaDeviceSynchronize());
}

RansacPlane::RansacPlane(float3 axis, float epsilon, int iterations, float threshold, int pcSize, float removalRadius)
        : pc(), axis(axis), epsilon(epsilon), threshold(threshold), pcSize(pcSize), removalRadius(removalRadius) {
    setIterations(iterations);
    cudaMalloc(&selection, sizeof(Plane));
    selectionCPU = (Plane*) malloc(sizeof(Plane));
}

Plane RansacPlane::computeModel(GPU_Cloud& pc) {
    if (pc.size == 0) {
        std::cout << "[WARNING] Can't run RANSAC on empty plane." << std::endl;
        *selectionCPU = {make_float3(0, 0, 0), make_float3(0, 0, 0), make_float3(0, 0, 0)};
        return *selectionCPU;
    }

    // Copy vars locally
    this->pc = pc;
    int blocks = iterations;
    int threads = MAX_THREADS;

    // Get a list of models and corresponding inlier count
    ransacKernel<<<blocks, threads>>>(pc, candidatePlanePoints, threshold, axis, cos(epsilon * 3.1415 / 180), inlierCounts);
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());

    // Choose the model with the greatest inlier count
    selectOptimalModel();

    // Copy selected plane to CPU
    cudaMemcpy(selectionCPU, selection, sizeof(Plane), cudaMemcpyDeviceToHost);

    // Filter out all the points in the plane
    NotInPlane predicate(selectionCPU->normal, selectionCPU->p1, threshold);
    Filter<NotInPlane>(pc, predicate, filterOp, 0);
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());

    return *selectionCPU;
}

int RansacPlane::getIterations() const {
    return iterations;
}

void RansacPlane::setIterations(int iterations) {
    if (iterations == this->iterations) return;

    //Set up buffers needed for RANSAC
    if (inlierCounts) cudaFree(inlierCounts);
    if (candidatePlanePoints) cudaFree(candidatePlanePoints);
    cudaMalloc(&inlierCounts, sizeof(int) * iterations);
    cudaMalloc(&candidatePlanePoints, sizeof(int3) * iterations);

    //Generate random numbers in CPU to use in RANSAC kernel
    int3* randomNumsCPU = (int3*) malloc(sizeof(int3) * iterations);

    for (int i = 0; i < iterations; i++) {
        int a = 0;
        int b = 0;
        int c = 0;
        // Prevent duplicate points in a particular model
        while (a == b || b == c || a == c) {
            a = rand() % pcSize;
            b = rand() % pcSize;
            c = rand() % pcSize;
        }
        randomNumsCPU[i].x = a;
        randomNumsCPU[i].y = b;
        randomNumsCPU[i].z = c;
    }
    cudaMemcpy(candidatePlanePoints, randomNumsCPU, sizeof(int3) * iterations, cudaMemcpyHostToDevice);
    free(randomNumsCPU);

    this->iterations = iterations;
}

RansacPlane::~RansacPlane() {
    cudaFree(inlierCounts);
    cudaFree(candidatePlanePoints);
    cudaFree(selection);
    free(selectionCPU);
}
