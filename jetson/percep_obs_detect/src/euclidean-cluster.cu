#include "euclidean-cluster.hpp"
#include <thrust/scan.h>
#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include "common.hpp"
#include <limits>     //for std::numeric_limits<> 

//Helper functions
__device__ float getFloatData(int axis, float4 &val) {
    if(!axis)
        return val.x;
    else if(axis == 1)
        return val.y;
    else
        return val.z;
}
            
__device__ float getData(int axis, int index, float4 *data) {
    return getFloatData(axis, data[index]); 
}

__global__ void determineGraphStructureKernelN2(GPU_Cloud pc, float tolerance, int* listStart) {
    int ptIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(ptIdx >= pc.size) return;
    if (ptIdx == 0) printf("OK\n");

    float4 ptLocal = pc.data[ptIdx];
    float3 pt = make_float3(ptLocal.x, ptLocal.y, ptLocal.z);
    int neighborCount = 0;
    
    //horrible slow way of doing this that is TEMPORARY --> please switch to radix sorted bins
    for(int i = 0; i < pc.size; i++) {
        float4 tmpPtLocal = pc.data[i];
        float3 dvec = (pt - make_float3(tmpPtLocal.x, tmpPtLocal.y, tmpPtLocal.z));
        //this is a neighbor
        if( length(dvec) < tolerance && i != ptIdx) {
            neighborCount++;
        }
    }
    listStart[ptIdx] = neighborCount;

    //we must do an exclusive scan using thrust after this kernel
    if (ptIdx == 0) printf("Nice\n");
}


/* This kernel builds the graph 
Fairly standard adjacency list structure. 
*/
__global__ void buildGraphKernelN2(GPU_Cloud pc, float tolerance, int* neighborLists, int* listStart, int* labels, bool* f1, bool* f2) {
    int ptIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(ptIdx >= pc.size) return;

    float4 ptLocal = pc.data[ptIdx];
    float3 pt = make_float3(ptLocal.x, ptLocal.y, ptLocal.z);
    int neighborCount = 0;
    //get the adjacency list for this point
    int* list = neighborLists + listStart[ptIdx];
    
    //horrible slow way of doing this that is TEMPORARY --> please switch to radix sorted bins
    for(int i = 0; i < pc.size; i++) {

        float4 tmpPtLocal = pc.data[i];
        float3 dvec = (pt - make_float3(tmpPtLocal.x, tmpPtLocal.y, tmpPtLocal.z));
        //this is a neighbor
        if( length(dvec) < tolerance && i != ptIdx) {
            list[neighborCount] = i;
            neighborCount++;
        }
    }
    
    labels[ptIdx] = ptIdx;
    f1[ptIdx] = true;
    f2[ptIdx] = false;
}

__global__ void determineGraphStructureKernel(GPU_Cloud pc, float tolerance, int* listStart, Bins bins) {
    int ptIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(ptIdx >= pc.size) return;

    float3 pt = make_float3(pc.data[ptIdx].x, pc.data[ptIdx].y, pc.data[ptIdx].z);
    int neighborCount = 0;
    float partitionLength = bins.partitionLength;
    int binNum = pc.data[ptIdx].w;
    int partitions = bins.partition;

    // Assume that the point is 1/4 the way from the edge of the bin, subtract this from
    // the tolerance and then figure out how many bins outward need to be traversed to satisfy the tolerance
    // Can make 1/4 larger to search fewer bins or smaller to search more bins
    int check = 3;
    if(ptIdx == check) printf("partitionLength: %f\n", partitionLength);
    int binsWithinThreshold = (tolerance - 0.25 * partitionLength)/partitionLength;
    int totalBinsSearch = (binsWithinThreshold*2+1)^3; 
    int partitionsCubed = partitions*partitions*partitions;
    
    if(ptIdx == check) printf("BinsWithinThreshold: %i\n", binsWithinThreshold);

    /*
    The bin one away from currBin in Z direction is binNum +1
    The bin one away from currBin in Y direction is binNum +partitions
    The bin one away from currBin in X direction is binNum +partitions^2
    */
    //Start at lower left, iterate front to back, bottom to top, left to right 
    //Iterate left to right
    // Create array of bins to search through 

    // Method for making sure you don't try to access negative bins. Definitely could use some revising to account for
    // other out of bounds cases that aren't necessarily negative

    int binsX, binsY, binsZ;
    binsX = binsY = binsZ = binsWithinThreshold;
    int startBin = binNum;
    int partitionsSquared = partitions*partitions;
    
    if(startBin - binsWithinThreshold >= 0) startBin -= binsWithinThreshold;
    else binsX = 0;
    
    if(startBin - binsWithinThreshold*partitions >= 0) startBin -= partitions*binsWithinThreshold; 
    else binsY = 0;

    if(startBin - binsWithinThreshold*partitions*partitions >=0) startBin -= partitionsSquared*binsWithinThreshold;
    else binsZ = 0;

    int binsSearched = 0;
    if(ptIdx == check) printf("PC Size: %i\n", pc.size);
    if(ptIdx == check) printf("PtMain: (%f, %f, %f)\n", pt.x, pt.y, pt.z);
    if(ptIdx == check) printf("BinNum: %i\n StartBin: %i\n", binNum, startBin);
    for(int i = startBin; i <= startBin + binsZ*2*partitionsSquared && i < partitionsCubed; i += partitionsSquared) {
        //Iterate bottom to top
        for(int j = i; j <= i + binsY*2*partitions && j < partitionsCubed; j += partitions) {
            //Iterate front to back
            for(int k = j; k <= j + binsX*2 && k < partitionsCubed; ++k){
                //Iterate through points in cloud
                if(ptIdx== check) printf("Differece: %i\n", bins.data[k+1] - bins.data[k]);
                if(ptIdx == check) printf("k Val: %i\n", k);
                for(int l = 0; l < bins.data[k+1] - bins.data[k]; ++l) {
                    float3 dvec = (pt - make_float3(pc.data[l+bins.data[k]].x, pc.data[l+bins.data[k]].y, pc.data[l+bins.data[k]].z));
                    //this is a neighbor
                    if( length(dvec) < tolerance && l+bins.data[k] != ptIdx) {
                        neighborCount++;
                    }
                    if(ptIdx == check) {
                        //printf("PtECE: (%f, %f, %f)\n", pc.data[l+bins.data[k]].x, pc.data[l+bins.data[k]].y, pc.data[l+bins.data[k]].z);
                    }
                }
                binsSearched++;
            }
        }
    }

    if(ptIdx == 0) printf("Bins Searched: %i\n", binsSearched);
    listStart[ptIdx] = neighborCount;

}


/* This kernel builds the graph 
Fairly standard adjacency list structure. 
*/
__global__ void buildGraphKernel(GPU_Cloud pc, float tolerance, int* neighborLists, int* listStart, int* labels, bool* f1, bool* f2, Bins bins) {
    int ptIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(ptIdx >= pc.size) return;

    float3 pt = make_float3(pc.data[ptIdx].x, pc.data[ptIdx].y, pc.data[ptIdx].z);
    int neighborCount = 0;
    float partitionLength = bins.partitionLength;
    int startBin = pc.data[ptIdx].w;
    int partitions = bins.partition;

    // Assume that the point is 1/4 the way from the edge of the bin, subtract this from
    // the tolerance and then figure out how many bins outward need to be traversed to satisfy the tolerance
    // Can make 1/4 larger to search fewer bins or smaller to search more bins
    int binsWithinThreshold = (tolerance - 0.25 * partitionLength)/partitionLength;
    int totalBinsSearch = (binsWithinThreshold*2+1)^3; 
    int partitionsCubed = partitions*partitions*partitions;
    int check = 3;
    if(ptIdx == check) printf("BinsWithinThreshold: %i\n", binsWithinThreshold);

    /*
    The bin one away from currBin in Z direction is binNum +1
    The bin one away from currBin in Y direction is binNum +partitions
    The bin one away from currBin in X direction is binNum +partitions^2
    */
    //Start at lower left, iterate front to back, bottom to top, left to right 
    //Iterate left to right
    // Create array of bins to search through 

    // Method for making sure you don't try to access negative bins. Definitely could use some revising to account for
    // other out of bounds cases that aren't necessarily negative
    int binsX, binsY, binsZ;
    binsX = binsY = binsZ = binsWithinThreshold;

    //int partitionsSquared = partitions*partitions;
    int* list = neighborLists + listStart[ptIdx];

    if(startBin - binsWithinThreshold >= 0) startBin -= binsWithinThreshold;
    else binsX = 0;
    
    if(startBin - binsWithinThreshold*partitions >= 0) startBin -= partitions*binsWithinThreshold; 
    else binsY = 0;

    if(startBin - binsWithinThreshold*partitions*partitions >=0) startBin -= partitions*partitions*binsWithinThreshold;
    else binsZ = 0;

    //int binsSearched = 0;
    //if(ptIdx == check) printf("PC Size: %i\n", pc.size);
    //if(ptIdx == check) printf("PtMain: (%f, %f, %f)\n", pt.x, pt.y, pt.z);
    //if(ptIdx == check) printf("BinNum: %i\n StartBin: %i\n", binNum, startBin);
    for(int i = startBin; i <= startBin + binsZ*2*partitions*partitions && i < partitionsCubed; i += partitions*partitions) {
        //Iterate bottom to top
        for(int j = i; j <= i + binsY*2*partitions && j < partitionsCubed; j += partitions) {
            //Iterate front to back
            for(int k = j; k <= j + binsX*2 && k < partitionsCubed; ++k){
                //Iterate through points in cloud
                if(ptIdx == check) printf("Differece: %i\n", bins.data[k+1] - bins.data[k]);
                if(ptIdx == check) printf("k Val: %i\n", k);
                for(int l = 0; l < bins.data[k+1] - bins.data[k]; ++l) {
                    float3 dvec = (pt - make_float3(pc.data[l+bins.data[k]].x, pc.data[l+bins.data[k]].y, pc.data[l+bins.data[k]].z));
                    //this is a neighbor
                    if( length(dvec) < tolerance && l+bins.data[k] != ptIdx) {
                        list[neighborCount] = l+bins.data[k];
                        neighborCount++;
                    }
                    if(ptIdx == check) {
                        //printf("PtECE: (%f, %f, %f)\n", pc.data[l+bins.data[k]].x, pc.data[l+bins.data[k]].y, pc.data[l+bins.data[k]].z);
                    }
                }
                //binsSearched++;
            }
        }
    }

    //if(ptIdx == 0) printf("Bins Searched: %i\n", binsSearched);
    labels[ptIdx] = ptIdx;
    f1[ptIdx] = true;
    f2[ptIdx] = false;
}

/*
this kernel propogates labels, it must be called in a loop until its flag "m" is false, indicating
no more changes are pending. 
*/
//each thread is a point 
__global__ void propogateLabels(GPU_Cloud pc, int* neighborLists, int* listStart, int* labels, bool* f1, bool* f2, bool* m) {
    int ptIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(ptIdx >= pc.size) return;

    if(ptIdx == -1){
        for(int i = 0; i < 10; i++){
            printf("Pt %i: ", i);
            for(int j = listStart[i]; j < listStart[i+1]; ++j){
                printf("%i, ", neighborLists[j]);
            }
            printf("\n");    
        }
        
    }
    //debug lines
   // if(threadIdx.x == 0) *m = false;
   // __syncthreads();
   // printf("pt idx: %d, label: %d, flag: %d frontier one: %d frontier two: %d \n", ptIdx, labels[ptIdx], (*m) ? 1 : 0, f1[ptIdx] ? 1 : 0, f2[ptIdx] ? 1 : 0);

    bool mod = false;
    //TODO, load the NEIGHBOR list to shared memory 
    if(f1[ptIdx]) {
        //printf("active frontier %d \n", ptIdx);

        int* list = neighborLists + listStart[ptIdx];
        int listLen = listStart[ptIdx+1] - listStart[ptIdx];
        f1[ptIdx] = false;
        int myLabel = labels[ptIdx];

        //printf("[len] pt idx: %d, list-len: %d \n", ptIdx, listLen);

        for(int i = 0; i < listLen; i++) {
            int otherLabel = labels[list[i]];
            if(myLabel < otherLabel) { //are these reads actually safe?
                //printf("-- updating other: %d to be %d \n", otherLabel, myLabel);

                atomicMin(&labels[list[i]], myLabel);
                f2[list[i]] = true;
                *m = true;
            } else if(myLabel > otherLabel) {
                myLabel = otherLabel;
                mod = true;
            }
        }

        if(mod) {
            atomicMin(&labels[ptIdx], myLabel);
            f2[ptIdx] = true;
            *m = true;
        }
    } 

    /*
    __syncthreads();
    if(threadIdx.x == 0) {
    if(*m) printf("still going \n");
    else printf("done \n");
    }*/
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

//this debug kernel colors points based on their label
__global__ void colorClusters(GPU_Cloud pc, int* labels, int* keys, int* values, int minCloudSize, int numClusters, float* minX, float* maxX, float* minY, float* maxY, float* minZ, float* maxZ) {
    int ptIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(ptIdx >= pc.size) return;

    //DEBUG STEP REMOVE
    //pc.data[ptIdx].w = 9.18340948595e-41;
    //return;

    int i = 0;
    while(true) {
        if(labels[ptIdx] == keys[i]) {
            if(values[i] < minCloudSize) {
                pc.data[ptIdx].w = VIEWER_BGR_COLOR;
                return;
            }
            else break;
        }
        i++;
    }
    
    //float red = 3.57331108403e-43;
    //float green = 9.14767637511e-41;
    //float blue = 2.34180515203e-38;
    //float magenta = 2.34184088514e-38; 
    float yellow = 9.18340948595e-41;
    
    pc.data[ptIdx].w = yellow+0.0000000000000001*labels[ptIdx]*4;
    
    //X
    atomicMinFloat(&minX[i], pc.data[ptIdx].x);
    atomicMaxFloat(&maxX[i], pc.data[ptIdx].x);

    //Y
    atomicMinFloat(&minY[i], pc.data[ptIdx].y);
    atomicMaxFloat(&maxY[i], pc.data[ptIdx].y);

    //Z
    atomicMinFloat(&minZ[i], pc.data[ptIdx].z);
    atomicMaxFloat(&maxZ[i], pc.data[ptIdx].z);
}

//this is practically serial, can we just color using OpenGL functions
__global__ void colorExtrema(GPU_Cloud pc, int* values, int minSize, int* labels, int numClustersOrig, int* validClustersCount, float* minX, float* maxX,  float* minY, float* maxY, float* minZ, float* maxZ) {
    int clusterIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(clusterIdx >= numClustersOrig) return;

    int place = 0;
    if(values[clusterIdx] > minSize) place = atomicAdd(validClustersCount, 1);
    else return;

  
    pc.data[place*2] = make_float4(minX[clusterIdx], (minY[clusterIdx] + maxY[clusterIdx])/2, minZ[clusterIdx], 0.0);
    pc.data[place*2+1] = make_float4(maxX[clusterIdx], (minY[clusterIdx] + maxY[clusterIdx])/2, minZ[clusterIdx], 0.0);
        
    //serailze the extrema into a float4 vector using the "place"
}

__global__ void colorClustersNew(GPU_Cloud pc, int* labels, int* keys, int numClusters) {
    int ptIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if(ptIdx >= pc.size) return;

    float yellow = 9.18340948595e-41;

    for(int i = 0; i < numClusters; i++) {
        if(labels[ptIdx] == keys[i]) {
            pc.data[ptIdx].w = yellow+0.0000000000000001*labels[ptIdx]*4;
            return;
        }
    }

    pc.data[ptIdx].w = VIEWER_BGR_COLOR;

}

    
class is_smaller_than_min {
public: 
    is_smaller_than_min(int min) : min(min) {}
    __device__ __host__ bool operator()(const int size) {
        return size < min;
    }
private:
    int min;
};

EuclideanClusterExtractor::EuclideanClusterExtractor(float tolerance, int minSize, float maxSize, size_t cloudArea, int partitions) 
: tolerance{tolerance}, minSize{minSize}, maxSize{maxSize}, partitions{partitions} {

    cudaMalloc(&listStart, sizeof(int)*(cloudArea+1));
    cudaMalloc(&labels, sizeof(int)*cloudArea);
    cudaMalloc(&f1, sizeof(bool)*cloudArea);
    cudaMalloc(&f2, sizeof(bool)*cloudArea);
    cudaMalloc(&stillGoing, sizeof(bool));

}

EuclideanClusterExtractor::EuclideanClusterExtractor() {}

//perhaps use dynamic parallelism 
EuclideanClusterExtractor::ObsReturn EuclideanClusterExtractor::extractClusters(GPU_Cloud &pc, Bins &bins) {
    ObsReturn empty;
    empty.size = 0;
    if(pc.size == 0) return empty;
    // Find the structure for adjacency list of all points
    #if !VOXEL
        determineGraphStructureKernelN2<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, tolerance, listStart);
        checkStatus(cudaGetLastError());
        checkStatus(cudaDeviceSynchronize());
    #endif
    #if VOXEL
       determineGraphStructureKernel<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, tolerance, listStart, bins);
       checkStatus(cudaGetLastError());
       checkStatus(cudaDeviceSynchronize());
    #endif
    thrust::exclusive_scan(thrust::device, listStart, listStart+pc.size+1, listStart, 0);
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());
    
    // Create helpful variables
    int totalAdjanecyListsSize;
    checkStatus(cudaMemcpy(&totalAdjanecyListsSize, &listStart[pc.size], sizeof(int), cudaMemcpyDeviceToHost));
    cudaMalloc(&neighborLists, sizeof(int)*totalAdjanecyListsSize);
    // Populate adjacency list structure
    #if !VOXEL
        buildGraphKernelN2<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, tolerance, neighborLists, listStart, labels, f1, f2);
        checkStatus(cudaGetLastError());
        checkStatus(cudaDeviceSynchronize());
    #endif
    #if VOXEL
        buildGraphKernel<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, tolerance, neighborLists, listStart, labels, f1, f2, bins);
        checkStatus(cudaGetLastError());
        checkStatus(cudaDeviceSynchronize());
    #endif

    //std::cerr<<"Graph kernel built\n";
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());
    bool stillGoingCPU = true;    
    while(stillGoingCPU) {
        //one iteration of label propogation
        stillGoingCPU = false;
        cudaMemcpy(stillGoing, &stillGoingCPU, sizeof(bool), cudaMemcpyHostToDevice);
        propogateLabels<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, neighborLists, listStart, labels, f1, f2, stillGoing);

        //swap the frontiers
        bool* t = f1;
        f1 = f2;
        f2 = t;

        //get flag to see if we are done
        cudaMemcpy(&stillGoingCPU, stillGoing, sizeof(bool), cudaMemcpyDeviceToHost);
    }

    //Build useful data structures mapping points to clusters and clusters to number of points
    //Let C be the number of clusters, and N the number of points in the cloud
    //After we preform the operations in this block, the contents of the vectors are as follows:
    thrust::device_vector<int> labelsSorted(pc.size); //Point labels sorted by cluster. Len(N). 
    thrust::device_vector<int> count(pc.size, 1); //buffer of all 1s. Len(N)
    thrust::device_vector<int> keys(pc.size); //Each clusters unique ID in ascending order Len(C)
    thrust::device_vector<int> values(pc.size); //The number of points in each cluster in ascending order by ID. Len(C)
    thrust::copy(thrust::device, labels, labels+pc.size, labelsSorted.begin()); //first make the labels sorted contain the labels in order of points
    thrust::sort(thrust::device, labelsSorted.begin(), labelsSorted.end()); //now sort the labels by their label idx, 
    auto pair = thrust::reduce_by_key(thrust::device, labelsSorted.begin(), labelsSorted.end(), count.begin(), keys.begin(), values.begin()); //remove duplicate labels and determine the number of points belonging to each label    
   
    //Determine how many clusters there actually are
    
    int numClustersOrig = thrust::distance(keys.begin(), pair.first);

    

    float *minX, *maxX, *minY, *maxY, *minZ, *maxZ; 
    cudaMalloc(&minX, sizeof(float)*numClustersOrig);
    cudaMalloc(&maxX, sizeof(float)*numClustersOrig);
    cudaMalloc(&minY, sizeof(float)*numClustersOrig);
    cudaMalloc(&maxY, sizeof(float)*numClustersOrig);
    cudaMalloc(&minZ, sizeof(float)*numClustersOrig);
    cudaMalloc(&maxZ, sizeof(float)*numClustersOrig);
    thrust::fill(thrust::device, minX, minX + numClustersOrig, std::numeric_limits<float>::max());
    thrust::fill(thrust::device, maxX, maxX + numClustersOrig, -std::numeric_limits<float>::max());
    thrust::fill(thrust::device, minY, minY + numClustersOrig, std::numeric_limits<float>::max());
    thrust::fill(thrust::device, maxY, maxY + numClustersOrig, -std::numeric_limits<float>::max());
    thrust::fill(thrust::device, minZ, minZ + numClustersOrig, std::numeric_limits<float>::max());
    thrust::fill(thrust::device, maxZ, maxZ + numClustersOrig, -std::numeric_limits<float>::max());

    /*
    //Now get a list of cluster ID keys that are bigger than the min size by removing those that are less than the min size
    is_smaller_than_min pred(minSize);
    auto keyEnd = thrust::remove_if(thrust::device, keys.begin(), keys.end(), values.begin(), pred);
    thrust::remove_if(thrust::device, values.begin(), values.end(), pred);

    int numClusters = keyEnd - keys.begin();
    keys.resize(numClusters);
    values.resize(numClusters);
    std::cout << "CLUSTERS NEW: " << numClusters << std::endl; */

    //find interest points
    //exculsive scan on values to give the indicies of each new cluster start in the points array 
    //for each on the array returned by the exclusive scan, going from the prev element to the cur,
    //first determine if the labels for that range are contained within the clusterIDs [keys] vector (binary search),
    //if so, then find extrema, otherwise move on

    //Call a kernel to color the clusters for debug reasons
    int* gpuKeys = thrust::raw_pointer_cast( keys.data() );
    int* gpuVals = thrust::raw_pointer_cast( values.data() );
    colorClusters<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, labels, gpuKeys, gpuVals, minSize, numClustersOrig, minX, maxX, minY, maxY, minZ, maxZ);

    int * validClustersCount;
    cudaMalloc(&validClustersCount, sizeof(int));
    cudaMemset(validClustersCount, 0, sizeof(int));
    //colorExtrema<<<ceilDiv(numClustersOrig, MAX_THREADS), MAX_THREADS >>>(pc, gpuVals, minSize, labels, numClustersOrig, validClustersCount, minX, maxX, minY, maxY, minZ, maxZ);
    // TODO: make maxSize do something
    float *minXCPU, *maxXCPU, *minYCPU, *maxYCPU, *minZCPU, *maxZCPU; 
    minXCPU = (float*) malloc(sizeof(float)*numClustersOrig);
    maxXCPU = (float*) malloc(sizeof(float)*numClustersOrig);
    minYCPU = (float*) malloc(sizeof(float)*numClustersOrig);
    maxYCPU = (float*) malloc(sizeof(float)*numClustersOrig);
    minZCPU = (float*) malloc(sizeof(float)*numClustersOrig);
    maxZCPU = (float*) malloc(sizeof(float)*numClustersOrig);
    cudaMemcpy(minXCPU, minX, sizeof(float)*numClustersOrig, cudaMemcpyDeviceToHost);
    cudaMemcpy(maxXCPU, maxX, sizeof(float)*numClustersOrig, cudaMemcpyDeviceToHost);
    cudaMemcpy(minYCPU, minY, sizeof(float)*numClustersOrig, cudaMemcpyDeviceToHost);
    cudaMemcpy(maxYCPU, maxY, sizeof(float)*numClustersOrig, cudaMemcpyDeviceToHost);
    cudaMemcpy(minZCPU, minZ, sizeof(float)*numClustersOrig, cudaMemcpyDeviceToHost);
    cudaMemcpy(maxZCPU, maxZ, sizeof(float)*numClustersOrig, cudaMemcpyDeviceToHost);

    int* leftBearing;
    int* rightBearing;
    int* leftCPU;
    int* rightCPU; 

    leftCPU = (int*) malloc(sizeof(int));
    rightCPU = (int*) malloc(sizeof(int));

    cudaMalloc(&leftBearing, sizeof(float));
    cudaMalloc(&rightBearing, sizeof(float));
    
    //Laucnh kernels to find clear paths using mins and max cluster arrasy
    findClearPathKernel<<<1, MAX_THREADS>>>(minX, maxX, minZ, maxZ, numClustersOrig, leftBearing, rightBearing);
    checkStatus(cudaGetLastError());
    cudaDeviceSynchronize();

    findAngleOffCenterKernel<<<1, MAX_THREADS>>>(minX, maxX, minZ, maxZ, numClustersOrig, leftBearing, 0);
    checkStatus(cudaGetLastError());
    cudaDeviceSynchronize();

    findAngleOffCenterKernel<<<1, MAX_THREADS>>>(minX, maxX, minZ, maxZ, numClustersOrig, rightBearing, 1);    
    checkStatus(cudaGetLastError());
    cudaDeviceSynchronize();
    
    //Copy bearings to CPU and display the bearings
    cudaMemcpy(leftCPU, leftBearing, sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(rightCPU, rightBearing, sizeof(int), cudaMemcpyDeviceToHost);
    
    //Set CPU bearings for viewer use
    bearingRight = *rightCPU;
    bearingLeft = *leftCPU;
    
    cudaFree(leftBearing);
    cudaFree(rightBearing);
    free(leftCPU);
    free(rightCPU);
    
    checkStatus(cudaDeviceSynchronize()); //not needed?
    cudaFree(neighborLists);
    cudaFree(minX);
    cudaFree(maxX);
    cudaFree(minY);
    cudaFree(maxY);
    cudaFree(minZ);
    cudaFree(maxZ);

    int validClustersCPU;
    cudaMemcpy(&validClustersCPU, validClustersCount, sizeof(int), cudaMemcpyDeviceToHost);
    std::cout << "valid cluster size: " << validClustersCPU << std::endl;

    ObsReturn obsReturn;
    obsReturn.size = numClustersOrig;
    //copy over elements so we can free the memory later
    Obstacle add;
    for (size_t i = 0; i < numClustersOrig; ++i) {
        //build an obstacle to add to the vector
        add.minX = minXCPU[i];
        add.maxX = maxXCPU[i];
        add.minY = minYCPU[i];
        add.maxY = maxYCPU[i];
        add.minZ = minZCPU[i];
        add.maxZ = maxZCPU[i];

        obsReturn.obs.push_back(add);
    }

    //free memory
    free(minXCPU);
    free(maxXCPU);
    free(minYCPU);
    free(maxYCPU);
    free(minZCPU);
    free(maxZCPU);
    /*
    * obsReturn has a vector of obstacles(named obs),
    which have the min and max of each axis as data members
    */
    return obsReturn;
}
