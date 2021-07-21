#include "plane-ransac.hpp"
#include "common.hpp"
#include <stdlib.h>
#include<unistd.h>

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

    // get the two vectors on the plane defined by the model points
    float3 v1 (modelPt1 - modelPt0);
    float3 v2 (modelPt2 - modelPt0);
    
    //get a vector normal to the plane model
    // TODO-MGM check this
    float3 n = cross(v1, v2);

    //add this constraint later
    //check that n dot desired axis is less than epsilon, if so, return here 
    if(abs(dot(normalize(n), normalize(axis))) < epsilon) {
        //if(threadIdx.x == 0) printf("eliminating model for axis tolerance failure %d \n", iteration);
        if(threadIdx.x == 0) inlierCounts[iteration] = 0; //make it -1 to show invalid model?
        return;
    }


    // figure out how many points each thread must compute distance for and determine if each is inlier/outlier
    int pointsPerThread = ceilDivGPU(pc.size, MAX_THREADS);
    for(int i = 0; i < pointsPerThread; i++) {
        // select a point index or return if this isn't a valid point
        int pointIdx = threadIdx.x * pointsPerThread + i;
        if(pointIdx >= pc.size) continue; //TODO Should this be return??? 
        
        // point in the point cloud that could be an inlier or outlier
        float3 curPt = make_float3(pc.data[pointIdx].x, pc.data[pointIdx].y, pc.data[pointIdx].z);
        if(curPt.x == 0 && curPt.y == 0 && curPt.z == 0) continue; //TEMPORARY (0,0,0 removal) until passthru
        
        //calculate distance of cur pt to the plane formed by the 3 model points [see doc for the complete derrivation]
        float3 d_to_model_pt = (curPt - modelPt1);
        
        // float d = abs(float3(n, d_to_model_pt)) / n.norm();
        float d = abs( dot( normalize(n), d_to_model_pt));
        
        //add a 0 if inlier, 1 if not 
        inliers += (d < threshold) ? 1 : 0; //very probalmatic line, how can we reduce these checks
        //inliers += (-1*abs(d - threshold)/(d - threshold) + 1 )/2;
    }
    
    //parallel reduction to get an aggregate sum of the number of inliers for this model
    //this is all equivalent to sum(inlierField), but it does it in parallel
    inlierField[threadIdx.x] = inliers;
    __syncthreads();
    int aliveThreads = (blockDim.x) / 2;
	while (aliveThreads > 0) {
		if (threadIdx.x < aliveThreads) {
            inliers += inlierField[aliveThreads + threadIdx.x];
            //if(iteration == 0 && threadIdx.x == 0) printf("t0: %f \n", inliers);
			if (threadIdx.x >= (aliveThreads) / 2) inlierField[threadIdx.x] = inliers;
		}
		__syncthreads();
		aliveThreads /= 2;
	}

    //at the final thread, write to global memory
    if(threadIdx.x == 0) {
        inlierCounts[iteration] = inliers;
        //printf("iteration %d, inlier ct: %f == %f \n", iteration, inlierCounts[iteration], inliers);
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
    //PROBLEM: This can easily index out of bounds if threadIdx.x > numPoints in the PC
    //another problem:  we must initalize the inlierCountsLocal with low valeus that wont be chosen
    float inliers = (threadIdx.x < iterations) ? inlierCounts[threadIdx.x] : 0;
    int optimalModel = threadIdx.x;
    inlierCountsLocal[threadIdx.x] = inliers;
    modelIndiciesLocal[threadIdx.x] = optimalModel;
    __syncthreads();

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
       // printf("--> model with most inliers is model: %d \n", modelIndiciesLocal[0]);
       // pc.data[ modelPoints[modelIndiciesLocal[0]*3 + threadIdx.x]].w =  9.14767637511e-41;//; //debug featre color model pt

        float3 pt = make_float3(pc.data[ modelPoints[modelIndiciesLocal[0]*3 + threadIdx.x] ].x, pc.data[ modelPoints[modelIndiciesLocal[0]*3 + threadIdx.x] ].y, pc.data[ modelPoints[modelIndiciesLocal[0]*3 + threadIdx.x] ].z);

        optimalModelOut[threadIdx.x].x = pt.x; 
        optimalModelOut[threadIdx.x].y = pt.y; 
        optimalModelOut[threadIdx.x].z = pt.z;
        printf("Optimal Model Set: %f, %f, %f\n", pt.x, pt.y, pt.z);
    } 
    if(threadIdx.x == 0) {
        // Find normal to the plane
        optimalModelOut.ComputeNormal();

        printf("winner model inliers: %f \n", inlierCountsLocal[0]);
        //check here if the inlier counts local is 0, if so return -1 instead
        if(inlierCountsLocal[0] > 1.0) {
            *optimalModelIndex = modelIndiciesLocal[0];
        } else *optimalModelIndex = -1;
    }
}

// this kernel is for DEBUGGING only. It will get the list of inliers so they can 
// be displayed. In competition, it is not necessary to have this information. 

__global__ void computeInliers(GPU_Cloud pc , int* optimalModelIndex, int* modelPoints, float threshold, float3 axis) {
    if(*optimalModelIndex < 0) return;


    float3 modelPt0 = make_float3(pc.data[modelPoints[*optimalModelIndex*3]].x, pc.data[modelPoints[*optimalModelIndex*3]].y, pc.data[modelPoints[*optimalModelIndex*3]].z);
    float3 modelPt1 = make_float3(pc.data[modelPoints[*optimalModelIndex*3 + 1]].x, pc.data[modelPoints[*optimalModelIndex*3 + 1]].y, pc.data[modelPoints[*optimalModelIndex*3 + 1]].z);
    float3 modelPt2 = make_float3(pc.data[modelPoints[*optimalModelIndex*3 + 2]].x, pc.data[modelPoints[*optimalModelIndex*3 + 2]].y, pc.data[modelPoints[*optimalModelIndex*3 + 2]].z);

    // get the two vectors on the plane defined by the model points
    float3 v1 (modelPt1 - modelPt0);
    float3 v2 (modelPt2 - modelPt0);
    
    //get a vector normal to the plane model
    float3 n = cross(v1, v2);


/*
        printf("mp0: %f %f %f and mp1: %f %f %f \n", modelPt0.x, modelPt0.y, modelPt0.z, modelPt1.x, modelPt1.y, modelPt1.z);

        printf("v1: %f %f %f and v2: %f %f %f \n", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
    
        printf("normal %f %f %f --- axis %f %f %f \n", n.x, n.y, n.z, axis.x, axis.y, axis.z);
    
       printf("axis normal dot: %f \n", abs(float3::dot(n/n.norm(), axis/axis.norm())));
  
*/
    __syncthreads();

    // figure out how many points each thread must compute distance for and determine if each is inlier/outlier
    int pointsPerThread = ceilDivGPU(pc.size, MAX_THREADS);
    for(int i = 0; i < pointsPerThread; i++) {
        // select a point index or return if this isn't a valid point
        int pointIdx = threadIdx.x * pointsPerThread + i;
        if(pointIdx >= pc.size) return; 
        
        // point in the point cloud that could be an inlier or outlier
        float3 curPt = make_float3(pc.data[pointIdx].x, pc.data[pointIdx].y, pc.data[pointIdx].z);
        
        //calculate distance of cur pt to the plane formed by the 3 model points [see doc for the complete derrivation]
        float3 d_to_model_pt = (curPt - modelPt1);
        
        // TODO-MGM check this float d = abs(dot(n, d_to_model_pt)) / n.norm();
        float d = abs( dot( normalize(n), d_to_model_pt));

        //add a 0 if inlier, 1 if not 
        int flag = (d < threshold) ? 1 : 0; //very probalmatic line, how can we reduce these checks
        //int flag = (-1*abs(d - threshold)/(d - threshold) + 1 )/2;
        
        if(flag != 0) { //colors are ABGR in float space(LOL?????)
            pc.data[pointIdx].w = 100;
        } else {
            uint32_t* color_uint = (uint32_t *) & pc.data[pointIdx].w;
            unsigned char* abgr = (unsigned char*) color_uint;
        }

        
    }

    //*debug color model points 3.57331108403e-43; // (red)
    __syncthreads();
    if(threadIdx.x == 0) {
        pc.data[modelPoints[*optimalModelIndex*3]].w = 3.57331108403e-43;
        pc.data[modelPoints[*optimalModelIndex*3 + 1]].w = 3.57331108403e-43;
        pc.data[modelPoints[*optimalModelIndex*3 + 2]].w = 3.57331108403e-43;

        //printf("optimal idx: %d %d %d \n",  modelPoints[*optimalModelIndex*3],  modelPoints[*optimalModelIndex*3+1], modelPoints[*optimalModelIndex*3+2]);
        //printf("> pt0: %f %f %f \n", modelPt0.x, modelPt0.y, modelPt0.z );
   //     printf("> pt1: %f %f %f \n", modelPt1.x, modelPt1.y, modelPt1.z );
     //   printf("> pt2: %f %f %f \n", modelPt2.x, modelPt2.y, modelPt2.z );


    }
}
/*
void removeInliers(GPU_Cloud pc, GPU_Cloud out, int* optimalModelIndex, int* modelPoints, float threshold, float3 axis, int* newSize) {
    int pointIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if(*optimalModelIndex < 0) {
        if(pointIdx < pc.size) {
            out.data[pointIdx] = pc.data[pointIdx];
            *newSize = pc.size;
        }
        return;
    }

    if(pointIdx >= pc.size) return;
    
    if(pointIdx < pc.size) pc.data[pointIdx].w = 2.34184088514e-38;
    *newSize = pc.size; //pc.size/2;
    return;

    float4 datum = {0,0,0,0};
    int newIdx = pc.size-1;
    if (pointIdx == -1)
    {
        printf("Model points: {%f, %f, %f}\n", pc.data[modelPoints[*optimalModelIndex*3]].x, pc.data[modelPoints[*optimalModelIndex*3]].y, pc.data[modelPoints[*optimalModelIndex*3]].z);

        printf("Model points: {%f, %f, %f}\n",pc.data[modelPoints[*optimalModelIndex*3+1]].x, pc.data[modelPoints[*optimalModelIndex*3+1]].y, pc.data[modelPoints[*optimalModelIndex*3+1]].z);

        printf("Model points: {%f, %f, %f}\n", pc.data[modelPoints[*optimalModelIndex*3+2]].x, pc.data[modelPoints[*optimalModelIndex*3+2]].y, pc.data[modelPoints[*optimalModelIndex*3+2]].z);
    }

    if (pointIdx == 0)
    {
        printf("Size: %i\n", pc.size);
    }
    __syncthreads();

    if(pointIdx < pc.size) {
        float3 modelPt0 = make_float3(pc.data[modelPoints[*optimalModelIndex*3]].x, pc.data[modelPoints[*optimalModelIndex*3]].y, pc.data[modelPoints[*optimalModelIndex*3]].z);
        float3 modelPt1 = make_float3(pc.data[modelPoints[*optimalModelIndex*3 + 1]].x, pc.data[modelPoints[*optimalModelIndex*3 + 1]].y, pc.data[modelPoints[*optimalModelIndex*3 + 1]].z);
        float3 modelPt2 = make_float3(pc.data[modelPoints[*optimalModelIndex*3 + 2]].x, pc.data[modelPoints[*optimalModelIndex*3 + 2]].y, pc.data[modelPoints[*optimalModelIndex*3 + 2]].z);

        // get the two vectors on the plane defined by the model points
        float3 v1 (modelPt1 - modelPt0);
        float3 v2 (modelPt2 - modelPt0);
        
        //get a vector normal to the plane model
        float3 n = cross(v1, v2);
    
        // point in the point cloud that could be an inlier or outlier
        float3 curPt = make_float3(pc.data[pointIdx].x, pc.data[pointIdx].y, pc.data[pointIdx].z);
        
        //calculate distance of cur pt to the plane formed by the 3 model points [see doc for the complete derrivation]
        float3 d_to_model_pt = (curPt - modelPt1);
        
        // float d = abs(dot(n, d_to_model_pt)) / n.norm();
        float d = abs( dot( normalize(n), d_to_model_pt));

        //add a 0 if inlier, 1 if not 
        int flag = (d < threshold) ? 1 : 0; //very probalmatic line, how can we reduce these checks
        //int flag = (-1*abs(d - threshold)/(d - threshold) + 1 )/2;
        printf("Flag %i and d %f\n", flag, d);
        if(flag != 0) { //colors are ABGR in float space(LOL?????)
            pc.data[pointIdx].w = 2.35098856151e-38; //VIEWER_BGR_COLOR;
           // pc.data[pointIdx].x = 0;
           // pc.data[pointIdx].y = 0;
           // pc.data[pointIdx].z = 0;
        } else {
            //pc.data[pointIdx].w =  9.18340948595e-41;
            
            datum = pc.data[pointIdx];
            newIdx = atomicAdd(newSize, 1);
        }
    }

    out.data[newIdx] = datum;
    if (pointIdx == 0) printf("Point: {%f, %f, %f}\n", datum.x, datum.y, datum.z);
    __syncthreads();
}
*/
RansacPlane::RansacPlane(float3 axis, float epsilon, int iterations, float threshold, int pcSize, float removalRadius)
: pc(pc), axis(axis), epsilon(epsilon), iterations(iterations), threshold(threshold), removalRadius(removalRadius)  {
    //Set up buffers needed for RANSAC
    cudaMalloc(&inlierCounts, sizeof(float) * iterations); 
    cudaMalloc(&modelPoints, sizeof(int) * iterations * 3);
    cudaMalloc(&selection, sizeof(Plane));
    cudaMalloc(&optimalModelIndex, sizeof(int));

    std::cout << "RANSAC Constructor, random indicies: " << std::endl;

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
        
       // std::cout << a <<  ", " << b << ", " << c << std::endl;
    }
    /*
    randomNumsCPU[0] = 2;
    randomNumsCPU[1] = 8;
    randomNumsCPU[2] = 1;
    */

    std::cout << std::endl << std::endl;

    cudaMemcpy(modelPoints, randomNumsCPU, sizeof(int) * iterations * 3, cudaMemcpyHostToDevice);
    free(randomNumsCPU);

    //Generate a buffer for retreiving the selected model from CUDA Kernels
    selectedModel = (Plane*) malloc(sizeof(Plane)); 


}

/*  
EFFECTS:
    1. [GPU] Use the RANSAC kernel to evaluate all the canidate models and report their associated inlier count
    2. [GPU] Select the canidate with the highest score and inform the CPU
    3. [CPU] Use the three model points to produce a plane equation in standard form and return to the user
*/
Plane RansacPlane::computeModel(GPU_Cloud pc) {
    this->pc = pc;

    int blocks = iterations;
    int threads = MAX_THREADS;
    std::cout<<"Doing more computing\n";
    ransacKernel<<<blocks, threads>>>(pc, inlierCounts, modelPoints, threshold, axis, cos(epsilon*3.1415/180));
    std::cout<<"Computing model\n";
    selectOptimalRansacModel<<<1, MAX_THREADS>>>(pc, inlierCounts, modelPoints, *selection, iterations, optimalModelIndex);
    std::cout << "Model selected\n";
    computeInliers<<<1, threads>>>(pc, optimalModelIndex, modelPoints, removalRadius, axis);
    std::cout << "Computing finished\n";
    //might be able to use memcpyAsync() here, double check
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());

    std::cout << "finishing ransac " << pc.size << std::endl;

    cudaMemcpy(selectedModel, selection, sizeof(Plane), cudaMemcpyDeviceToHost);
    //for(int i = 0; i < 3; i++) {
    //    cout << "model " << i << ":" << selectedModel[0] << selected 
    //}
    Plane plane = *selectedModel;
    
    return plane;
}

Plane RansacPlane::computeModel(GPU_Cloud &pc, bool flag) {
    if(pc.size == 0) return {make_float3(0,0,0), make_float3(0,0,0), make_float3(0,0,0)};

    // GPU_Cloud tmpCloud; //exp
    // tmpCloud.size = pc.size;
    // std::cout << "Size: " << tmpCloud.size * sizeof(float4) << "\n";
    // cudaMalloc(&tmpCloud.data, tmpCloud.size * sizeof(float4));
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());
    std::cout << "Malloced\n";
    // usleep(1000000);
    this->pc = pc;
    int blocks = iterations;
    int threads = MAX_THREADS;
    ransacKernel<<<blocks, threads>>>(pc, inlierCounts, modelPoints, threshold, axis, cos(epsilon*3.1415/180));
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());
    std::cout << "ransacKernel\n";
    selectOptimalRansacModel<<<1, MAX_THREADS>>>(pc, inlierCounts, modelPoints, *selection, iterations, optimalModelIndex);
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());   
    std::cout <<"optimalRansacModel " << "\n";
    int* size;
    cudaMalloc(&size, sizeof(int));
    cudaMemset(size, 0, sizeof(int));
 
    /*
    std::cout << "all should be good up to here\n";
    removeInliers<<<ceilDiv(pc.size, MAX_THREADS), MAX_THREADS>>>(pc, tmpCloud, optimalModelIndex, modelPoints, removalRadius, axis, size);
    checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());
   // while(true){}
    std::cout << "I'm expecting issues\n";
    int sizeCpu;
    cudaMemcpy(&sizeCpu, size, sizeof(int), cudaMemcpyDeviceToHost);
    std::cout << "Size after removal " << sizeCpu << "\n";
    */
    //tmpCloud.size = sizeCpu;
    // if(sizeCpu > 0) copyCloud(pc, tmpCloud);
    // std::cout << "Copy cloud called\n";
    // cudaFree(tmpCloud.data); //exp
    // std:: cout << "Freed stuff\n";
   // pc.size = 320/2*180/2;
    //std::cout << sizeCpu << std::endl;
    //might be able to u
    //cout << "b4: " << pc.size << endl;

    // Remove points lying in the plane
    cudaMemcpy(selectedModel, selection, sizeof(Plane), cudaMemcpyDeviceToHost);
    InPlane predicate(selectedModel->normal, threshold); 
    Filter<InPlane>(pc, predicate);

    //for(int i = 0; i < 3; i++) {
    //    cout << "model " << i << ":" << selectedModel[0] << selected 
    //}
     //std::cout << "Surprised as usual\n";

    Plane plane = *selectedModel;
    //cudaFree(size);
    std::cout << "cleaned things up\n";
    return plane;
}

RansacPlane::~RansacPlane() {
    cudaFree(inlierCounts);
    cudaFree(modelPoints);
    cudaFree(selection);
    cudaFree(optimalModelIndex);
    free(selectedModel);
}