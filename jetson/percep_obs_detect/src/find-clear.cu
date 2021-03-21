#include "common.hpp"
#include <float.h>
//Find clear path
//Checks to see if minx and maxX are between the two projected lines
//It not checks to see if the line segment between the two minX and maxX intersect the projected line
//If such points are found launch write them to place in shared memory
//If such poitns are not found then in shared memory space write float_max or float_min
//SyncThreads and perform parallel reduction to find minX and maxX thread values
//launch left and right path kernels using new calculated lines
//Left and right path kernels will perform the same steps except in a particular direction
//Assumes we don't have over 1024 clusters

class compareLine {
public:
    int xIntercept;
    float slope;
    
    __device__ compareLine(float angle_in, int xInt_in) : xIntercept{xInt_in}, 
                        slope{tan(angle_in*PI/180)} {
                            if(slope != 0) {
                                slope = 1/slope;
                            }
                        }

    //Returns 1 if point is right of line, 0 if on, -1 if left of line
    __device__ int operator()(float x, float y) {
        
        //Make sure don't divide by 0
        float xc = xIntercept; //x calculated
        if(slope != 0) {
            xc = y/slope+xIntercept; //Find x value on line with same y value as input point
        }
            
        //Point is right of the line
        if(x > xc) {
            return 1;
        }
        //Point is on the line
        else if (x == xc) {
            return 0;
        }
        //Point is left of the line
        else {
            return -1;
        } 
    }

    //Assumes x1 < x2
    __device__ bool operator()(float x1, float y1, float x2, float y2) {
        if(x1 != x2){
            if(slope != 0){
                float slopeSeg = (y2-y1)/(x2-x1);
                float xIntersect = (-slopeSeg*x1+y1-xIntercept)/(slope-slopeSeg);
                return (xIntersect < x2 && xIntersect > x1);
            }
            //Check if left of line and right of line if slope is undefined
            else if(this->operator()(x1,y1) < 1 && this->operator()(x2,y2) > -1) return true; 
        }
        return false;
        
    }
};

int findNextLargestSquare(int num){
    int exp = log2(num) + 1;
    return pow(2,exp);
}

//Finds the leftmost or rightmost obstacle in a given path, calculates a new path based on where this obstacle is, 
//and then checks that path for obstacles. Direction of 1 is right and 0 is left
__global__ void findAngleOffCenterKernel(float* minXG, float* maxXG, float* minZG, float* maxZG, int numClusters, int* bearing, int direction) {
    
    if(!(*bearing)) return; //Check if bearing is 0
    if(threadIdx.x == 0) *bearing = direction ? INT_MIN : INT_MAX;
    if(threadIdx.x >= numClusters) return;

    //Copy over data from global to local
    float maxX = maxXG[threadIdx.x];
    float minX = minXG[threadIdx.x];
    float minZ = minZG[threadIdx.x];
    float maxZ = maxZG[threadIdx.x];
    
    if(maxZ>= 0.01) {
        //Each thread finds the angle off center of their obstacle and we take the rightmost or leftmost one
        int buffer = 10;
        float oppSideRTri = direction ? maxX : minX;
        float adjSideRTri = minZ;//Length of adjacent side of right triangle
        oppSideRTri += direction ? buffer+HALF_ROVER : -(buffer+HALF_ROVER); //Calculate length of opposite side of right triangle
        int angle = atan(oppSideRTri/adjSideRTri)*180/PI;//arctan(opposite/adjacent) //TODO Change this to float, but not for now, since atomicMin only owrks with ints

        //Take the farthest right or left and write to global memory
        direction ? atomicMax(bearing, angle) : atomicMin(bearing, angle);
        
        __syncthreads(); 

        if(direction == 0 && *bearing == angle) printf("Leftmost: (%.1f, %.1f) Bearing: %i\n", minX, minZ, angle);
        if(direction == 1 && *bearing == angle) printf("Rightmost: (%.1f, %.1f) Bearing: %i\n", maxX, minZ, angle);   
        
    }
    
    
}
__global__ void findClearPathKernel(float* minXG, float* maxXG, float* minZG, float* maxZG, int numClusters, int* leftBearing, int* rightBearing) {
        
    //Declare variables
    if(threadIdx.x >= numClusters) return;

    compareLine rightLine(0,HALF_ROVER); //Is there a way to make these objects shared?
    compareLine leftLine(0,-HALF_ROVER);
    __shared__ bool obstacle;

    if(threadIdx.x == 0) obstacle = false;

    //Copy over data from global to local
    float maxX = maxXG[threadIdx.x];
    float minX = minXG[threadIdx.x];
    float minZ = minZG[threadIdx.x];
    float maxZ = maxZG[threadIdx.x];

    //Check where point is relative to line
    //Since the z values of the min and max x values aren't stored we are going
    //to assume that they are both located at the min z value of the cluster

    //Checks if either of the points is between the two lines
    if(((leftLine(maxX, minZ) > -1 && rightLine(maxX, minZ) < 1) || //check if maxX is between right and left
       (leftLine(minX, minZ) > -1 && rightLine(minX, minZ) < 1) || //Check if minX is between right and left
       (leftLine(minX, minZ, maxX, minZ) || rightLine(minZ, minZ, maxX, minZ))) && !(maxZ < 0.01)) { //check if lines intersect line seg
        obstacle = true;
    }

    __syncthreads();

    //Iterate through points to find mins and maxes
    //No need for parallel reduction, since cluster size is relatively small
    
    if(threadIdx.x == 0){
        if(!obstacle) { //Path is clear
            *leftBearing = 0;
            *rightBearing = 0;
            printf("Center Path Clear\n");
        }
        else { //Path is blocked find left and right angles
            *leftBearing = -1;
            *rightBearing = -1;
        }
    }
}

/*
void testClearPath() {

    int testClusterSize = 4;
    float* minXG;
    float* maxXG;
    float* minZG;
    float* leftBearing;
    float* rightBearing;

    cudaMalloc(&minXG, sizeof(float)*testClusterSize);
    cudaMalloc(&maxXG, sizeof(float)*testClusterSize);
    cudaMalloc(&minZG, sizeof(float)*testClusterSize);
    cudaMalloc(&leftBearing, sizeof(float));
    cudaMalloc(&rightBearing, sizeof(float));

    float minXCPU[testClusterSize] = { -6, -10, 6, -100, };
    float maxXCPU[testClusterSize] = { 6, -7, 7, -9};
    float minZCPU[testClusterSize] = { 10, 10, 10, 10};

    cudaMemcpy(minXG, minXCPU, sizeof(float)*testClusterSize, cudaMemcpyHostToDevice);
    cudaMemcpy(maxXG, maxXCPU, sizeof(float)*testClusterSize, cudaMemcpyHostToDevice);
    cudaMemcpy(minZG, minZCPU, sizeof(float)*testClusterSize, cudaMemcpyHostToDevice);

    findClearPathKernel<<<1, MAX_THREADS>>>(minXG, maxXG, minZG, testClusterSize, leftBearing, rightBearing);
    
    findAngleOffCenterKernel<<<1, MAX_THREADS>>>(minXG, maxXG, minZG, testClusterSize, leftBearing, 0);
    findAngleOffCenterKernel<<<1, MAX_THREADS>>>(minXG, maxXG, minZG, testClusterSize, rightBearing, 1);    
    
    checkStatus(cudaGetLastError());
    cudaDeviceSynchronize();

    cudaFree(minXG);
    cudaFree(maxXG);
    cudaFree(minZG);
}
*/