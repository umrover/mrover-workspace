
#include </usr/local/zed/include/sl/Camera.hpp>
#include <vector>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/fill.h>
#include <algorithm>
#include <iostream>


#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/fill.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/sequence.h>
#include <stdlib.h>
#include <cmath>
#include <limits>

// START COMMON IMPLEMENTATION


#define CHANNEL 4
#define BLOCK_SIZE 512
#define PI 3.141592
#define HALF_ROVER 584
#define VIEWER_BGR_COLOR 2.14804915479e-38
// Temporary #define we can use until voxel grid is fully implemented
#define VOXEL 0
#define MAX_THREADS 1024

struct float4k {
	float x;
	float y;
	float z;
	float a;
	float operator[](int g) {
	if (g == 0)
		return x;
else if (g==1)
	return y;
else if (g==2)
	return z;
else 
	return a;
}
};

/**
 * \struct GPU_Cloud_F4
 * \brief GPU point cloud struct that can be passed to cuda kernels and represents a point cloud
 */
struct GPU_Cloud_F4 {
    float4k data;
    int size;
};


/**
 * \enum Axis
 * \brief Enum for x,y,z axis
 */
enum class Axis {X, Y, Z};

/**
 * \class CompareFloat4
 * \brief Functor that compares Float4 values
*/
class CompareFloat4 : public thrust::binary_function<bool,sl::float4,sl::float4> {
public: 

    /**
     * \brief CompareFloat4 construct
     * \param axis axis to compare
     */
    CompareFloat4(Axis axisIn) : axis{axisIn} {}

    /**
     * \brief overloaded operator for comparing if lhs < rhs on given axis
     * \param lhs: float to compare
     * \param rhs: float to compare
     * \return bool
     */
    __host__ __device__ bool operator() (sl::float4 lhs, sl::float4 rhs) {
        
        switch (axis) {
        
            case Axis::X :
                return lhs.x < rhs.x;
        
            case Axis::Y :
                return lhs.y < rhs.y;
        
            case Axis::Z :
                return lhs.z < rhs.z;
        }
    };

private:

    Axis axis;

};

/**
 * \struct bins
 * \brief struct containing bin info
 */
struct Bins {
    int* data;
    int size;
    int partition;
    float partitionLength;
};

/**
 * \brief Returns true if a cuda error occured and prints an error message
 */
bool checkStatus(cudaError_t status);

/**
 * \brief ceiling division x/y. e.g. ceilDiv(3,2) -> 2
 */
int ceilDiv(int x, int y);

/**
 * \brief Get a CUDA workable gpu point cloud struct from Zed GPU cloud
 */
GPU_Cloud_F4 getRawCloud(sl::Mat zed_cloud);

/**
 * \brief Crete an empty GPU Cloud of given size
 */
GPU_Cloud_F4 createCloud(int size);

/**
 * \brief Copys one GPU cloud's data to another
 */
void copyCloud(GPU_Cloud_F4 &to, GPU_Cloud_F4 &from);

/**
 * \brief Iterate through the unused memory in a GPU cloud and set it to zeros
 */
void clearStale(GPU_Cloud_F4 &cloud, int maxSize);

__global__ void findClearPathKernel(float* minXG, float* maxXG, float* minZG, float* maxZ, int numClusters, int* leftBearing, int* rightBearing);

__global__ void findAngleOffCenterKernel(float* minXG, float* maxXG, float* minZG, float* maxZ, int numClusters, int* bearing, int direction);

__device__ float atomicMinFloat (float* addr, float value);

__device__ float atomicMaxFloat (float* addr, float value);

/**
 * \brief function that given required info will hash point to bin based on coords
 * \param data: float4 with x,y,z data of a point
 * \param extrema: array with pairs of maxes and mins of each axis
 * \param partitions: number of divisions on each axis
 * \return int containing the bin number a point hashed to
 */
__device__ __forceinline__ int hashToBin (sl::float4 &data, std::pair<float,float>* extrema, int partitions) {
    int cpx = (data.x-extrema[0].first)/(extrema[0].second-extrema[0].first)*partitions;
    int cpy = (data.y-extrema[1].first)/(extrema[1].second-extrema[1].first)*partitions;
    int cpz = (data.z-extrema[2].first)/(extrema[2].second-extrema[2].first)*partitions;
    return cpx*partitions*partitions+cpy*partitions+cpz;
}


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
/*
GPU_Cloud_F4 getRawCloud(sl::Mat zed_cloud) {
    GPU_Cloud_F4 g;
    g.data = zed_cloud.getPtr<sl::float4>(sl::MEM::GPU);
    g.size = zed_cloud.getWidth() * zed_cloud.getHeight();
    return g;
}

GPU_Cloud_F4 createCloud(int size) {
    GPU_Cloud_F4 g;
    cudaMalloc(&g.data, sizeof(sl::float4)*size);
    g.size = size;
    return g;
}

*/
__global__ void copyKernel(GPU_Cloud_F4 to, GPU_Cloud_F4 from) {
    int pointIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if(pointIdx >= from.size) return;
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

void copyCloud(GPU_Cloud_F4 &to, GPU_Cloud_F4 &from) {
    to.size = from.size;
    copyKernel<<<ceilDiv(from.size, MAX_THREADS), MAX_THREADS>>>(to, from);
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




// END COMMON IMPLEMENTATION
// START PASS THROUGH IMPLEMENTATION


using namespace std;

/** 
 * \class PassThrough
 * \brief Filters out points that are less than a certain coordinate or greater than a certain coordinate on a particular cartesian axis
 */
class PassThrough {

    public:

        /**
         * \brief PassThrough constructor 
         * \param axis Either 'x', 'y', or 'z', this is the axis we filter on
         * \param min The minimum allowable coordinate of the point on the selected axis
         * \param max The maximum allowable coordinate of the point on the selected axis
         */
        PassThrough(char axis, float min, float max);

        /**
         * \brief Runs the pass through on the given cloud
         * \param cloud Point cloud to be filtered
         */    
        void run(GPU_Cloud_F4 &cloud);

    private:

        float min;
        float max;

        char axis;

};


PassThrough::PassThrough(char axis, float min, float max) : min{min}, max{max}, axis(axis){};


//Functor predicate to check if a point is within some min and max bounds on a particular axis
class WithinBounds {
    public:
        WithinBounds(float min, float max, char axis) : min(min), max(max), axis(axis) {}

        __host__ __device__ bool operator()(const sl::float4 val) {
            float test;
            if(axis == 'z') test = val.z;
            else if(axis == 'y') test = val.y;
            return test > min && test < max;
        }

    private:
        float min, max;
        char axis;
};

//Execute pass through
void PassThrough::run(GPU_Cloud_F4 &cloud){
    if(cloud.size == 0) return;
    
    //Instansiate a predicate functor and copy the contents of the cloud
    WithinBounds pred(min, max, axis);
    thrust::device_vector<float4k> buffer(cloud.data, cloud.data+cloud.size);
	checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());
    //Copy from the temp buffer back into the cloud only the points that pass the predicate 
    float4k* end = thrust::copy_if(thrust::device, buffer.begin(), buffer.end(), cloud.data, pred);
checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());
    //Clear the remainder of the cloud of points that failed pass through
    thrust::fill(thrust::device, end, cloud.data+cloud.size, float4k(0, 0, 0, 0));
checkStatus(cudaGetLastError());
    checkStatus(cudaDeviceSynchronize());
    //update the cloud size
    cloud.size = end - cloud.data;
}



// END PASSTHROUGH IMPLEMENTATION

int main(int argc, char *argv[]) {



int size = 4;
    float4k testCPU[size] = {
        {1,-3,-2,4},
        {2,2,2,4},
        {0,0,0,4},
        {0,0,0,1}
    };
    
    //GPU_Cloud_F4 testCPUpc {
      //  testCPU, size
    //};
    printf("Hello\n");
    float4k testGPU;
    cudaMalloc(&testGPU, sizeof(float4k)*size);
    cudaMemcpy(testGPU, testCPU, sizeof(float4)*size, cudaMemcpyHostToDevice);
printf("I'm getting dumber\n");
    //GPU_Cloud_F4 testPC = { testGPU, size};
//PassThrough pass('y',200,7000);
//pass.run(testPC);



    //    thrust::copy(   thrust::host,
      //                  hvec.begin(),
        //                hvec.begin() + num_elements,
          //              fish.begin());
    return 0;
}

