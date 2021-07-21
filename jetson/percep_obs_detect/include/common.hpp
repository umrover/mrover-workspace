#include <sl/Camera.hpp>
#include <thrust/functional.h>
#include "helper_math.h"
#include <stdlib.h>
#include <cmath>
#include <limits>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/fill.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/sequence.h>
#include <assert.h>

#ifndef COMMON
#define COMMON

#define CHANNEL 4
#define BLOCK_SIZE 512
#define PI 3.141592
#define HALF_ROVER 584
#define VIEWER_BGR_COLOR 2.14804915479e-38
// Temporary #define we can use until voxel grid is fully implemented
#define VOXEL 0

/**
 * \struct GPU_Cloud_F4
 * \brief GPU point cloud struct that can be passed to cuda kernels and represents a point cloud
 */
struct GPU_Cloud_F4 {
    sl::float4* data;
    int size;
};

/**
 * \enum Axis
 * \brief Enum for x,y,z axis
 */
enum class Axis {X, Y, Z};

/**
 * \struct GPU_Cloud_F4
 * \brief GPU point cloud struct that can be passed to cuda kernels and represents a point cloud
 */
struct GPU_Cloud {
    float4* data;
    int size;
};

/**
class float3d : public float3 {
public:  

    __host__ __device__ float3d () : float3{0, 0, 0} {}

    __host__ __device__ float3d (float a, float b, float c) : float3{a,b,c} {}
    
    __host__ __device__ float3d (float4 in) : float3{in.x, in.y, in.z} {}
    
    __host__ __device__ float3d operator- (float3d b) {
        return float3d(this->x - b.x, this->y - b.y, this->z - b.z);
    }

    __host__ __device__ static float3d cross (float3d a, float3d b) {
        return float3d(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y-a.y*b.x);
    }

    __host__ __device__ static float dot (float3d a, float3d b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    __host__ __device__ float norm() {
        return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
    }

    __host__ __device__ float3d operator/ (float a) {
        return float3d(this->x/a, this->y/a, this->z/a);
    }

};
*/

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
 * \brief Copys one GPU cloud's data to another
 */
void copyCloud(GPU_Cloud &to, GPU_Cloud &from);

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

class IPredicateFunctor {
public:
    virtual __host__ __device__ bool operator()(const float4 val) = 0;
};

template<typename T>
__host__ __device__ void Filter(GPU_Cloud &cloud, T &pred) {
    // Ensure pred is of the correct type
    assert((std::is_base_of<IPredicateFunctor, T>::value));
    
    if(cloud.size == 0) return;

    // Create thrust vector with all cloud point in it
    thrust::device_vector<float4> buffer(cloud.data, cloud.data+cloud.size);

    // Copy from the temp buffer back into the cloud only the points that pass the predicate 
    float4* end = thrust::copy_if(thrust::device, buffer.begin(), buffer.end(), cloud.data, pred);

    // Clear the remainder of the cloud of points that failed predicate
    thrust::fill(thrust::device, end, cloud.data+cloud.size, float4{0, 0, 0, 0});

    //update the cloud size
    cloud.size = end - cloud.data;
    printf("Cloud Size: %i\n", cloud.size);
}

#define MAX_THREADS 1024

#endif
