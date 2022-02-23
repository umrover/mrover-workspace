#include <sl/Camera.hpp>
#include <thrust/functional.h>
#include "helper_math.h"
#include <cstdlib>
#include <cmath>
#include <limits>
#include "rapidjson/document.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#ifdef WITH_JARVIS
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <lcm/lcm-cpp.hpp>
#endif

#ifndef COMMON
#define COMMON

#define CHANNEL 4
#define BLOCK_SIZE 512
#define PI 3.141592
#define HALF_ROVER 584
#define VIEWER_BGR_COLOR 2.14804915479e-38
// Temporary #define we can use until voxel grid is fully implemented
// TODO: remove this
#define VOXEL 1
#define MAX_THREADS 1024

/**
 * \enum Axis
 * \brief Enum for x,y,z axis
 */
enum class Axis {
    X, Y, Z
};

/*
 *** Set up debugging level ***
 */
enum class OperationMode {
    DEBUG, SILENT
};

/**
 * \struct GPU_Cloud_F4
 * \brief GPU point cloud struct that can be passed to cuda kernels and represents a point cloud
 */
struct GPU_Cloud {
    float4* data;
    int size;
};

/**
 * \class CompareFloat4
 * \brief Functor that compares Float4 values
*/
class CompareFloat4 : public thrust::binary_function<bool, float4, float4> {
    public:

        /**
         * \brief CompareFloat4 construct
         * \param axis axis to compare
         */
        explicit CompareFloat4(Axis axisIn) : axis{axisIn} {}

        /**
         * \brief overloaded operator for comparing if lhs < rhs on given axis
         * \param lhs: float to compare
         * \param rhs: float to compare
         * \return bool
         */
        __host__ __device__ bool operator()(float4 lhs, float4 rhs) {
            switch (axis) {
                case Axis::X :
                    return lhs.x < rhs.x;
                case Axis::Y :
                    return lhs.y < rhs.y;
                case Axis::Z :
                    return lhs.z < rhs.z;
            }
            assert(false);
            return false;
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
void getRawCloud(GPU_Cloud& pc, sl::Mat& zed_cloud);

/**
 * \brief Copies one GPU cloud's data to another
 */
void copyCloud(GPU_Cloud& to, GPU_Cloud& from);

GPU_Cloud createCloud(int size);

void deleteCloud(GPU_Cloud& cloud);

OperationMode parse_operation_mode(const rapidjson::Document& mRoverConfig);

/**
 * \brief Iterate through the unused memory in a GPU cloud and set it to zeros
 */
void clearStale(GPU_Cloud& cloud, int maxSize);

__global__ void findClearPathKernel(float* minXG, float* maxXG, float* minZG, float* maxZ, int numClusters, int* leftBearing, int* rightBearing);

__global__ void findAngleOffCenterKernel(float* minXG, float* maxXG, float* minZG, float* maxZ, int numClusters, int* bearing, int direction);

__device__ float atomicMinFloat(float* addr, float value);

__device__ float atomicMaxFloat(float* addr, float value);

#endif
