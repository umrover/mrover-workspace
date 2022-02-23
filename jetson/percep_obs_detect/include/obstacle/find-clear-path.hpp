#include "common.hpp"
#include <cfloat>
#include <cstdlib>
#include <unistd.h>
#include "euclidean-cluster.hpp"

//TODO: where to put these constants in the final code

static const float rovWidth = 1500;

static const int bearingNum = 1024;

static const float fov = 80; //Degree fov from straight ahead (0 degrees)

class BearingLines {
    public:
        float heading; //Each bearingline defined by a heading
        float2 n; //Normal vector to bearingline
        float2 bLeft; //Left offset
        float2 bRight; //Right offset

        //Default Ctor
        __device__ BearingLines();

        //Ctor with heading in
        __device__ explicit BearingLines(float heading_in);
};

class FindClearPath {
    public:

        //Default Ctor
        __host__ __device__ FindClearPath();

        //Allocate host and device memory
        float3 find_clear_path_initiate(EuclideanClusterExtractor::ObsReturn obsVec);

        //Find closest bearing to the left that is clear
        float find_left_closest(bool* headings);

        //Find closest bearing to the right that is clear
        float find_right_closest(bool* headings);

};

//Kernel for find clear path parallelization 
__global__ void find_clear_path(EuclideanClusterExtractor::Obstacle* obstacles, bool* heading_checks, float* min_dist_ptr, int obsArrSize);
