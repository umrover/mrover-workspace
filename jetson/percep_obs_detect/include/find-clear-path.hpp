#include "common.hpp"
#include <float.h>
#include <stdlib.h>
#include <unistd.h>
#include "euclidean-cluster.hpp"
//#include <cudamath>

//TODO: where to put these constants in the final code

static const float rovWidth = 1500; //TODO, get actual rovWidth 

static const int bearingNum = 1024; 

static const float fov = 80; //Degree fov from straight ahead (0 degrees)

class BearingLines{
      public:
        float heading; //Each bearingline defined by a heading
        float2 n; //Normal vector to bearingline
        float2 bLeft; //Left offset
        float2 bRight; //Right offset

        //Default Ctor
        __device__ BearingLines();

        //Ctor with heading in
        __device__ BearingLines(float heading_in);
};

class FindClearPath {
  public:
    //int rovWidth; //Width of rover TODO: get actual size
    //int bearingNum; //Number of bearings, tentatively 1024 (Max threadcount)

    //Default Ctor
    FindClearPath();

    //Allocate host and device memory
    void find_clear_path_initiate(EuclideanClusterExtractor::ObsReturn obsVec);

    //Run find clear 
    //void find_clear_path(Obstacle* obstacles, bool* heading_checks, int obsArrSize);

    //Find closest bearing to the left that is clear
    float find_left_closest(bool* headings);
    
    //Find closest bearing to the right that is clear
    float find_right_closest(bool* headings);

};

//Kernel for find clear path parallelization 
__global__ void find_clear_path(EuclideanClusterExtractor::Obstacle* obstacles, bool* heading_checks, int obsArrSize);