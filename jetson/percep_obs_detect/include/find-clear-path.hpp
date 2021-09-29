#include "common.hpp"
#include <float.h>
#include <stdlib.h>
#include <unistd.h>
#include <cudamath>

class FindClearPath {
  public:
    int rovWidth;
    int bearingNum;

    class BearingLines{
      public:
        float heading;
        float2 n;
        float2 bLeft;
        float2 bRight;

        //Default Ctor
        __device__ BearingLines() {};

        //Ctor with heading in
        __device__ BearingLines(float heading_in) : heading{heading_in} {};
    };

    //Allocate host and device memory
    void find_clear_path_initiate(ObsReturn obsVec){};

    //Run find clear 
    bool* find_clear_path(){};


};