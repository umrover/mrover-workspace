#include "common.hpp"
#include <float.h>
#include <stdlib.h>
#include <unistd.h>
//#include <cudamath>



struct Obstacle {
  float minX;
  float maxX;
  float minY;
  float maxY;
  float minZ; 
  float maxZ; 
};

struct ObsReturn {
  int size =0; 
  std::vector<Obstacle> obs; 
};


class FindClearPath {
  public:
    //int rovWidth; //Width of rover TODO: get actual size
    //int bearingNum; //Number of bearings, tentatively 1024 (Max threadcount)

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

    //Allocate host and device memory
    void find_clear_path_initiate(ObsReturn obsVec);

    //Run find clear 
    void find_clear_path(Obstacle* obstacles, bool* heading_checks, int obsArrSize);

    int find_closest_clear_path(bool* headings);

    static const int rovWidth = 10; //TODO, get actual rovWidth 

    static const int bearingNum = 1024; 

};