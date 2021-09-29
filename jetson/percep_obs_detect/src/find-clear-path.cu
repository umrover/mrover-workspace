#include "find-clear-path.hpp"

int rovWidth; //TODO, get actual rovWidth 

int bearingNum = 1024; 

struct Obstacle {
  float minX;
  float maxX;
  float minY;
  float maxY;
  float minZ; 
  float maxZ; 
}

struct ObsReturn {
  int size =0; 
  std::vector<Obstacle> obs; 
}


//Default Ctor
__device__ FindClearPath::BearingLines::BearingLines() {
  heading = 0;
  n.x = -1;
  n.y = 0;
  bLeft.x = -1;
  bLeft.y = 0;
  bRight.x = 1;
  bRight.y = 0;
}


//Ctor with specified heading
__device__ FindClearPath::BearingLines::BearingLines(float heading_in) : heading{heading_in} {
  //NB: Defines heading = 0 as straight, heading > 0 right, heading < 0 left
  n.x = -cos(heading_in);
  n.y = sin(heading_in);
  bLeft.x = (-rovWidth/2) * cos(heading_in);
  bLeft.y = (rovWidth/2) * sin(heading_in);
  bRight.x = (rovWidth/2) * cos(heading_in);
  bRight.y = (-rovWidth/2) * sin(heading_in);
}

void FindClearPath::find_clear_path_initiate(ObsReturn obsVec){
  Obstacle* gpuObstacles; 
  cudaMalloc(&gpuObstacles, obsVec.obs.size());
  cudaMemcpy(gpuObstacles, &obsVec[0], obsVec.obs.size(), cudaMemcpyHostToDevice);
  
  BearingLines* bearingsArr;
  cudaMalloc(&bearingsArr, bearingNum);

  find_clear_path(gpuObstacles, bearingsArr);

  cudaFree(gpuObstacles);
  cudaFree(bearingsArr);
}

__global__ bool* FindClearPath::find_clear_path(Obstacle* obstacles, BearingLines* bearings){
  

}
