#include "find-clear-path.hpp"
#include <iostream>

//Default findClear Ctor
__device__ FindClearPath::FindClearPath(){}

//Default bearingLine Ctor
__device__ BearingLines::BearingLines() {
  heading = 0;
  n.x = -1;
  n.y = 0;
  bLeft.x = -1;
  bLeft.y = 0;
  bRight.x = 1;
  bRight.y = 0;
}


//Ctor with specified heading
__device__ BearingLines::BearingLines(float heading_in) : heading{heading_in} {
  //NB: Defines heading = 0 as straight, heading > 0 right, heading < 0 left
  n.x = -cos(heading_in); //Calculate x component of orthogonal vec from heading_in
  n.y = sin(heading_in); //Calculate y component of orthogonal vec from heading_in
  bLeft.x = (-rovWidth/2) * cos(heading_in); //Calculate bLeft x offset from heading_in
  bLeft.y = (rovWidth/2) * sin(heading_in); //Calculate bLeft y offset from heading_in
  bRight.x = (rovWidth/2) * cos(heading_in); //Calculate bRight x offset from heading_in
  bRight.y = (-rovWidth/2) * sin(heading_in); //Calculate bRight y offset from heading_in
}

void FindClearPath::find_clear_path_initiate(EuclideanClusterExtractor::ObsReturn obsVec){
  //Allocate and copy obstacle structs array
  EuclideanClusterExtractor::Obstacle* gpuObstacles; 
  cudaMalloc(&gpuObstacles, obsVec.obs.size()*sizeof(EuclideanClusterExtractor::Obstacle));
  cudaMemcpy(gpuObstacles, &obsVec.obs[0], obsVec.obs.size()*sizeof(EuclideanClusterExtractor::Obstacle), cudaMemcpyHostToDevice);
  
  //Allocate heading checks array
  bool* heading_checks;
  cudaMalloc(&heading_checks, bearingNum*sizeof(bool));

  //Run find_clear_path on each of the 1024 headings (threads)
  find_clear_path<<<1, bearingNum>>>(gpuObstacles, heading_checks, obsVec.obs.size());

  checkStatus(cudaDeviceSynchronize());

  //TODO: what to do with heading_checks array
  bool* cpu_heading_checks = new bool[bearingNum];
  cudaMemcpy(cpu_heading_checks, heading_checks, bearingNum, cudaMemcpyDeviceToHost);

  //Find closest heading to the left and right of our current heading
  int heading_left = find_left_closest(cpu_heading_checks);
  int heading_right = find_right_closest(cpu_heading_checks);

  //TODO cout in the driver
  std::cout << "left heading: " << heading_left << std::endl;
  std::cout << "right heading: " << heading_right << std::endl;

  //Free memory
  cudaFree(gpuObstacles);
  cudaFree(heading_checks);
}

__global__ void find_clear_path(EuclideanClusterExtractor::Obstacle* obstacles, bool* heading_checks, int obsArrSize){
  
  int i = threadIdx.x;
  heading_checks[i] = 1; //Assume a clear heading

  //Create bearing lines based on threadIdx
  //NB: threadIdx 511 is the 0 heading
  BearingLines bearings((fov*(i-(int)(bearingNum/2)))/(int)(bearingNum/2)); //Create bearing lines from threadIdx
  for(int j = 0; j < obsArrSize; ++j){ //Check all obstacles in obstacles array
    float detectLmin = (bearings.n.x * (obstacles[j].minX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].minY - bearings.bLeft.y));
    float detectLmax = (bearings.n.x * (obstacles[j].maxX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].maxY - bearings.bLeft.y));
    float detectRmin = (bearings.n.x * (obstacles[j].minX - bearings.bRight.y)) + (bearings.n.y * (obstacles[j].minY - bearings.bRight.y));
    float detectRmax = (bearings.n.x * (obstacles[j].maxX - bearings.bRight.y)) + (bearings.n.y * (obstacles[j].maxY - bearings.bRight.y));

    //TODO link tne lmin and lmax checks
    if(detectLmin < 0  && detectLmax < 0 && detectRmin > 0 && detectRmax > 0){ //If to the left of left bearing and right of right bearing
      heading_checks[i] = 0; //This is not a clear heading
    }
  }
}

//Find first clear bearing to the left of our straight ahead bearing and convert it to a degree bearing
int FindClearPath::find_left_closest(bool* headings){
  int idx = bearingNum/2; //Start at 0 heading 
  int clear = 0;
  //Find first clear heading in the heading array and return it
  for(int i = bearingNum/2; i >= 0; --i){ 
    if(headings[i] == 1){
      idx == i;
      break;
    }
  }
  return (int)(fov*(idx-(int)(bearingNum/2)))/(int)(bearingNum/2);
}
    
//Find first clear bearing to the right of our straight ahead bearing and convert it to a degree bearing
int FindClearPath::find_right_closest(bool* headings){
  int idx = bearingNum/2; //Start at 0 heading
  int clear = 0;
  //Find first clear heading in the heading array and return it
  for(int i = bearingNum/2; i<bearingNum; ++i){
    if(headings[i] == 1){
      idx == i;
      break;
    }
  }
  return (int)(fov*(idx-(int)(bearingNum/2)))/(int)(bearingNum/2);
}