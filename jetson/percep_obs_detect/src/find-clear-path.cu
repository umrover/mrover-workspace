#include "find-clear-path.hpp"



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
  n.x = -cos(heading_in); //Calculate x component of orthogonal vec from heading_in
  n.y = sin(heading_in); //Calculate y component of orthogonal vec from heading_in
  bLeft.x = (-rovWidth/2) * cos(heading_in); //Calculate bLeft x offset from heading_in
  bLeft.y = (rovWidth/2) * sin(heading_in); //Calculate bLeft y offset from heading_in
  bRight.x = (rovWidth/2) * cos(heading_in); //Calculate bRight x offset from heading_in
  bRight.y = (-rovWidth/2) * sin(heading_in); //Calculate bRight y offset from heading_in
}

void FindClearPath::find_clear_path_initiate(ObsReturn obsVec){
  //Allocate and copy obstacle structs array
  Obstacle* gpuObstacles; 
  cudaMalloc(&gpuObstacles, obsVec.obs.size()*sizeof(Obstacle));
  cudaMemcpy(gpuObstacles, &obsVec.obs[0], obsVec.obs.size()*sizeof(Obstacle), cudaMemcpyHostToDevice);
  
  //Allocate heading checks array
  bool* heading_checks;
  cudaMalloc(&heading_checks, bearingNum*sizeof(bool));

  //Run find_clear_path on each of the 1024 headings (threads)
  find_clear_path<<<1, bearingNum>>>(gpuObstacles, heading_checks, obsVec.obs.size());

  checkStatus(cudaDeviceSynchronize());

  //TODO: what to do with heading_checks array
  bool* cpu_heading_checks = new bool[bearingNum];
  cudaMemcpy(cpu_heading_checks, heading_checks, bearingNum, cudaMemcpyDeviceToHost);
  int heading_final = find_closest_clear_path(cpu_heading_checks);

  //Free memory
  cudaFree(gpuObstacles);
  cudaFree(heading_checks);
}

__global__ void FindClearPath::find_clear_path(Obstacle* obstacles, bool* heading_checks, int obsArrSize){
  //NB: Currently treating obstacles as points
  //TODO: Logic for obstacles as dimensional objects
  
  int i = threadIdx.x;

  //Create bearing lines based on threadIdx
  //NB: threadIdx 511 is the 0 heading
  BearingLines bearings((80*(i-511))/511) //Create bearing lines from threadIdx
  for(int j = 0; j < obsArrSize; ++j){ //Check all obstacles in obstacles array
    float detectL = (bearings.n.x * (obstacles.x - bLeft.x)) + (bearings.n.y * (obstacles.y - bLeft.y));
    float detectR = (bearings.n.x * (obstacles.x - bRight.y)) + (bearings.n.y * (obstacles.y - bRight.y));
    if(detectL < 0 && detectR > 0){ //If to the left of left bearing and right of right bearing
      heading_checks[i] = 0; //This is not a clear heading
      break; //Stop checking other obstacles
    }
    heading_checks[i] = 1; //This is a clear heading
  }
}

int find_closest_clear_path(bool* headings){
  int idx = 511; //This is the 0 heading
  int clear = 0;
  for(int i = 0; i < 511; ++i){
    if(headings[curr_clear_path_idx - i] == 0){
      return 1;
    }else if(headings[curr_clear_path_idx + i] == 0){
      return 0;
    }
  }
}