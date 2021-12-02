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
  bLeft.x = (-rovWidth/2) * cos(heading_in); //Calculate bLeft x offset from heading_in //POSSIBLE ISSUE, LEFT IS POS x TODO: CHECK
  bLeft.y = (rovWidth/2) * sin(heading_in); //Calculate bLeft y offset from heading_in //POSSIBLE ISSUE, LEFT IS POS x
  bRight.x = (rovWidth/2) * cos(heading_in); //Calculate bRight x offset from heading_in //POSSIBLE ISSUE, LEFT IS POS x
  bRight.y = (-rovWidth/2) * sin(heading_in); //Calculate bRight y offset from heading_in //POSSIBLE ISSUE, LEFT IS POS x
}

float2 FindClearPath::find_clear_path_initiate(EuclideanClusterExtractor::ObsReturn obsVec){

  std::cout << "size of obsVec: " << obsVec.obs.size() << std::endl;

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

  // Prints out heading_check array
  // for(int i = 0; i < bearingNum; ++i){
  //   std::cout << cpu_heading_checks[i] << " ";
  // }
  // std::cout << std::endl;

  //Find closest heading to the left and right of our current heading
  int heading_left = find_left_closest(cpu_heading_checks);
  int heading_right = find_right_closest(cpu_heading_checks);

  //TODO cout in the driver
  // std::cout << "left heading: " << heading_left << std::endl;
  // std::cout << "right heading: " << heading_right << std::endl;

  //Free memory
  cudaFree(gpuObstacles);
  cudaFree(heading_checks);

  float2 output;
  output.x = heading_left;
  output.y = heading_right;
  return output;
}

__global__ void find_clear_path(EuclideanClusterExtractor::Obstacle* obstacles, bool* heading_checks, int obsArrSize){
  
  int i = threadIdx.x;
  heading_checks[i] = 1; //Assume a clear heading
 
  int map = (i - bearingNum/2);

  float bearing_deg = float(map * fov) / (bearingNum / 2); //converts thread # to degrees //TODO Bring this back

  BearingLines bearings(bearing_deg * 3.1415926535/180.0); //Create bearing lines from bearing //TODO how accurate should pi be?

  // if detect variables are negative, obs is to the right of bearing line
  // if detect variables are positive, obs is to the left of bearing line
  // if detect variables are 0, obs is on the bearing line
  for(int j = 0; j < obsArrSize; ++j){ //Check all obstacles in obstacles array
      if(obstacles[j].minX < obstacles[j].maxX && obstacles[j].minZ < obstacles[j].maxZ){ 

        //Note: botLeft = minX, minZ    botRight = maxX, minZ    topLeft = minX, maxZ    topRight = maxX, maxZ
        float LBL_botLeft = (bearings.n.x * (obstacles[j].minX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bLeft.y));
        float RBL_botLeft = (bearings.n.x * (obstacles[j].minX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bRight.y));

        float LBL_botRight = (bearings.n.x * (obstacles[j].maxX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bLeft.y));
        float RBL_botRight = (bearings.n.x * (obstacles[j].maxX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bRight.y));

        float LBL_topLeft = (bearings.n.x * (obstacles[j].minX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bLeft.y));
        float RBL_topLeft = (bearings.n.x * (obstacles[j].minX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bRight.y));

        float LBL_topRight = (bearings.n.x * (obstacles[j].maxX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bLeft.y));
        float RBL_topRight = (bearings.n.x * (obstacles[j].maxX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bRight.y));

        // Check if obstacle its between bearing lines
        if((LBL_botLeft > 0 && RBL_botLeft < 0) || (LBL_botLeft < 0 && RBL_botLeft > 0)
            || LBL_botLeft == 0 || RBL_botLeft == 0){ 
          heading_checks[i] = 0; // This is not a clear path
        }

        if((LBL_botRight > 0 && RBL_botRight < 0) || (LBL_botRight < 0 && RBL_botRight > 0)
            || LBL_botRight == 0 || RBL_botRight == 0){ 
          heading_checks[i] = 0; // This is not a clear path
        }

        if((LBL_topLeft > 0 && RBL_topLeft < 0) || (LBL_topLeft < 0 && RBL_topLeft > 0)
            || LBL_topLeft == 0 || RBL_topLeft == 0){ 
          heading_checks[i] = 0; // This is not a clear path
        }

        if((LBL_topRight > 0 && RBL_topRight < 0) || (LBL_topRight < 0 && RBL_topRight > 0)
            || LBL_topRight == 0 || RBL_topRight == 0){ 
          heading_checks[i] = 0; // This is not a clear path
        }

        // Check if obstacle is larger than span of bearing lines
        if((LBL_botLeft > 0 && RBL_botRight < 0) || (LBL_topLeft > 0 && RBL_topRight < 0)){
          heading_checks[i] = 0; // This is not a clear path
        }
      }
  }
}

//Find first clear bearing to the left of our straight ahead bearing and convert it to a degree bearing
float FindClearPath::find_left_closest(bool* headings){
  int idx = bearingNum/2; //Start at 0 heading 
  int clear = 0;
  //Find first clear heading in the heading array and return it
  while(clear == 0 && idx >= 0){ 
    if(headings[idx] == 1){
      clear = 1;
    }
    --idx;
  }
  return (fov*((idx+1)-(bearingNum/2)))/(bearingNum/2);
}
    
//Find first clear bearing to the right of our straight ahead bearing and convert it to a degree bearing
float FindClearPath::find_right_closest(bool* headings){
  int idx = bearingNum/2; //Start at 0 heading
  int clear = 0;
  //Find first clear heading in the heading array and return it
  while(clear == 0 && idx < bearingNum){ 
    if(headings[idx] == 1){
      clear = 1;
    }
    ++idx;
  }
  return (fov*((idx+1)-(bearingNum/2)))/(bearingNum/2);
}