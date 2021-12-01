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

void FindClearPath::find_clear_path_initiate(EuclideanClusterExtractor::ObsReturn obsVec){

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

  for(int i = 0; i < bearingNum; ++i){
    std::cout << cpu_heading_checks[i] << " ";
  }
  std::cout << std::endl;

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
 
  //printf("%d\n", (heading_checks[i]) ? 1 : 0);

  //Create bearing lines based on threadIdx
  //NB: threadIdx 511 is the 0 heading

  //fov: one directional field of view, currently 80 degrees
  //bearingNum: number of gpu threads
  //i is index of thread

  //Test individual threads
  int map = (i - bearingNum/2);

  //Temporary thread manual overwrite
  //int map = (555 - bearingNum/2);

  //Test individual bearings
  float bearing_deg = float(map * fov) / (bearingNum / 2); //converts thread # to degrees //TODO Bring this back

  //Temporary bearing manual overwrite
  //float bearing_deg = 7.3;

  //printf("%f\n", bearing_deg);
  BearingLines bearings(bearing_deg * 3.1415926535/180.0); //Create bearing lines from bearing //TODO how accurate should pi be?

  // if detect variables are negative, obs is to the left of bearing line     MAYBE RIGHT?? TODO I think this is right
  // if detect variables are positive, obs is to the right of bearing line    MAYBE LEFT??
  // if detect variables are 0, obs is on the bearing line
  for(int j = 0; j < obsArrSize; ++j){ //Check all obstacles in obstacles array
  //printf("obstacle %d minx: %f maxX: %f minY: %f maxY: %f\n", j, obstacles[j].minX, obstacles[j].maxX, obstacles[j].minY, obstacles[j].maxY);

      if(obstacles[j].minX < obstacles[j].maxX && obstacles[j].minZ < obstacles[j].maxZ){ //TODO: also check for minY < maxY ?

        //Note: botLeft = minX, minZ    botRight = maxX, minZ    topLeft = minX, maxZ    topRight = maxX, maxZ

        float LBL_botLeft = (bearings.n.x * (obstacles[j].minX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bLeft.y));
        float RBL_botLeft = (bearings.n.x * (obstacles[j].minX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bRight.y));

        //printf("LBL_botLeft: %f RBL_botLeft: %f\n", LBL_botLeft, RBL_botLeft);

        float LBL_botRight = (bearings.n.x * (obstacles[j].maxX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bLeft.y));
        float RBL_botRight = (bearings.n.x * (obstacles[j].maxX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bRight.y));

        //printf("LBL_botRight: %f RBL_botRight: %f\n", LBL_botRight, RBL_botRight);

        float LBL_topLeft = (bearings.n.x * (obstacles[j].minX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bLeft.y));
        float RBL_topLeft = (bearings.n.x * (obstacles[j].minX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bRight.y));

        //printf("LBL_topLeft: %f RBL_topLeft: %f\n", LBL_topLeft, RBL_topLeft);

        float LBL_topRight = (bearings.n.x * (obstacles[j].maxX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bLeft.y));
        float RBL_topRight = (bearings.n.x * (obstacles[j].maxX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bRight.y));

        //printf("LBL_topRight: %f RBL_topRight: %f\n", LBL_topRight, RBL_topRight);

        //CHECK IF OBSTACLE IS BETWEEN BEARING LINES

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

        //CHECK IF OBSTACLE IS LARGER THAN SPAN OF BEARING LINES
        if((LBL_botLeft > 0 && RBL_botRight < 0) || (LBL_topLeft > 0 && RBL_topRight < 0)){
          heading_checks[i] = 0; // This is not a clear path
        }


        // //checks the left bearing at the minimum z of the object
        // float detectLmin = (bearings.n.x * (obstacles[j].minX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bLeft.y));

        // //checks the left bearing at the maximum z of the object
        // float detectLmax = (bearings.n.x * (obstacles[j].maxX - bearings.bLeft.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bLeft.y));

        // //checks the right bearing at the minimum z of the object
        // float detectRmin = (bearings.n.x * (obstacles[j].minX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].minZ - bearings.bRight.y));

        // //check the right bearing at the maximum z of the object
        // float detectRmax = (bearings.n.x * (obstacles[j].maxX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].maxZ - bearings.bRight.y));

        // printf("detectLmin: %f detectLmax: %f detectRmin: %f detectRmax: %f\n", detectLmin, detectLmax, detectRmin, detectRmax);

        // bool allNegative = detectLmin < 0 && detectLmax < 0 
        //                       && detectRmin < 0 && detectRmax < 0; 

        // bool allPositive = detectLmin > 0 && detectLmax > 0 
        //                       && detectRmin > 0 && detectRmax > 0;  


        // // printf("%f all negative : %d\n", bearing, (allNegative) ? 1 : 0);
        // // printf("%f all positive : %d\n", bearing, (allPositive) ? 1 : 0);

        // if(!allNegative && !allPositive){
        //   heading_checks[i] = 0; //This is not a clear heading
        // } else if(allNegative){
        //   //check left bearing line with xmax, ymin to see if its to the left
        //   //if its to the left, then that is a collision
        //   float detectBottomRight = (bearings.n.x * (obstacles[j].maxX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].minY - bearings.bRight.y));

        //   if(detectBottomRight > 0){
        //     heading_checks[i] = 0; //This is not a clear heading
        //   }
        // }else if(allPositive){
        //   //check right bearing line with xmin, ymax to see if its to the right
        //   //if its to the right, then that is a collision
        //   float detectTopLeft = (bearings.n.x * (obstacles[j].minX - bearings.bRight.x)) + (bearings.n.y * (obstacles[j].maxY - bearings.bRight.y));
          
        //   if(detectTopLeft < 0){
        //     heading_checks[i] = 0; //This is not a clear heading
        //   }
        // }
      }


    // bool min_test = (detectLmin < 0 && detectRmin < 0) || (detectLmin > 0 && detectRmin > 0); //checks that the left bearing and the right bearing are on the same side for 
    // bool max_test = (detectLmax < 0 && detectRmax < 0) || (detectLmax > 0 && detectRmax > 0);
    // if(min_test && max_test){ //If to the left of left bearing and right of right bearing
    //   heading_checks[i] = 0; //This is not a clear heading
  }
  // } 
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