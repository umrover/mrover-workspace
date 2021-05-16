#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "config.h"
#include "camera.hpp"
#include "rover_msgs/Obstacle.hpp"
#include <chrono>
#include <cmath>
#include <lcm/lcm-cpp.hpp>
#include <sys/stat.h> // for disk writing
#include <algorithm>
#include <ctime>
#include <string>
#include <chrono>
#include <thread>
#include <fstream>
#include "rapidjson/document.h"

#if OBSTACLE_DETECTION
/* --- PCL Includes --- */
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif

/* --- Ignore Error Checking for 3rd Part Header Files --- */
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wcomment"

#if ZED_SDK_PRESENT
  #include <sl/Camera.hpp>
#endif

#if PERCEPTION_DEBUG
  #include <opencv2/highgui/highgui.hpp>
  #include <cstdlib>  
#endif

#pragma GCC diagnostic pop

#define PI 3.14159265

class obstacle_return {
  public:
  double bearing;
  double distance; //Distance to nearest obstacle
  
  obstacle_return() {
    bearing = 0;
    distance = -1;
  }

  obstacle_return(double bearing_in, double distance_in) : 
                  bearing{bearing_in}, distance{distance_in} {}

  obstacle_return& operator=(obstacle_return & in){
    if(this == &in){
      return *this;
    }
    
    bearing = in.bearing;
    distance = in.distance;
    return *this;
  }

};

//ar tag detector class
#include "artag_detector.hpp"

//pcl class
#include "pcl.hpp"
