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

/* --- PCL Includes --- */
#include <iostream>
#include "perception.hpp"
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

/* --- Ignore Error Checking for 3rd Part Header Files --- */
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wcomment"

#include <sl/Camera.hpp>

#if PERCEPTION_DEBUG
  #include <opencv2/highgui/highgui.hpp>
  #include <cstdlib>
  #include <pcl/visualization/pcl_visualizer.h>
#endif

#pragma GCC diagnostic pop

#define THRESHOLD_NO_WAY  80000 //how will we calibrate if the rover width changes
//#define THRESHOLD_NO_OBSTACLE_CENTER  80000
//#define THRESHOLD_NO_SUBWINDOW 27000
#define THRESHOLD_CONFIDENCE 90
#define SKY_START_ROW 400
#define BALL_DETECTION_MAX_DIST 6.0  // this number is obtained from experiment. if the distance of the detected ball is greater than this number, false detection, we should ignore
#define BALL_DETECTION_MIN_RAD 5
#define BALL_DETECTION_MAX_RAD 300

#define SIMILARITY_THRESHOLD 8000

#define PI 3.14159265

/* --- Point Cloud Definitions --- */
#define PT_CLOUD_WIDTH 320
#define PT_CLOUD_HEIGHT 180
#define ROVER_W_MM 1168
#define HALF_ROVER 584
#define CENTERX 0

const float inf = -std::numeric_limits<float>::infinity();

const int FRAME_WRITE_INTERVAL = 10; // How many frames between writing to disk

#if ZED_SDK_PRESENT
  const int FRAME_WAITKEY = 1; // for cv::imshow, set to 1 if using zed, set to 0 if offline test
#else
  const int FRAME_WAITKEY = 0;
#endif

//Zed Specs
const int RESOLUTION_WIDTH = 1280;
const int RESOLUTION_HEIGHT = 720; // 720p
const float TENNIS_BALL_PIXEL_SIZE = 25; // Pixel radius of tennis ball at 1m distance
const float fieldofView = 110 * PI/180;
const float focalLength = 2.8; //zed focal length in mm

//Rover Specs
const float zedHeight = 17 * 0.0254; //uinches to meters off the ground
const float realWidth = 46 * 25.4; //rover width , rover is 46 inches wide TODO make a little longer to be safe
const float angleOffset = 10 * PI/180;    //angle offset of the rover

//Obstacle Detection variables
const int num_sliding_windows = 20;
const float distThreshold = 2.5;    //meters, used to calculate rover pixels
const float obstacleThreshold = 5 * 0.0254; //inches to meters

class obstacle_return {
  public:
  float bearing;
  float distance; //Distance to nearest obstacle
  std::vector<pcl::PointXYZRGB> points;
  
  obstacle_return() {
    bearing = 0;
    distance = 0;
  }

  obstacle_return& operator=(obstacle_return & in){
    if(this == &in){
      return *this;
    }
    for(auto point : in.points){
      points.push_back(point);
    }
    
    bearing = in.bearing;
    return *this;
  }

};

//functions
obstacle_return pcl_obstacle_detection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, std::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

//ar tag detector class
#include "artag_detector.hpp"
