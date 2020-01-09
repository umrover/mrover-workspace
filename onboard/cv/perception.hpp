#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "camera.hpp"
#include "rover_msgs/Obstacle.hpp"
#include <chrono>
#include <cmath>
#include <lcm/lcm-cpp.hpp>
#include <sys/stat.h> // for disk writing
#if PERCEPTION_DEBUG
  #include <opencv2/highgui/highgui.hpp>
  #include <cstdlib>
#endif

#define THRESHOLD_NO_WAY  80000 //how will we calibrate if the rover width changes
#define THRESHOLD_NO_OBSTACLE_CENTER  80000
#define THRESHOLD_NO_SUBWINDOW 27000
#define THRESHOLD_CONFIDENCE 90
#define SKY_START_ROW 400
#define BALL_DETECTION_MAX_DIST 6.0  // this number is obtained from experiment. if the distance of the detected ball is greater than this number, false detection, we should ignore
#define BALL_DETECTION_MIN_RAD 5
#define BALL_DETECTION_MAX_RAD 300

#define SIMILARITY_THRESHOLD 8000

#define PI 3.14159265
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
const float zedHeight = 17 * 0.0254; //inches to meters off the ground
const float realWidth = 46 * 25.4; //rover width , rover is 46 inches wide TODO make a little longer to be safe
const float angleOffset = 10 * PI/180;    //angle offset of the rover

//Obstacle Detection variables
const int num_sliding_windows = 20;
const float distThreshold = 2.5;    //meters, used to calculate rover pixels
const float obstacleThreshold = 5 * 0.0254; //inches to meters

struct obstacle_return {
  float center_distance; // distance to anything in front of the center of the camera
  float bearing; // [-50 degree, 50 degree]
};

//functions
std::pair<cv::Point2f, double> findTennisBall(cv::Mat &src, cv::Mat &depth_src);
obstacle_return avoid_obstacle_sliding_window(cv::Mat &depth_img, cv::Mat &rgb_img, int num_windows, int rover_width);
