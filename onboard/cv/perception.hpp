#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera.hpp"
#include "rover_msgs/TennisBall.hpp"
#include "rover_msgs/Obstacle.hpp"
#include <chrono>
#include <cmath>
#include <lcm/lcm-cpp.hpp>
#include <vector>

#define PERCEPTION_DEBUG
#define THRESHOLD_NO_WAY  300000 //how will we calibrate if the rover width changes
#define THRESHOLD_NO_OBSTACLE_CENTER  400000
#define SKY_START_ROW 200


#define PI 3.14159265
#define FRAME_WAITKEY 0 // for cv::imshow

const int RESOLUTION_HEIGHT = 720; // 720p
const float fieldofView = 110 * PI/180;
const float inf = -std::numeric_limits<float>::infinity();
const float zedHeight = 17 * 0.0254; //inches to meters off the ground
const float realWidth = 46 * 25.4; //rover width , rover is 46 inches wide TODO make a little longer to be safe
const float angleOffset = 10 * PI/180;    //angle offset of the rover
const int num_sliding_windows = 10;
const float focalLength = 2.8; //zed focal length in mm

const float distThreshold = 4;    //meters, used to calculate rover pixels
const float obstacleThreshold = 5 * 0.0254; //inches to meters

struct obstacle_return {
  float center_distance; // distance to anything in front of the center of the camera                                       
  float bearing; // [-50 degree, 50 degree]
};


std::vector<cv::Point2f> findTennisBall(cv::Mat &src);
obstacle_return avoid_obstacle_sliding_window(cv::Mat &depth_img, cv::Mat &rgb_img, int num_windows, int rover_width );
