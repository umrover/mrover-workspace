#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
//#include "config.h"
#include "camera.hpp"
#include "artag_detector.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "config.h"
#include <cmath>
#include <lcm/lcm-cpp.hpp>
#include <sys/stat.h> // for disk writing
#include <algorithm>
#include <ctime>
#include <string>
#include <chrono>
#include <thread>

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


