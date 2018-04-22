#pragma once

#include <cmath>
#include "rover_msgs/Odometry.hpp"     
							
#define earthRadiusMeters 6371000
#define PI 3.141592654
#define JOYSTICK_CHANNEL "/joystick"
#define WITHIN_GOAL 4 //outer threshold distance
#define AT_GOAL 1 //inner threshold distance
#define OUTER_MIN 1 //min value for outer threshold
#define INNER_MIN 0.5 //min value for inner threshold

// convert degree to radian
inline double degree_to_radian(const double x, const double minute = 0);

inline double radian_to_degree(const double x);

double estimate_noneuclid(const rover_msgs::Odometry &start, 
							  const rover_msgs::Odometry &dest);

// function to calculate bearing from location 1 to location 2
// Odometry latitude_deg and longintude_deg in degrees
// return bearing in degrees
double calc_bearing(const rover_msgs::Odometry & odom1,
                    const rover_msgs::Odometry & odom2);

bool within_threshold(const rover_msgs::Odometry & odom1,
                      const rover_msgs::Odometry & odom2,
                      const double threshold);
