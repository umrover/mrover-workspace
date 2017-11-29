#pragma once

#include <tgmath.h>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Joystick.hpp"
#include "layers.hpp"
#include <cassert>

#define earthRadiusMeters 6371000
#define PI 3.141592654
#define JOYSTICK_CHANNEL "/joystick"
#define kp 0.02
#define ki 0.000000001
#define kd 0.025

class Layer1 : public Auton::Layer {
public:

	// custom constructor for layer1
	Layer1 (Auton::System &sys, lcm::LCM & lcm_object);

	~Layer1 () {}

	rover_msgs::Odometry cur_odom, goal_odom;

	// 
	virtual void run() override;

private:

	double total_error = 0, last_error = 0;

	lcm::LCM & lcm_;

	// function to calculate bearing from location 1 to location 2
	// Odometry latitude_deg and longintude_deg in degrees
	// return bearing in degrees
	double calc_bearing(const rover_msgs::Odometry &odom1, 
						const rover_msgs::Odometry &odom2);

	double estimate_noneuclid(const rover_msgs::Odometry &start, 
							  const rover_msgs::Odometry &dest);

	rover_msgs::Joystick make_joystick_msg(const double forward_back, const double left_right,
							   const bool kill);

	// convert degree to radian
	inline double degree_to_radian(const double x, const double minute);

	// convert radian to degree
	inline double radian_to_degree(const double x);

	bool within_threshold(const rover_msgs::Odometry &odom1, 
						  const rover_msgs::Odometry &odom2, 
						  const double threshold);

};
