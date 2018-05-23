#pragma once

#include <tgmath.h>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "calculations.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Joystick.hpp"
#include "rover_msgs/TennisBall.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "pid.hpp"
#include "thor.hpp"
#include <cassert>
#include <cmath>

// typedef rover_msgs::Odometry odom;
using odom = rover_msgs::Odometry;
using TennisBall = rover_msgs::TennisBall;
using Obstacle = rover_msgs::Obstacle;

class Layer1 {
public:

	// custom constructor for layer1
	Layer1 (lcm::LCM & lcm_object);

	bool turn(const odom & current_odom, const odom & target_odom);

	bool drive(const odom & current_odom, const odom & target_odom);

	// bool translational(const odom & current_odom, const odom & target_odom);

	void turn_to_bearing(const odom & current_position, double desired_bearing);

	void drive_forward(const odom & cur_odom, const double bearing_offset, const double dist_to_target);

	void make_publish_joystick(const double forward_back, const double left_right, const bool kill);

	PidLoop bearing_pid;
	PidLoop distance_pid;
	
private:

	//bool representing whether first turn has been executed
	bool first;
	bool in_outer_thresh;
	bool in_inner_thresh;
	double outer_thresh;
	double inner_thresh;

	void calc_bearing_thresholds(const odom & cur_odom,
            const double distance, const double bearing);

	lcm::LCM & lcm_;

	void throughZero(double & dest_bearing, const double cur_bearing);
	
	double turn_to_dest(const odom & cur_odom, const odom & goal_odom);

	double turn_to_dest(const odom &cur_odom, const double angle);

	//rover_msgs::Joystick make_joystick_msg(const double forward_back, const double left_right, const bool kill);


};
