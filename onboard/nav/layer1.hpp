#pragma once

#include <tgmath.h>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Joystick.hpp"
#include "layers.hpp"
#include "pid.hpp"
#include "thor.hpp"
#include <cassert>
#include <cmath>

#define earthRadiusMeters 6371000
#define PI 3.141592654
#define JOYSTICK_CHANNEL "/joystick"
#define WITHIN_GOAL 5 //outer threshold distance
#define AT_GOAL 0.25 //inner threshold distance
#define OUTER_MIN 1 //min value for outer threshold
#define INNER_MIN 0.5 //min value for inner threshold

class Layer1 : public Auton::Layer {
public:

	// custom constructor for layer1
	Layer1 (Auton::System &sys, lcm::LCM & lcm_object);

	~Layer1 () {}

    Thor::Volatile<rover_msgs::Odometry> cur_odom;
    Thor::Volatile<rover_msgs::Odometry> goal_odom;

	// 
	virtual void run() override;

private:

	PidLoop bearing_pid;
	PidLoop distance_pid;

	//bool representing whether first turn has been executed
	bool first;
	bool in_outer_thresh;
	bool in_inner_thresh;
	double outer_thresh;
	double inner_thresh;

	lcm::LCM & lcm_;

	void calc_bearing_thresholds(const rover_msgs::Odometry &cur_odom,
            const double distance, const double bearing);

	void turn_to_dest(
            const rover_msgs::Odometry &cur_odom,
            const rover_msgs::Odometry &goal_odom);

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
