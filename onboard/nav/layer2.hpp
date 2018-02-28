#pragma once

#include "layer1.hpp"
#include <tgmath.h>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Joystick.hpp"
#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/NavStatus.hpp"
#include <deque>

#include "thor.hpp"

const double METER_IN_MINUTES = 0.0005389625;
const int PATH_WIDTH = 200;
const double DIRECTION_THRESH = 1.0;
const double INNER_SEARCH_THRESH = .0001;
#define NAV_STATUS_CHANNEL "/nav_status"

//typedef rover_msgs::Odometry odom;
typedef rover_msgs::Waypoint waypoint;

enum class State {
	translational, 
	faceN,
	rotation120,
	rotation240,
	rotationN,
};

class Layer2 {
public:
	// custom constructor for layer2
	Layer2 (lcm::LCM & lcm_object);

	~Layer2() {}

	Thor::Volatile<rover_msgs::Odometry> cur_odom_;
	Thor::Volatile<bool> ball_detected_;

	Thor::Volatile<rover_msgs::AutonState> auton_state_;

	void run();

	void set_course(const rover_msgs::Course * course);

private:
	struct CourseData{
		std::deque <rover_msgs::Waypoint> overall;
		int64_t hash;
	};

  	State state;
  	lcm::LCM & lcm_;
  	Layer1 layer1;

	int32_t total_wps; //fjul : original # wps in course
	int32_t completed_wps; //fjul
	int8_t nav_state; //fjul
	Thor::Volatile<CourseData> course_data_;

	std::deque <waypoint> search;
	std::vector<std::pair<short,short>> search_point_multipliers;

	void init_search_multipliers();

	void add_four_points_to_search(const waypoint & origin_way);

	void search_func(const waypoint & search_origin);

	bool rotation_pred(const odom & cur_odom, const double theta1, const double theta2);

	void search_rotation(const odom & cur_odom);

	bool waypoint_eq(const waypoint & way1, const waypoint & way2);

	void waypoint_assign(waypoint & way1, const waypoint & way2);

	void make_and_publish_nav_status(const int8_t state);

};
