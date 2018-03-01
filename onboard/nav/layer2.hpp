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
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/TennisBall.hpp"
#include <deque>

#include "thor.hpp"

const double EARTH_CIRCUM = 40075000; // meters
const double LAT_METER_IN_MINUTES = 0.0005389625; // meters/minute
const int PATH_WIDTH = 50; // meters
const double DIRECTION_THRESH = 1.0;
const double INNER_SEARCH_THRESH = .0001;
const double BALL_THRESH = 2; // meters
const double CV_THRESH = 3; // meters
#define NAV_STATUS_CHANNEL "/nav_status"

//typedef rover_msgs::Odometry odom;
// typedef rover_msgs::Waypoint waypoint;
using waypoint = rover_msgs::Waypoint;

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
	// Thor::Volatile<bool> ball_detected_;

	Thor::Volatile<rover_msgs::AutonState> auton_state_;

	Thor::Volatile<TennisBall> tennis_ball_;
	Thor::Volatile<Obstacle> obstacle_;

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

  	double long_meter_in_minutes;
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

	void long_meter_mins(const odom & cur_odom);
  
  void go_to_ball(odom & cur_odom, TennisBall & ball);

	void obstacle_avoidance(odom & cur_odom, Obstacle & obs);
  
  void turn(odom & cur_odom, double bearing_offset);
  
  void obstacle_dummy_odom(odom & new_odom, const double cur_bearing,
                           const double dist);

};
