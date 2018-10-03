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
#include <iostream>
#include <cassert>
#include <iomanip>
#include <unistd.h>

const double EARTH_CIRCUM = 40075000; // meters
const double LAT_METER_IN_MINUTES = 0.0005389625; // minutes/meter
const int PATH_WIDTH = 2; // meters
const int SEARCH_BAIL_THRESH = 10; // meters
const double DIRECTION_THRESH = 5; // degrees
const double INNER_SEARCH_THRESH = 5; // degrees
const double BALL_THRESH = 1; // meters
const double CV_THRESH = 5; // meters
const double AVOIDANCE_SCALE = 2;
const int CV_FOV = 110;

#define NAV_STATUS_CHANNEL "/nav_status"
using waypoint = rover_msgs::Waypoint;
using odom = rover_msgs::Odometry;
using TennisBall = rover_msgs::TennisBall;
using Obstacle = rover_msgs::Obstacle;
using AutonState = rover_msgs::AutonState;

enum class State {
	off,						// 0
	turn,						// 10
	drive,						// 11
	search_face0,				// 20
	search_turn120,				// 21
	search_turn240,				// 22
	search_turn360,				// 23
	search_turn,				// 24
	search_drive, 				// 25
	turn_to_ball,				// 28
	drive_to_ball,				// 29
	turn_around_obs,			// 30
	drive_around_obs,			// 31
	search_turn_around_obs,		// 32
	search_drive_around_obs,	// 33
	done						// 1
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
		std::deque <waypoint> overall;
		int64_t hash;
	};

	class BallUpdate {
	private: 
		const TennisBall * prev_ball;

	public:
		BallUpdate(const TennisBall * ball) : prev_ball(ball) {}

		bool operator()(const TennisBall & new_ball) const {
			return new_ball.found != prev_ball->found;
		}
	};

	class ObstacleUpdate {
	private:
		const Obstacle * prev_obs;

	public:
		ObstacleUpdate(const Obstacle * obs) : prev_obs(obs) {}

		bool operator()(const Obstacle & new_obs) const {
			return new_obs.detected != prev_obs->detected;
		}
	};

	class AutonStateUpdate {
	private:
		const AutonState * prev_state;

	public:
		AutonStateUpdate(const AutonState * state) : prev_state(state) {}

		bool operator()(const AutonState & new_state) const {
			return new_state.is_auton != prev_state->is_auton;
		}
	};

	odom rover_cur_odom;
	TennisBall rover_ball;
	Obstacle rover_obstacle;
	CourseData rover_course;
	AutonState rover_auton_state;
	waypoint search_center;
	odom dummy_obs_odom;
	double original_obs_angle;

  	State state;
  	lcm::LCM & lcm_;
  	Layer1 layer1;

  	double long_meter_in_minutes;
	int32_t total_wps; //fjul : original # wps in course
	int32_t completed_wps; //fjul
	int32_t missed_wps;
	int8_t nav_state; //fjul
	Thor::Volatile<CourseData> course_data_;

	std::deque <waypoint> rover_search; // TODO make odom
	std::vector<std::pair<short,short>> search_point_multipliers;

	void init_search_multipliers();

	bool add_four_points_to_search(const waypoint & origin_way);

	// void search_func(const waypoint & search_origin);

	bool rotation_pred(const odom & cur_odom, const double theta1, 
					   const double theta2);

	// void search_rotation(const odom & cur_odom);

	bool waypoint_eq(const waypoint & way1, const waypoint & way2);

	void waypoint_assign(waypoint & way1, const waypoint & way2);

	void make_and_publish_nav_status(const int8_t state);

	void long_meter_mins(const odom & cur_odom);
  
  	// void go_to_ball(odom & cur_odom, TennisBall & ball);

	// void obstacle_avoidance(odom & cur_odom, Obstacle & obs);
  
  	void obstacle_dummy_odom(odom & new_odom, const double cur_bearing,
                             const double dist);

  	double calc_dist_obs_way(const odom & way, const double obs_angle) const;

  	// double calc_bearing_from_rover(const odom & goal) const;

	void updateRover();

	void updateRover_ballUnconditional();

	void updateRover_obsUnconditional();

};
