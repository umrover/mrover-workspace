#include "layer2.hpp"
#include <iostream>
#include <cassert>
#include <iomanip>
// TODO latitude vs longitude calc minutes to meters

// custom constructor for layer2
Layer2::Layer2(lcm::LCM &lcm_object) : 
    state(State::translational), lcm_(lcm_object), layer1(lcm_), 
    total_wps (-1), completed_wps(-1), nav_state(-1) {}

void Layer2::set_course(const rover_msgs::Course * course)
{
	this->course_data_.transaction([&](CourseData & d) {

		if (d.hash == course->hash ) { return false; }

		d.overall.clear();

		for (int i = 0; i < course->num_waypoints; ++i)
			d.overall.push_back(course->waypoints[i]);

		this->total_wps = d.overall.size();
		this->completed_wps = 0;
		d.hash = course->hash;
		state = State::translational;
		return true;
	});	
}

void Layer2::run()
{
	CourseData d = this->course_data_.clone();
	rover_msgs::Odometry cur_odom = this->cur_odom_.clone();
	rover_msgs::AutonState auton_state = this->auton_state_.clone();
	bool prev_entered_auton = false;

	long_meter_mins(cur_odom);

	while (true)
	{
		while ( !auton_state.is_auton ) {
			make_and_publish_nav_status(6);
			auton_state = this->auton_state_.clone_when_changed();
		}

		d = this->course_data_.clone();
		this->completed_wps = 0;
		this->total_wps = d.overall.size();

		rover_msgs::Obstacle obs = this->obstacle_.clone();

	    while ( !d.overall.empty() && auton_state.is_auton ) 
	    {
	    	prev_entered_auton = true;
			waypoint target = d.overall.front();
			cur_odom = this->cur_odom_.clone();

			if (obs.detected) {
				obstacle_avoidance(cur_odom, obs);
			}

			if ( layer1.translational(cur_odom, target.odom) ) {
				++this->completed_wps;
				d.overall.pop_front();
				if ( target.search ) search_func(target);
			}

			make_and_publish_nav_status(0);
			cur_odom = this->cur_odom_.clone_when_changed();
			auton_state = this->auton_state_.clone();
			this->obstacle_.clone_conditional([&](const Obstacle & new_obs) {
				return new_obs.detected != obs.detected;
			}, &obs);

		} //while(!empty && auton)

		// if main loop prev entered and course{} empty ==> DONE, clear/reset
		if ( prev_entered_auton && d.overall.empty() ) {
			make_and_publish_nav_status(8); //pubish DONE navStatus
			d.hash = 0; //required for transactional --> clone_when_changed()
			prev_entered_auton = false;
			d = this->course_data_.clone_when_changed();
		}

		// else if(!prev_entered_auton) {
		// 	make_and_publish_nav_status(7);
		// 	d = this->course_data_.clone_when_changed();
		// }

		//else if ( ANYTHING && !empty() ) ==> (DO NOTHING) auton off, wait @ top of loop for it to turn on)

	} //while(true)
}

void Layer2::search_func(const waypoint & search_origin) {

	this->state = State::faceN;

	rover_msgs::Odometry cur_odom = this->cur_odom_.clone();
	make_and_publish_nav_status(1);

	while ( state != State::translational ) {
		this->search_rotation(cur_odom);
		cur_odom = this->cur_odom_.clone_when_changed();
	}

	TennisBall ball = this->tennis_ball_.clone();

	this->init_search_multipliers();
	this->add_four_points_to_search(search_origin); //arbitrary addition of waypoints to search
	// this->add_four_points_to_search(search_origin);
	// this->add_four_points_to_search(search_origin);

	while (!ball.found && !search.empty() ) {
		auto target = search.front();

		Obstacle obs = this->obstacle_.clone();
		if (obs.detected) obstacle_avoidance(cur_odom, obs);
		while ( !layer1.translational(cur_odom, target.odom) ) {
			cur_odom = this->cur_odom_.clone_when_changed();
			this->obstacle_.clone_conditional( [&](const Obstacle & new_obs) {
				return new_obs.detected != obs.detected;
			}, &obs);
			if (obs.detected) obstacle_avoidance(cur_odom, obs);
		}
		
		search.pop_front();		//once @ point, pop it off
		this->tennis_ball_.clone_conditional([&](const TennisBall & new_ball) {
			return new_ball.found != ball.found;
		}, &ball);
	}


	make_and_publish_nav_status(3);
	// cur_odom = this->cur_odom_.clone();
	go_to_ball(cur_odom, ball);
	make_and_publish_nav_status(4);

	search.clear();
}

bool Layer2::rotation_pred(const odom & cur_odom, const double theta1, const double theta2){
	return (abs(cur_odom.bearing_deg - theta1) < INNER_SEARCH_THRESH || 
			abs(cur_odom.bearing_deg - theta2) < INNER_SEARCH_THRESH);
}

void Layer2::search_rotation(const odom & cur_odom) {
	State next_state = state;

	if (state == State::faceN) {
		layer1.turn_to_bearing(cur_odom, 0);
		if (rotation_pred(cur_odom, 0, 360))
       		next_state = State::rotation120;
	}
	else if (state == State::rotation120) {
		layer1.turn_to_bearing(cur_odom, 120);
		if (rotation_pred(cur_odom, 120, 120))
        	next_state = State::rotation240; 
    }
	else if (state == State::rotation240) {
	    layer1.turn_to_bearing(cur_odom, 240);
	 	if (rotation_pred(cur_odom, 240, 240))
        	next_state = State::rotationN;
    }
    else if (state == State::rotationN) {
        layer1.turn_to_bearing(cur_odom, 0);
        if (rotation_pred(cur_odom, 0, 360))
        	next_state = State::translational;
    }

    else abort();
    
    state = next_state;
}

void Layer2::init_search_multipliers(){
	search_point_multipliers.clear();
	search_point_multipliers.push_back(std::pair<short,short> (0,1));
    search_point_multipliers.push_back(std::pair<short,short> (-1,1));
    search_point_multipliers.push_back(std::pair<short,short> (-1,-1));
    search_point_multipliers.push_back(std::pair<short,short> (1,-1));
}

void Layer2::add_four_points_to_search(const waypoint & origin_way)
{    
    for ( int i = 0; i < 4 ; ++i ){
        std::pair<short,short> & lead_pat = search_point_multipliers[i];
    
        rover_msgs::Waypoint next_search_way = origin_way;
        next_search_way.search = false;
   
   		double total_lat_min = next_search_way.odom.latitude_min + 
   				((lead_pat.first * PATH_WIDTH) * LAT_METER_IN_MINUTES);
        double total_long_min = next_search_way.odom.longitude_min + 
        		((lead_pat.second * PATH_WIDTH) * long_meter_in_minutes);

  		int added_lat_deg = total_lat_min / 60;
        int added_long_deg = total_long_min / 60;
        
        next_search_way.odom.latitude_deg += added_lat_deg;
        next_search_way.odom.longitude_deg += added_long_deg;
        
        next_search_way.odom.latitude_min = total_lat_min - (added_lat_deg * 60.0);
        next_search_way.odom.longitude_min = total_long_min - (added_long_deg * 60.0);

        search.push_back(next_search_way);

        lead_pat.first < 0 ? --lead_pat.first : ++lead_pat.first;
        lead_pat.second < 0 ? --lead_pat.second : ++lead_pat.second;
    }
}

void Layer2::make_and_publish_nav_status(const int8_t state) 
{
	rover_msgs::NavStatus nav_status_;
	nav_status_.nav_state = state;
	nav_status_.completed_wps = this->completed_wps;
	nav_status_.total_wps = this->total_wps;
	lcm_.publish(NAV_STATUS_CHANNEL, &nav_status_);	
}

inline bool Layer2::waypoint_eq(const waypoint & way1, const waypoint & way2)
{
	if (way1.search != way2.search) return false;
	if (abs(way1.odom.latitude_min - way2.odom.latitude_min) > 0.0001) return false;
	if (abs(way1.odom.longitude_min - way2.odom.longitude_min) > 0.0001) return false;
	if (way1.odom.latitude_deg != way2.odom.latitude_deg) return false;
	if (way1.odom.longitude_deg != way2.odom.longitude_deg) return false;
	return true;
}

inline void Layer2::waypoint_assign(waypoint & way1, const waypoint & way2)
{
	way1.search = way2.search;
	way1.odom.latitude_deg = way2.odom.latitude_deg;
	way1.odom.longitude_deg = way2.odom.longitude_deg;
	way1.odom.latitude_min = way2.odom.latitude_min;
	way1.odom.longitude_min = way2.odom.longitude_min;
}

void Layer2::long_meter_mins(const odom & cur_odom) {
	long_meter_in_minutes = 60 / (EARTH_CIRCUM * 
		cos(degree_to_radian(cur_odom.latitude_deg, cur_odom.latitude_min)) / 360);
}

void Layer2::go_to_ball(odom & cur_odom, TennisBall & ball) {
	// while ( ball.distance > BALL_THRESH) {
		while (ball.found && abs(ball.bearing) > DIRECTION_THRESH) {
			turn(cur_odom, ball.bearing);
			ball = this->tennis_ball_.clone();
		}

		while (ball.distance > BALL_THRESH) {
			layer1.drive_forward(cur_odom, ball.bearing, ball.distance);
			cur_odom = this->cur_odom_.clone_when_changed();
			ball = this->tennis_ball_.clone();
		}
	// }
} // go_to_ball()

// REQUIRES: obstacle_dummy_odom != location of an obstacle
	// assuming CV gives us a valid angle to drive to
void Layer2::obstacle_avoidance(odom & cur_odom, Obstacle & obs) {
	while (obs.detected) {
		turn(cur_odom, obs.bearing);
		obs = this->obstacle_.clone();
	}

	double dist_around_obs = CV_THRESH * sin(degree_to_radian(obs.bearing));
	// obstacle_layer1(lcm_);
	rover_msgs::Odometry around_obs = cur_odom;
	obstacle_dummy_odom(around_obs, cur_odom.bearing_deg, dist_around_obs);

	while(layer1.translational(cur_odom, around_obs)) {
		cur_odom = this->cur_odom_.clone_when_changed();
		this->obstacle_.clone_conditional( [&](const Obstacle & new_obs) {
			return new_obs.detected != obs.detected;
		}, &obs);
		if (obs.detected) obstacle_avoidance(cur_odom, obs);
	}

} // obstacle_avoidance()

// EFFECTS: turns rover by the bearing offset amount, then drives forward the given distance
void Layer2::turn(odom & cur_odom, double bearing_offset) {
	double desired_bearing = cur_odom.bearing_deg + bearing_offset;
	while (abs(cur_odom.bearing_deg - desired_bearing) > DIRECTION_THRESH) {
		layer1.turn_to_bearing(cur_odom, cur_odom.bearing_deg + bearing_offset);
		cur_odom = this->cur_odom_.clone_when_changed();
	}
} // turn()

void Layer2::obstacle_dummy_odom(odom & new_odom, const double cur_bearing, const double dist) {
	new_odom.latitude_min += sin(degree_to_radian(cur_bearing)) * dist * LAT_METER_IN_MINUTES;
	new_odom.longitude_min += sin(degree_to_radian(cur_bearing)) * dist * long_meter_in_minutes;
} // obstacle_dummy_odom()
