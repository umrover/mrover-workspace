#include "layer2.hpp"
// TODO latitude vs longitude calc minutes to meters -- done??

// custom constructor for layer2
Layer2::Layer2(lcm::LCM &lcm_object) : 
    state(State::off), lcm_(lcm_object), layer1(lcm_), 
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
		state = State::off;
		return true;
	});	
}

bool Layer2::rotation_pred(const odom & cur_odom, const double theta1, const double theta2){
	return (abs(cur_odom.bearing_deg - theta1) < INNER_SEARCH_THRESH || 
			abs(cur_odom.bearing_deg - theta2) < INNER_SEARCH_THRESH);
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

// EFFECTS: turns rover by the bearing offset amount, then drives forward the given distance
void Layer2::turn(odom & cur_odom, double bearing_offset) {
	double desired_bearing = cur_odom.bearing_deg + bearing_offset;
	while (abs(cur_odom.bearing_deg - desired_bearing) > DIRECTION_THRESH) {
		layer1.turn_to_bearing(cur_odom, cur_odom.bearing_deg + bearing_offset);
		cur_odom = this->cur_odom_.clone_when_changed();
	} // while
} // turn()


void Layer2::obstacle_dummy_odom(odom & new_odom, const double cur_bearing, const double dist) {
	new_odom.latitude_min += sin(degree_to_radian(cur_bearing)) * dist * LAT_METER_IN_MINUTES;
	new_odom.longitude_min += sin(degree_to_radian(cur_bearing)) * dist * long_meter_in_minutes;
} // obstacle_dummy_odom()

void Layer2::updateRover() {
	rover_cur_odom = this->cur_odom_.clone_when_changed();
	BallUpdate updateBall(&rover_ball);
	this->tennis_ball_.clone_conditional(updateBall, &rover_ball);
	ObstacleUpdate updateObs(&rover_obstacle);
	this->obstacle_.clone_conditional(updateObs, &rover_obstacle);
	AutonStateUpdate updateAuton(&rover_auton_state);
	this->auton_state_.clone_conditional(updateAuton, &rover_auton_state);
} // updateRover()

void Layer2::updateRover_ballUnconditional() {
	rover_cur_odom = this->cur_odom_.clone_when_changed();
	BallUpdate updateBall(&rover_ball);
	this->tennis_ball_.clone();
	ObstacleUpdate updateObs(&rover_obstacle);
	this->obstacle_.clone_conditional(updateObs, &rover_obstacle);
	AutonStateUpdate updateAuton(&rover_auton_state);
	this->auton_state_.clone_conditional(updateAuton, &rover_auton_state);
} // updateRover_ballUnconditional()

void Layer2::updateRover_obsUnconditional() {
	rover_cur_odom = this->cur_odom_.clone_when_changed();
	BallUpdate updateBall(&rover_ball);
	this->tennis_ball_.clone_conditional(updateBall, &rover_ball);
	ObstacleUpdate updateObs(&rover_obstacle);
	this->obstacle_.clone();
	AutonStateUpdate updateAuton(&rover_auton_state);
	this->auton_state_.clone_conditional(updateAuton, &rover_auton_state);
} // updateRover_ballUnconditional()

void Layer2::run() {
	State state = State::off;
	while (true) {
		State nextState = state;
		switch (state) {
			case State::off: {
				make_and_publish_nav_status(0);
				rover_auton_state = this->auton_state_.clone();

				if (!rover_auton_state.is_auton) {
					rover_auton_state = this->auton_state_.clone_when_changed();
				} // if auton state is off

				// rover_course.overall.clear(); ???
				rover_course = this->course_data_.clone();
				rover_cur_odom = this->cur_odom_.clone();
				rover_ball = this->tennis_ball_.clone();
				rover_obstacle = this->obstacle_.clone();
				// based on current location, determine the longitude conversion
				long_meter_mins(rover_cur_odom);
				// initialize course information
				this->completed_wps = 0;
				this->total_wps = rover_course.overall.size();

				if (rover_ball.found) {
					nextState = State::turn_to_ball;
				} // if ball found

				else if (rover_obstacle.detected) {
					nextState = State::turn_around_obs;
				} // else if obstacle detected

				else if (rover_course.overall.empty()) {
					nextState = State::done;
				} // else if no more waypoints to go to
				
				else {
					nextState = State::turn_and_drive;
				} // else turn to face the next waypoint
				break;
			} // state = off

			case State::turn_and_drive: {
				make_and_publish_nav_status(10);
				const waypoint& goal = rover_course.overall.front(); // caution: reference vs value

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
					break;
				} // if rover turned off

				else if (rover_course.overall.empty()) {
					nextState = State::done;
					break;
				} // if no more waypoints

				/*
				// if put back in, caution with case of getting to a ball then coming to this state
				else if (rover_ball.found) {
					nextState = turn_to_ball;
					break;
				} // if ball found
				*/

				else if (rover_obstacle.detected) {
					nextState = State::turn_around_obs;
					break;
				} // else if obstacle detected

				// drive and update rover
				bool arrived = layer1.translational(rover_cur_odom, goal.odom);
				updateRover();

				if (!arrived) {
					nextState = state;
				} // if at goal odom

				else {
					rover_course.overall.pop_front();

					if (goal.search) {
						nextState = State::search_face0;
					} // if the current waypoint is a search point

					else {
						nextState = state;
					} // else go to next waypoint
				} // else if at goal odom
				break;
			} // state = turn_and_drive

			case State::search_face0: {
				make_and_publish_nav_status(20);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = State::turn_to_ball;
				} // if ball found

				else if (rotation_pred(rover_cur_odom, 90, 90)) {
       				nextState = State::search_turn120;
				} // if facing 90 (90 + 0) within threshold

				else {
					layer1.turn_to_bearing(rover_cur_odom, 90);
					updateRover();
					nextState = state;
				} // else
				break;
			} // state = search_face0

			case State::search_turn120: {
				make_and_publish_nav_status(21);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = State::turn_to_ball;
				} // if ball found

				else if (rotation_pred(rover_cur_odom, 210, 210)) {
       				nextState = State::search_turn240;
				} // if facing 210 (90 + 120) within threshold

				else {
					layer1.turn_to_bearing(rover_cur_odom, 210);
					updateRover();
					nextState = state;
				} // else
				break;
			} // state = search_turn120

			case State::search_turn240: {
				make_and_publish_nav_status(22);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = State::turn_to_ball;
				} // if ball found

				else if (rotation_pred(rover_cur_odom, 330, 330)) {
       				nextState = State::search_turn360;
				} // if facing 330 (90 + 240) within threshold

				else {
					layer1.turn_to_bearing(rover_cur_odom, 330);
					updateRover();
					nextState = state;
				} // else
				break;
			} // state = search_turn240

			case State::search_turn360: {
				make_and_publish_nav_status(23);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = State::turn_to_ball;
				} // if ball found

				else if (rotation_pred(rover_cur_odom, 90, 90)) {
       				nextState = State::search_turn_and_drive;
       				init_search_multipliers();
       				search_center.odom = rover_cur_odom;
       				search_center.search = false;
				} // if facing 90 (90 + 0) within threshold

				else {
					layer1.turn_to_bearing(rover_cur_odom, 90);
					updateRover();
					nextState = state;
				} // else
				break;
			} // state = search_turn360

			case State::search_turn_and_drive: {
				make_and_publish_nav_status(24);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = State::turn_to_ball;
				} // if ball found

				else if (rover_obstacle.detected) {
					nextState = State::search_turn_around_obs;
				} // else if obstacle detected

				else {
					if (search.empty()) add_four_points_to_search(search_center);
					const odom& goal = search.front().odom;
					layer1.translational(rover_cur_odom, goal);
					nextState = state;
				} // else keep searching
				break;
			} // state = search_turn_and_drive

			case State::turn_to_ball: {
				make_and_publish_nav_status(28);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
					break;
				} // if rover turned off

				// TODO: if lost ball
				assert(rover_ball.found);

				if (abs(rover_ball.bearing) > DIRECTION_THRESH) {
					turn(rover_cur_odom, rover_ball.bearing);
					updateRover_ballUnconditional();
					nextState = state;
					break;
				} // else if bearing relative to ball != 0 within threshold

				else {
					nextState = State::drive_to_ball;
					break;
				} // else we are facing the ball
			} // state = turn_to_ball

			case State::drive_to_ball: {
				make_and_publish_nav_status(29);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
					break;
				} // if rover turned off

				// TODO: if ball lost
				assert(rover_ball.found);

				/*
				if (rover_obstacle.detected) {
					dummy_obs_way.search = ????????
					nextState = search_turn_around_obs;
					break;
				} // else if obstacle detected
				*/

				if (rover_ball.distance > BALL_THRESH) {
					layer1.drive_forward(rover_cur_odom, rover_ball.bearing, rover_ball.distance);
					updateRover_ballUnconditional();
					nextState = state;
				} // else if distance to ball is not withing threshold

				else {
					nextState = State::turn_and_drive;
				} // else if we are at the ball
				break;
			} // state = drive_to_ball

			case State::turn_around_obs: 
			case State::search_turn_around_obs: {
				make_and_publish_nav_status(30);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
				} // if rover turned off
				
				else if (!rover_obstacle.detected) {
					if (state == State::search_turn_around_obs) {
						nextState = State::search_drive_around_obs;
					} // if in search mode
					else {
						nextState = State::drive_around_obs;
					}
					double dist_around_obs = CV_THRESH * sin(degree_to_radian(rover_obstacle.bearing));
					dummy_obs_odom = rover_cur_odom;
					obstacle_dummy_odom(dummy_obs_odom, rover_cur_odom.bearing_deg, dist_around_obs);
				} // else if we are facing a clear path

				else {
					double desired_bearing = rover_cur_odom.bearing_deg + rover_obstacle.bearing;
					if (desired_bearing > 360) desired_bearing -= 360;
					else if (desired_bearing < 0) desired_bearing += 360;
					layer1.turn_to_bearing(rover_cur_odom, desired_bearing);
					updateRover_obsUnconditional();
					nextState = state;
				} // else there is still an obstacle in front of rover
				break;
			} // state = turn_around_obs

			case State::drive_around_obs: 
			case State::search_drive_around_obs: {
				make_and_publish_nav_status(31);

				if (!rover_auton_state.is_auton) {
					nextState = State::off;
					break;
				} // if rover turned off

				else if (rover_obstacle.detected) {
					nextState = State::turn_around_obs;
					break;
				} // else if obstacle detected

				bool arrived = layer1.translational(rover_cur_odom, dummy_obs_odom);
				updateRover();

				if (!arrived) {
					nextState = state;
				} // if not at dummy odom

				else {
					// if in search mode, return to search
					if (state == State::search_drive_around_obs) nextState = State::search_turn_and_drive; // TODO
					else nextState = State::turn_and_drive;
				} // if at dummy odom
			} // state = drive_around_obs

			case State::done: {
				make_and_publish_nav_status(1);
				if (!rover_auton_state.is_auton) {
					nextState = State::off;
				} // if rover turned off

				else {
					rover_auton_state = this->auton_state_.clone_when_changed();
					nextState = state;
				} // else done
				break;
			} // state = done
		} // switch
		state = nextState;
	} // while
} // run()

/* Future TODOs
two states for one, remove bool search_mode;
fix setting goal waypoint at beginning of turn_and_drive and search_turn_and_drive
separate translational into turning state and driving state
fix turn_and_drive to use a reference to goal not a copy (pop would invalidate goal if it was a reference as is)
clean up cloning at the end of states
if we see an object while turning in turn and drive, will we try to avoid it?
obs/ball update clone when changed
*/
