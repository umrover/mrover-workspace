enum class State {
	off,					// 0
	turn_and_drive,			// 10
	search_face0,			// 20
	search_turn120,			// 21
	search_turn240,			// 22
	search_turn360,			// 23
	search_turn_and_drive,	// 24 
	turn_to_ball,			// 28
	drive_to_ball,			// 29
	turn_around_obs,		// 30
	drive_around_obs,		// 31
	done					// 1
}

#include <cassert>

// private member variables
odom rover_cur_odom;
TennisBall rover_ball;
Obstacle rover_obstacle;
CourseData rover_course;
AutonState rover_auton_state;
odom dummy_obs_odom;
odom search_center;
bool search_mode;


void Layer2::run() {

	State state = off;
	while (true) {
		State nextState = state;
		switch (state) {
			case off: {
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
				search_mode = false;
				// based on current location, determine the longitude conversion
				long_meter_mins(rover_cur_odom);
				// initialize course information
				this->completed_wps = 0;
				this->total_wps = rover_course.overall.size();

				if (rover_ball.found) {
					nextState = turn_to_ball;
					break;
				} // if ball found

				else if (rover_obstacle.detected) {
					nextState = turn_around_obs;
					dummy_obs_way.search = false;
					break;
				} // else if obstacle detected

				else if (rover_course.overall.empty()) {
					nextState = done;
					break;
				} // else if no more waypoints to go to
				
				else {
					nextState = turn_and_drive;
					break;
				} // else turn to face the next waypoint
			} // state = off

			case turn_and_drive: {
				make_and_publish_nav_status(10);
				search_mode = false;
				const waypoint& goal = rover_course.overall.front(); // caution: reference vs value

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				else if (rover_course.overall.empty()) {
					nextState = done;
					break;
				} // if no more waypoints

				/*
				// if put back in, caution with case of getting to a ball then coming to this state
				else if (rover_ball.found) {
					nextState = ball_found;
					break;
				} // if ball found
				*/

				else if (rover_obstacle.detected) {
					nextState = turn_around_obs;
					dummy_obs_way = false;
					break;
				} // else if obstacle detected

				// drive and update rover
				bool arrived = layer1.translational(rover_cur_odom, goal.odom)
				updateRover();

				if (!arrived) {
					nextState = state;
					break;
				} // if at goal odom

				else {
					rover_course.overall.pop();

					if (goal.search) {
						nextState = search_face0;
						break;
					} // if the current waypoint is a search point

					else {
						nextState = state;
					} // else go to next waypoint
				} // else if at goal odom
			} // state = turn_and_drive

			case search_face0: {
				make_and_publish_nav_status(20);
				search_mode = true;

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = ball_found;
					break;
				} // if ball found

				else if(rover_obstacle.detected) {
					nextState =obstacle_detected;
					break;
				} // else if obstacle detected

				else if (rotation_pred(rover_cur_odom, 90, 90)) {
       				nextState = search_turn120;
       				break;
				} // if facing 90 (90 + 0) within threshold

				else {
					layer1.turn_to_bearing(rover_cur_odom, 90);
					updateRover();
					nextState = state;
					break;
				} // else
			} // state = search_face0

			case search_turn120: {
				make_and_publish_nav_status(21);

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = ball_found;
					break;
				} // if ball found

				else if(rover_obstacle.detected) {
					nextState =obstacle_detected;
					break;
				} // else if obstacle detected

				else if (rotation_pred(rover_cur_odom, 210, 210)) {
       				nextState = search_turn240;
       				break;
				} // if facing 210 (90 + 120) within threshold

				else {
					layer1.turn_to_bearing(rover_cur_odom, 210);
					updateRover();
					nextState = state;
					break;
				} // else
			} // state = search_turn120

			case search_turn240: {
				make_and_publish_nav_status(22);

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = ball_found;
					break;
				} // if ball found

				else if(rover_obstacle.detected) {
					nextState = obstacle_detected;
					break;
				} // else if obstacle detected

				else if (rotation_pred(rover_cur_odom, 330, 330)) {
       				nextState = search_turn360;
       				break;
				} // if facing 330 (90 + 240) within threshold

				else {
					layer1.turn_to_bearing(rover_cur_odom, 330);
					updateRover();
					nextState = state;
					break;
				} // else
			} // state = search_turn240

			case search_turn360: {
				make_and_publish_nav_status(23);

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = ball_found;
					break;
				} // if ball found

				else if(rover_obstacle.detected) {
					nextState =obstacle_detected;
					break;
				} // else if obstacle detected

				else if (rotation_pred(rover_cur_odom, 90, 90)) {
       				nextState = search_turn_and_drive;
       				init_search_multipliers();
       				search_center = rover_cur_odom;
       				break;
				} // if facing 90 (90 + 0) within threshold

				else {
					layer1.turn_to_bearing(rover_cur_odom, 90);
					updateRover();
					nextState = state;
					break;
				} // else
			} // state = search_turn360

			case search_turn_and_drive: {
				make_and_publish_nav_status(24);

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				else if (rover_ball.found) {
					nextState = ball_found;
					break;
				} // if ball found

				else if (rover_obstacle.detected) {
					nextState =obstacle_detected;
					break;
				} // else if obstacle detected

				else {
					if (search.empty()) add_four_points_to_search(search_center);
					const odom& goal = search.front();
					layer1.translational(rover_cur_odom, goal);
					nextState = state;
				} // else keep searching
			} // state = search_turn_and_drive

			case turn_to_ball: {
				make_and_publish_nav_status(28);

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				// TODO: if lost ball
				assert(rover_ball.found);

				else if (abs(rover_ball.bearing) > DIRECTION_THRESH) {
					turn(rover_cur_odom, rover_ball.bearing);
					updateRover_ballUnconditional();
					newState = state;
					break;
				} // else if bearing relative to ball != 0 within threshold

				else {
					newState = drive_to_ball;
					break;
				} // else we are facing the ball
			} // state = turn_to_ball

			case drive_to_ball: {
				make_and_publish_nav_status(29);

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				// TODO: if ball lost
				assert(rover_ball.found);

				/*
				if (rover_obstacle.detected) {
					dummy_obs_way.search = ????????
					nextState = turn_around_obs;
					break;
				} // else if obstacle detected
				*/

				else if (rover_ball.distance > BALL_THRESH) {
					layer1.drive_forward(rover_cur_odom, rover_ball.bearing, rover_ball.distance);
					updateRover_ballUnconditional();
					newState = state;
					break;
				} // else if distance to ball is not withing threshold

				else {
					newState = turn_and_drive;
					break;
				} // else if we are at the ball
			} // state = drive_to_ball

			case turn_around_obs: {
				make_and_publish_nav_status(30);

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off
				
				else if (!rover_obstacle.detected) {
					nextState = drive_around_obs;
					double dist_around_obs = CV_THRESH * sin(degree_to_radian(obs.bearing));
					dummy_obs_way.odom = cur_odom;
					obstacle_dummy_odom(dummy_obs_way.odom, cur_odom.bearing_deg, dist_around_obs);
					break;
				} // else if we are facing a clear path

				else {
					double desired_bearing = rover_cur_odom.bearing + rover_obstacle.bearing;
					if (desired_bearing > 360) desired_bearing -= 360;
					else if (desired_bearing < 0) desired_bearing += 360;

					layer1.turn_to_bearing(rover_cur_odom, desired_bearing);
					updateRover_obsUnconditional();
					break;
				} // else there is still an obstacle in front of rover
			} // state = turn_around_obs

			case drive_around_obs: {
				make_and_publish_nav_status(31);

				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				else if (rover_obstacle.detected) {
					nextState = turn_around_obs;
					break;
				} // else if obstacle detected

				bool arrived = layer1.translational(rover_cur_odom, dummy_obs_odom)
				updateRover();

				if (!arrived) {
					nextState = state;
					break;
				} // if not at dummy odom

				else {
					// if in search mode, return to search
					if (search_mode) nextState = search_turn_and_drive;
					else nextState = turn_and_drive;
					break;
				} // if at dummy odom
			} // state = drive_around_obs

			case done: {
				make_and_publish_nav_status(1);
				if (!rover_auton_state.is_auton) {
					nextState = off;
					break;
				} // if rover turned off

				else {
					rover_auton_state = this->auton_state_.clone_when_changed();
					nextState = state;
					break;
				} // else done
			} // state = done

		} // switch
		state = nextState;
	} // while
} // run()





Layer2::updateRover() {
	rover_cur_odom = this->cur_odom_.clone_when_changed();
	BallUpdate updateBall(&rover_ball);
	this->tennis_ball_.clone_conditional(updateBall, &rover_ball);
	ObstacleUpdate updateObs(&rover_obstacle);
	this->obstacle_.clone_conditional(updateObs, &rover_obstacle);
	AutonStateUpdate updateAuton(&rover_auton_state);
	this->auton_state_.clone_conditional(updateAuton, &rover_auton_state);
} // updateRover()

Layer2::updateRover_ballUnconditional() {
	rover_cur_odom = this->cur_odom_.clone_when_changed();
	BallUpdate updateBall(&rover_ball);
	this->tennis_ball_.clone();
	ObstacleUpdate updateObs(&rover_obstacle);
	this->obstacle_.clone_conditional(updateObs, &rover_obstacle);
	AutonStateUpdate updateAuton(&rover_auton_state);
	this->auton_state_.clone_conditional(updateAuton, &rover_auton_state);
} // updateRover_ballUnconditional()

Layer2::updateRover_obsUnconditional() {
	rover_cur_odom = this->cur_odom_.clone_when_changed();
	BallUpdate updateBall(&rover_ball);
	this->tennis_ball_.clone_conditional(updateBall, &rover_ball);
	ObstacleUpdate updateObs(&rover_obstacle);
	this->obstacle_.clone();
	AutonStateUpdate updateAuton(&rover_auton_state);
	this->auton_state_.clone_conditional(updateAuton, &rover_auton_state);
} // updateRover_ballUnconditional()

class BallUpdate {
private: 
	TennisBall * prev_ball;

public:
	BallUpdate(const TennisBall * ball) : prev_ball(ball) {}

	bool operator()(const TennisBall & new_ball) const {
		return new_ball.found != prev_ball.found;
	}
};

class ObstacleUpdate {
private:
	Obstacle * prev_obs;

public:
	ObstacleUpdate(const Obstacle * obs) : prev_obs(obs) {}

	bool operator()(const Obstacle & new_obs) const {
		return new_obs.detected != obs.detected;
	}
};

class AutonStateUpdate {
private:
	AutonState * prev_state;

public:
	AutonStateUpdate(const AutonState * state) : prev_state(state) {}

	bool operator()(const AutonState & new_state) const {
		return new_state.is_auton != prev_state.is_auton;
	}
};

/* Future TODOs
fix setting goal waypoint at beginning of turn_and_drive and search_turn_and_drive
separate translational into turning state and driving state
fix turn_and_drive to use a reference to goal not a copy (pop would invalidate goal if it was a reference as is)
clean up cloning at the end of states
if we see an object while turning in turn and drive, will we try to avoid it?
obs/ball update clone when changed
*/
