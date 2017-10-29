#include <cmath>
#include <chrono>
#include <lcm/lcm-cpp.hpp>

#define PI 3.1415926
#define JOYSTICK_CHANNEL "/joystick"

class GPS {
public:
    double latitude;
    double longitude; 
    GPS() : latitude(0), longitude(0) {}
};

// function to calculate bearing from location 1 to location 2
// GPS latitude and longitude in degrees
// return bearing in degrees
double calc_bearing(const GPS &gps1, const GPS &gps2);

// function to turn rover to face destination
// GPS latitude and longitude in degrees
void turn_to_destination(const GPS &cur_gps, const GPS &dest_gps, const double time, const double threshold);

// function to drive to destination
// GPS latitude and longitude in degrees
// distance: how much to distance to move in one iteration
// threshold: to accept and stop moving certain difference within current location and destination
void go_setpoint(const GPS &dest_gps, const double time, const double gps_threshold, const double bearing_threshold);

lcm::LCM lcm;

Joystick make_joystick_msg(const double forward_back, const double left_right, const bool kill) {
	Joystick js;
	js.forward_back = forward_back;
	js.left_right = left_right;
	js.kill = kill;
	return js;
}

void publish_joystick_for(const std::chrono::milliseconds ms, const Joystick &js)
{
    std::chrono::time_point<std::chrono::system_clock> end;
    end = std::chrono::system_clock::now() + ms; 
    while(std::chrono::system_clock::now() < end) 
    {
        lcm.publish(JOYSTICK_CHANNEL, &js);
    }
}

void publish_zero_joystick() {
	lcm.publish(make_joystick_msg(0, 0, false));
}	

// assuming that left is 1
void turn_left(const double time) {
    // publish_joystick_for(time, make_joystick_msg(0.5, 0.5, false));
	publish_joystick_for(time, make_joystick_msg(0, 1, false));
	publish_zero_joystick();
}

// assuming that right is -1
void turn_right(const double time) {
    // publish_joystick_for(time, make_joystick_msg(0.5, -0.5, false));
	publish_joystick_for(time, make_joystick_msg(0, -1, false));
	publish_zero_joystick();
}

// function tp move rover forward by certain distance
// send joystick input for certain time
void drive_forward(const double time) {
    publish_joystick_for(time, make_joystick_msg(1, 0, false));
	publish_zero_joystick();
}

// function tp move rover backward by certain distance
// send joystick input for certain time
void drive_backward(const double time) {
    publish_joystick_for(time, make_joystick_msg(-1, 0, false));
    publish_zero_joystick();
}

// convert degree to radian
double degree_to_radian(const double x) {
    return x * PI / 180;
}

// convert radian to degree
double radian_to_degree(const double x) {
    return x * 180 / PI;
}

// prototypes not implemented
// just abstraction

// function to get current bearing
double get_current_bearing() {
    return 0;
}

// function to turn rover by this angle
// angle in degree
// send joystick input for certain time
void turn_rover(const double angle, const double time) {
	if(angle > 0) {
		turn_left(time);
	}
	else if(angle < 0) {
		turn_right(time);
	}
}

// functin to get current gps
GPS get_current_gps() {
    return GPS();
}

double calc_bearing(const GPS &gps1, const GPS &gps2) {
    double lat1 = degree_to_radian(gps1.latitude);
    double long1 = degree_to_radian(gps1.longitude);
    double lat2 = degree_to_radian(gps2.latitude);
    double long2 = degree_to_radian(gps2.longitude);

    double y = std::sin(long2 - long1) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(long2 - long1);

    return radian_to_degree(std::atan2(y, x));
}

void turn_to_destination(const GPS &cur_gps, const GPS &dest_gps, const double time, const double threshold) {
    double cur_bear = get_current_bearing();
    double dest_bearing = calc_bearing(cur_gps, dest_gps);
    while(abs(cur_bear - dest_bearing) > threshold) {
    	turn_rover(dest_bearing - cur_bear, time);
    	cur_bear = get_current_bearing();
    }
    return;
}

bool within_threshold(const GPS &gps1, const GPS &gps2, const double threshold) {
    return (std::abs(gps1.latitude - gps2.latitude) <= threshold ) && (std::abs(gps1.longitude - gps2.longitude) <= threshold);
}

void go_setpoint(const GPS &dest_gps, const double time, const double gps_threshold, const double bearing_threshold) {
    // current gps location
    GPS cur_gps = get_current_gps();

    // turn rover
    turn_to_destination(cur_gps, dest_gps, time, bearing_threshold);

    while(!within_threshold(dest_gps, cur_gps, gps_threshold)) {
        // move rover
        drive_forward(time);

        // readjust rover bearing
        cur_gps = get_current_gps();

        turn_to_destination(cur_gps, dest_gps, time, bearing_threshold);
    }

    return;
}

int main() {
    return 0;
}
