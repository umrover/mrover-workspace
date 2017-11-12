#include <cmath>
#include <tgmath.h>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Odometry.hpp"
#include <cassert>

#define earthRadiusMeters 6371000
#define PI 3.141592654
#define JOYSTICK_CHANNEL "/joystick"

// function to calculate bearing from location 1 to location 2
// Odometry latitude_deg and longintude_deg in degrees
// return bearing in degrees
double calc_bearing(const rover_msgs::Odometry &odom1, const rover_msgs::Odometry &odom2);

double estimate_noneuclid(const rover_msgs::Odometry &start, const rover_msgs::Odometry &dest);

// function to turn rover to face destination
// Odometry latitude_deg and longintude_deg in degrees
void turn_to_destination(const rover_msgs::Odometry &cur_odom, const rover_msgs::Odometry &dest_odom, const double time, const double threshold);

// function to drive to destination
// Odometry latitude_deg and longintude_deg in degrees
// distance: how much to distance to move in one iteration
// threshold: to accept and stop moving certain difference within current location and destination
void go_setpoint(const rover_msgs::Odometry &dest_odom, const double time, const double odom_threshold, const double bearing_threshold);

lcm::LCM lcm;

Joystick make_joystick_msg(const double forwardesback, const double left_right, const bool kill) {
    Joystick js;
    js.forwardesback = forwardesback;
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
inline double degree_to_radian(const double x) {
    return x * PI / 180;
}

// convert radian to degree
inline double radian_to_degree(const double x) {
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

double calc_bearing(const rover_msgs::Odometry &start, const rover_msgs::Odometry &dest) {
    double strt_lat = degree_to_radian(start.latitude_deg);
    double strt_long = degree_to_radian(start.longintude_deg);
    double dest_lat = degree_to_radian(dest.latitude_deg);
    double dest_long = degree_to_radian(dest.longintude_deg);


    double vert_component_dist = earthRadiusMeters * std::sin(dest_lat - strt_lat);
    double noneuc_dist = estimate_noneuclid(start,dest);

    double bearing_phi = std::acos(vert_component_dist / noneuc_dist);

    if(vert_component_dist < 1 && vert_component_dist > -1){
        dest_long - strt_long > 0 ? bearing_phi = degree_to_radian(90) : bearing_phi = degree_to_radian(270);
    }

    return bearing_phi;
  
}

void turn_to_destination(const rover_msgs::Odometry &cur_odom, const rover_msgs::Odometry &dest_odom, const double time, const double threshold) {
    double cur_bear = get_current_bearing();
    double dest_bearing = calc_bearing(cur_odom, dest_odom);
    while(abs(cur_bear - dest_bearing) > threshold) {
        turn_rover(dest_bearing - cur_bear, time);
        cur_bear = get_current_bearing();
    }
    return;
}

double estimate_noneuclid(const rover_msgs::Odometry &start, const rover_msgs::Odometry &dest){
    double st_lat = degree_to_radian(start.lat);
    double st_long = degree_to_radian(start.long); 
    double des_lat = degree_to_radian(dest.lat);
    double des_long = degree_to_radian(dest.long);

    double dlon = (des_long - st_long) * cos((st_lat + des_lat) / 2);
    dlon *= dlon;
    double dlat = (des_lat - st_lat);
    dlat *= dlat;

    assert(dlon > 0 && dlat > 0);
    return sqrt(dlon + dlat)*earthRadiusMeters;
}

bool within_threshold(const rover_msgs::Odometry &odom1, const rover_msgs::Odometry &odom2, const double threshold) {
    return estimate_noneuclid(odom1,odom2) <= threshold;
}

void go_setpoint(const rover_msgs::Odometry &dest_odom, const double time, const double odom_threshold, const double bearing_threshold) {
    // current odom location
    rover_msgs::Odometry cur_odom = get_current_odom();

    // turn rover
    turn_to_destination(cur_odom, dest_odom, time, bearing_threshold);

    while(!within_threshold(dest_odom, cur_odom, odom_threshold)) {
        // move rover
        drive_forward(time);

        // readjust rover bearing
        cur_odom = get_current_odom();

        turn_to_destination(cur_odom, dest_odom, time, bearing_threshold);
    }

    return;
}

int main() {
    return 0;
}