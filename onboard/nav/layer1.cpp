// November 26, 2017
// layer1 is a subclass of layer

#include "layer1.hpp"
#include <iostream>

Layer1::Layer1 (Auton::System &sys, lcm::LCM &lcm_object) : 
    Auton::Layer(sys),
    bearing_pid(0.05, 0, 0.0055),
    distance_pid(0.1, 0, 0),
    first(false),
    in_outer_thresh(false),
    in_inner_thresh(false),
    outer_thresh(OUTER_MIN),
    inner_thresh(INNER_MIN),
    lcm_(lcm_object) {
	// calls the base class constructor
}

void Layer1::run() {
    rover_msgs::Odometry cur_odom = this->cur_odom.clone();
    rover_msgs::Odometry goal_odom = this->goal_odom.clone();
    double dist = estimate_noneuclid(cur_odom, goal_odom);
    double bearing = calc_bearing(cur_odom, goal_odom);
    std::cout << "target at " << dist << " bearing " << bearing << ", rover heading " << cur_odom.bearing_deg << std::endl;
    calc_bearing_thresholds(cur_odom, dist, bearing); //recalculates inner and outer bearing thresholds for updated location
    if (dist < WITHIN_GOAL) {
        this->pause();
        std::cout << "found it" << std::endl;
    } else if (!first) { //turns rover to face target before any motion
        turn_to_dest(cur_odom, goal_odom);
        if (in_inner_thresh) {
            first = true;
        }
    }
    else if (!in_outer_thresh) { //if outside outer threshold, turn rover back to inner threshold
        turn_to_dest(cur_odom, goal_odom); 
    } else {
        //drives rover forward
        double effort = distance_pid.update(-1 * dist, 0);
        rover_msgs::Joystick joy = make_joystick_msg(effort, 0, false);
        lcm_.publish(JOYSTICK_CHANNEL, &joy);
    }
}

void Layer1::calc_bearing_thresholds(const rover_msgs::Odometry &cur_odom,
        const double distance, const double bearing){
    outer_thresh = atan2(WITHIN_GOAL, distance);
    if(outer_thresh < OUTER_MIN) outer_thresh = OUTER_MIN; //min outer_thresh value

    inner_thresh = atan2(AT_GOAL, distance);
    if(inner_thresh < INNER_MIN) inner_thresh = INNER_MIN; //min inner_thresh value

    in_outer_thresh = (abs(cur_odom.bearing_deg - bearing) < outer_thresh);
    in_inner_thresh = (abs(cur_odom.bearing_deg - bearing) < inner_thresh);
}

void Layer1::turn_to_dest(
        const rover_msgs::Odometry &cur_odom,
        const rover_msgs::Odometry &goal_odom) {
    double dest_bearing = calc_bearing(cur_odom, goal_odom);
    double cur_bearing = cur_odom.bearing_deg;
    double effort = bearing_pid.update(cur_bearing, dest_bearing);
    rover_msgs::Joystick joy = make_joystick_msg(0, effort, false);
    lcm_.publish(JOYSTICK_CHANNEL, &joy);
}

double Layer1::calc_bearing(const rover_msgs::Odometry &start, 
							const rover_msgs::Odometry &dest) {
    double start_lat = degree_to_radian(start.latitude_deg, start.latitude_min);
    double start_long = degree_to_radian(start.longitude_deg, start.longitude_min); 
    double dest_lat = degree_to_radian(dest.latitude_deg, dest.latitude_min);
    double dest_long = degree_to_radian(dest.longitude_deg, dest.longitude_min);


    double vert_component_dist = earthRadiusMeters * std::sin(dest_lat - start_lat);
    double noneuc_dist = estimate_noneuclid(start,dest);

    double bearing_phi = std::acos(vert_component_dist / noneuc_dist);
    if (start_long > dest_long) bearing_phi = 2 * PI - bearing_phi;

    if(vert_component_dist < 1 && vert_component_dist > -1){
        if (start_long < dest_long) {
            bearing_phi = PI/2.0;
        } else {
            bearing_phi = 3.0*PI/2.0;
        }
    }

    return radian_to_degree(bearing_phi);
}

double Layer1::estimate_noneuclid(const rover_msgs::Odometry &start, 
								  const rover_msgs::Odometry &dest){
    double start_lat = degree_to_radian(start.latitude_deg, start.latitude_min);
    double start_long = degree_to_radian(start.longitude_deg, start.longitude_min); 
    double dest_lat = degree_to_radian(dest.latitude_deg, dest.latitude_min);
    double dest_long = degree_to_radian(dest.longitude_deg, dest.longitude_min);

    double dlon = (dest_long - start_long) * cos((start_lat + dest_lat) / 2);
    dlon *= dlon;
    double dlat = (dest_lat - start_lat);
    dlat *= dlat;
    return sqrt(dlon + dlat)*earthRadiusMeters;
}

rover_msgs::Joystick Layer1::make_joystick_msg(const double forward_back, const double left_right,
								   const bool kill) {
    rover_msgs::Joystick js;
    js.forward_back = forward_back;
    js.left_right = left_right;
    js.kill = kill;
    return js;
}

inline double Layer1::degree_to_radian(const double x, const double minute) {
    return (PI / 180) * (x + minute / 60.0);
}

inline double Layer1::radian_to_degree(const double x) {
    //pls fix
    return x * 180 / PI;
}

bool Layer1::within_threshold(const rover_msgs::Odometry &odom1, 
							  const rover_msgs::Odometry &odom2, 
							  const double threshold) {
    return estimate_noneuclid(odom1,odom2) <= threshold;
}




