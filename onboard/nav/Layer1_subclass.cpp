// November 26, 2017
// layer1 is a subclass of layer

#include "Layer1_subclass.h"
#include <iostream>

Layer1::Layer1 (Auton::System &sys, lcm::LCM &lcm_object) : 
    Auton::Layer(sys),
    lcm_(lcm_object) {
	// calls the base class constructor
}

void Layer1::run() {
    double dest_bearing = calc_bearing(cur_odom, goal_odom), cur_bearing = cur_odom.bearing_deg;
    std::cout << "bearing to target " << dest_bearing << ", current bearing " << cur_bearing << std::endl;
    double error = dest_bearing - cur_bearing;
    total_error += error;
    double prop = kp * error;
    double integ = ki * total_error;
    double deriv = kd * last_error;
    last_error = error;
    double effort = prop+integ+deriv;
    if (effort < -1) effort = -1;
    if (effort > 1) effort = 1;
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
        dest_long - start_long > 0 ? bearing_phi = degree_to_radian(90, 0) : bearing_phi = degree_to_radian(270, 0);
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




