#include "layer1.hpp"
#include <iostream>

Layer1::Layer1 (lcm::LCM & lcm_object) : 
    bearing_pid(0.05, 0, 0.0055),
    distance_pid(0.1, 0, 0),
    first(false),
    in_outer_thresh(false),
    in_inner_thresh(false),
    outer_thresh(OUTER_MIN),
    inner_thresh(.00001), //rover somehow functions better with 0.00001 thresh. with small threshold, rover doesn't overshoot
    lcm_(lcm_object) {}
    // calls the base class constructor

//public methods -- called by layer2
bool Layer1::translational(const odom & current_odom, const odom & target_odom)
{
    double dist = estimate_noneuclid(current_odom, target_odom);
    double bearing = calc_bearing(current_odom, target_odom);

    std::cout << "current bearing: " << bearing << " || target bearing: " << 
    current_odom.bearing_deg << " || target at distance of: " << dist << " m\n";

    calc_bearing_thresholds(current_odom, dist, bearing); //recalculates inner and outer bearing thresholds for updated location
    if (dist < WITHIN_GOAL) {
        std::cout << "reached [intermediate] target" << std::endl;
        this->first = false;
        return true;
    } 
    else if (!this->first) {  //turns rover to face target before any motion
        make_publish_joystick(0, turn_to_dest(current_odom, target_odom), false);
        if (in_inner_thresh) this->first = true;
    }
    else if (!in_outer_thresh) { //if outside outer threshold, turn rover back to inner threshold
        make_publish_joystick(0, turn_to_dest(current_odom, target_odom), false);
        std::cout << "shits fucked up, this should not be happening in this bizach\n";
    } 
    else { //drives rover forward
        double effort = distance_pid.update(-1 * dist, 0);
        make_publish_joystick(effort, turn_to_dest(current_odom, target_odom), false);
    }
    return false;
}

void Layer1::turn_to_bearing(const odom & curnt_odom, double desired_bearing)
{
    // rover_msgs::Joystick joy = make_joystick_msg(0, turn_to_dest(current_position, desired_bearing), false);
    // lcm_.publish(JOYSTICK_CHANNEL, &joy);
    make_publish_joystick(0, turn_to_dest(curnt_odom, desired_bearing), false);
}


//private methods implemented below
void Layer1::calc_bearing_thresholds(const odom & cur_odom,
        const double distance, const double bearing)
{
    outer_thresh = atan2(WITHIN_GOAL, distance);
    if(outer_thresh < OUTER_MIN) outer_thresh = OUTER_MIN; //min outer_thresh value

    inner_thresh = atan2(AT_GOAL, distance);
    if(inner_thresh < INNER_MIN) inner_thresh = INNER_MIN; //min inner_thresh value

    in_outer_thresh = (abs(cur_odom.bearing_deg - bearing) < outer_thresh);
    in_inner_thresh = (abs(cur_odom.bearing_deg - bearing) < inner_thresh);
}

double Layer1::turn_to_dest(const odom & cur_odom, const odom &goal_odom)
{
    double dest_bearing = calc_bearing(cur_odom, goal_odom);
    double cur_bearing = cur_odom.bearing_deg;
    std::cout << "before: " << dest_bearing;
    throughZero(dest_bearing, cur_bearing);
    std::cout << " after: " << dest_bearing << std::endl;
    return bearing_pid.update(cur_bearing, dest_bearing);
}

double Layer1::turn_to_dest(const odom & cur_odom, double angle )
{
    double cur_bearing = cur_odom.bearing_deg;
    std::cout << "before: " << angle;
    throughZero(angle, cur_bearing);
    std::cout << " after: " << angle << std::endl;
    return bearing_pid.update(cur_bearing, angle);
}

void Layer1::throughZero(double & dest_bearing, const double cur_bearing)
{
    assert(cur_bearing >= 0 && cur_bearing <= 360);
    if(abs(cur_bearing - dest_bearing) > 180) {
        if(cur_bearing < 180) dest_bearing = (dest_bearing - 360);
        else dest_bearing = dest_bearing + 360;
    }
}   

// rover_msgs::Joystick Layer1::make_joystick_msg(const double forward_back, const double left_right, const bool kill) {}

void Layer1::make_publish_joystick(const double forward_back, const double left_right, const bool kill)
{
    rover_msgs::Joystick joys;
    joys.forward_back = forward_back;
    joys.left_right = left_right;
    joys.kill = kill;
    lcm_.publish(JOYSTICK_CHANNEL, &joys);
}
