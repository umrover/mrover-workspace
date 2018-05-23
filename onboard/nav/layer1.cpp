#include "layer1.hpp"
#include <iostream>

Layer1::Layer1 (lcm::LCM & lcm_object) : 
    bearing_pid(0.1, 0.00001, 0.0055),
    distance_pid(0.2, 0, 0),
    first(false),
    in_outer_thresh(false),
    in_inner_thresh(false),
    outer_thresh(OUTER_MIN),
    inner_thresh(3), //rover somehow functions better with 0.00001 thresh. with small threshold, rover doesn't overshoot
    lcm_(lcm_object) {}
    // calls the base class constructor


bool Layer1::turn(const odom & current_odom, const odom & target_odom) {
    // calculate bearing threshold for turning to face target
    double dist = estimate_noneuclid(current_odom, target_odom);
    double bearing = calc_bearing(current_odom, target_odom);
    std::cout << "bearing to target: " << bearing << " deg" << std::endl;
    calc_bearing_thresholds(current_odom, dist, bearing);

    // turns rover to face target before any motion
    make_publish_joystick(0, turn_to_dest(current_odom, target_odom), false);

    // if within inner threshold, done initial turn
    if (in_inner_thresh) return true;
    return false;
} // turn()

bool Layer1::drive(const odom & current_odom, const odom & target_odom) {
    // calculate distance to the goal
    double dist = estimate_noneuclid(current_odom, target_odom);
    double bearing = calc_bearing(current_odom, target_odom);
    calc_bearing_thresholds(current_odom, dist, bearing);

    if (dist < AT_GOAL) {
        return true;
    } // if at the goal
    else if (!in_outer_thresh) {
        make_publish_joystick(0, turn_to_dest(current_odom, target_odom), false);
        std::cout << "shits fucked up, this should not be happening in this bizach\n";
        // TODO throw exception to go back to big turn
        throw 1;
    } // if outside out threshold, turn rover back to inner threshold

    else {
        double effort = distance_pid.update(-1 * dist, 0);
        make_publish_joystick(effort, turn_to_dest(current_odom, target_odom), false);
    } // drives rover forward
    return false;
} // drive()

/*
//public methods -- called by layer2
bool Layer1::translational(const odom & current_odom, const odom & target_odom) {
    double dist = estimate_noneuclid(current_odom, target_odom);
    double bearing = calc_bearing(current_odom, target_odom);

    calc_bearing_thresholds(current_odom, dist, bearing); //recalculates inner and outer bearing thresholds for updated location

    if (dist < WITHIN_GOAL) {
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
} // translational()
*/

void Layer1::turn_to_bearing(const odom & curnt_odom, double desired_bearing) {
    // rover_msgs::Joystick joy = make_joystick_msg(0, turn_to_dest(current_position, desired_bearing), false);
    // lcm_.publish(JOYSTICK_CHANNEL, &joy);
    make_publish_joystick(0, turn_to_dest(curnt_odom, desired_bearing), false);
} // turn_to_bearing()

void Layer1::drive_forward(const odom & cur_odom, const double bearing_offset, const double dist_to_target) {
    double turn_effort = turn_to_dest(cur_odom, cur_odom.bearing_deg + bearing_offset);
    double forward_effort = distance_pid.update(-1 * dist_to_target, 0);
    make_publish_joystick(forward_effort, turn_effort, false);
} // drive_forward()

//private methods implemented below
void Layer1::calc_bearing_thresholds(const odom & cur_odom,
        const double distance, const double bearing) {
    outer_thresh = atan2(WITHIN_GOAL, distance);
    if(outer_thresh < OUTER_MIN) outer_thresh = OUTER_MIN; //min outer_thresh value

    inner_thresh = atan2(AT_GOAL, distance);
    if(inner_thresh < INNER_MIN) inner_thresh = INNER_MIN; //min inner_thresh value

    in_outer_thresh = (abs(cur_odom.bearing_deg - bearing) < outer_thresh);
    in_inner_thresh = (abs(cur_odom.bearing_deg - bearing) < inner_thresh);

    // std::cout << cur_odom.bearing_deg << " " << bearing << "\n";
} // calc_bearing_thresholds()

double Layer1::turn_to_dest(const odom & cur_odom, const odom &goal_odom) {
    double dest_bearing = calc_bearing(cur_odom, goal_odom);
    double cur_bearing = cur_odom.bearing_deg;
    throughZero(dest_bearing, cur_bearing);
    return bearing_pid.update(cur_bearing, dest_bearing);
} // turn_to_dest()

double Layer1::turn_to_dest(const odom & cur_odom, double angle ) {
    double cur_bearing = cur_odom.bearing_deg;
    throughZero(angle, cur_bearing);
    return bearing_pid.update(cur_bearing, angle);
} // turn_to_dest()

void Layer1::throughZero(double & dest_bearing, const double cur_bearing) {
    assert(cur_bearing >= 0 && cur_bearing <= 360);
    if (fabs(cur_bearing - dest_bearing) > 180) {
        if (cur_bearing < 180) dest_bearing = (dest_bearing - 360);
        else dest_bearing = dest_bearing + 360;
    }
}   

// rover_msgs::Joystick Layer1::make_joystick_msg(const double forward_back, const double left_right, const bool kill) {}

void Layer1::make_publish_joystick(const double forward_back, const double left_right, const bool kill) {
    rover_msgs::Joystick joys;
    joys.dampen = -1.0; // power limit (0 = 50%, 1 = 0%, -1 = 100% power)
    joys.forward_back = -forward_back;
    joys.left_right = left_right;
    joys.kill = kill;
    lcm_.publish(JOYSTICK_CHANNEL, &joys);
} // make_publish_joystick()
