#ifndef MROVER_ARM_H
#define MROVER_ARM_H

#include "json.hpp"
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>
#include "rover_msgs/ArmPosition.hpp"
#include "rover_msgs/TargetOrientation.hpp"


using nlohmann::json;
using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

/**
 * This is the MRoverArm class, responsible for
 * initiating callback functions and sending calculations
 * over LCM.
*/
class MRoverArm {

    MRoverArm(lcm::LCM lcm, json config);

    void arm_position_callback(string channel, ArmPosition msg);

    void publish_config(Vector6d config, string channel);

    void publish_transforms();

    void target_orientation_callback(string channel, TargetOrientation msg);

    void plan_path(Vector6d goal);
};


#endif