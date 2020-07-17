#include "arm_state.hpp"
#include "json.hpp"
#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix4d;
using namespace nlohmann;
using namespace std;

ArmState::ArmState(json &geom) : ef_pos_world(Vector3d::Zero()), ef_xform(Matrix4d::Identity()) {}

double ArmState::get_joint_axis(string joint) {
    cout << "joint axis for joint: " << joint << "\n";
    return 0;
}

double ArmState::get_joint_com(string joint) {
    cout << "joint com for joint: " << joint << "\n";
    return 0;
}

double ArmState::get_joint_mass(string joint) {
    cout << "joint mass for joint: " << joint << "\n";
    cout << "end effector position: \n" << ef_pos_world << "\n";
    return 0;
}

void ArmState::set_joint_angles(vector<double> angles) {
    // TODO
}


