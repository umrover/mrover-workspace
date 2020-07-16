#include "arm_state.hpp"
#include "json.hpp"
#include <iostream>

using namespace nlohmann;
using namespace std;

ArmState::ArmState(json &geom) {
    cout << "new ArmState object created" << '\n';
}

double ArmState::get_joint_axis(string joint) {
    cout << "joint axis for joint: " << joint;
    return 0;
}

double ArmState::get_joint_com(string joint) {
    cout << "joint com for joint: " << joint;
    return 0;
}

double ArmState::get_joint_mass(string joint) {
    cout << "joint mass for joint: " << joint;
    return 0;
}


