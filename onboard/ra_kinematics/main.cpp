#include "json.hpp"
#include "arm_state.h"

#include <lcm/lcm-cpp.hpp>
#include <iostream>

using nlohmann::json;
using namespace std;

int main() 
{

    lcm::LCM lcmObject;
    json geom;

    ArmState arm = ArmState();

    string joint = "joint_a";
    arm.get_joint_axis(joint);
    return 0;
}