#include "json.hpp"
#include "arm_state.hpp"
#include "utils.hpp"


#include <lcm/lcm-cpp.hpp>
#include <iostream>

using nlohmann::json;
using namespace std;

int main() 
{
    string config_path = getenv("MROVER_CONFIG");
    string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

    lcm::LCM lcmObject;
    json geom = read_json_from_file(geom_file);

    cout << config_path;

    ArmState arm = ArmState(geom);

    string joint = "joint_a";
    arm.get_joint_axis(joint);
    return 0;
}