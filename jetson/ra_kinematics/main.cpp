#include "json.hpp"
#include "arm_state.hpp"
#include "utils.hpp"


#include <lcm/lcm-cpp.hpp>
#include <iostream>

using nlohmann::json;
using namespace std;

int main() 
{
    cout << "INITIALIZING KINEMATICS FOR ROBOT ARM\n";
    string config_path = getenv("MROVER_CONFIG");
    string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

    lcm::LCM lcmObject;
    json geom = read_json_from_file(geom_file);

    ArmState arm = ArmState(geom);

    arm.get_joint_mass("joint_a");


    return 0;
}