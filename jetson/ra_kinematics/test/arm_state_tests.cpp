#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.hpp"
#include "../utils.hpp"

#include <iostream>

using namespace nlohmann;
using namespace std;

TEST(arm_initialization_test) {
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
}

TEST(read_geom_file) {
    // string config_path = "../../config";
    // string geom_file = config_path + "/kinematics/mrover_arm_geom.json";
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    ASSERT_EQUAL(geom["name"], "mrover_arm");
}

TEST(joint_creation_test) {
    // TODO: Change configuration paths as needed.
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    json joint_a_json = geom["joints"]["joint_a"];

    ArmState arm = ArmState(geom);

    Vector3d joint_c_local(-0.09827958500, 0.37832483699999997, 0.015874163831999997);
    Vector3d joint_c_com(0, 0, 0);
    Vector3d joint_c_axis(1, 0, 0);

    double joint_c_lower_limit = -2.36;
    double joint_c_upper_limit = 2.36;

    ASSERT_EQUAL(joint_c_local, arm.get_joint_pos_local("joint_c"));
    ASSERT_EQUAL(joint_c_com, arm.get_joint_com("joint_c"));
    ASSERT_EQUAL(joint_c_axis, arm.get_joint_axis("joint_c"));

    ASSERT_EQUAL(joint_c_lower_limit, arm.get_joint_limits("joint_c")["lower"]);
    ASSERT_EQUAL(joint_c_upper_limit, arm.get_joint_limits("joint_c")["upper"]);
}

TEST(avoidance_link_creation_test) {
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    json joint_a_json = geom["joints"]["joint_a"];

    ArmState arm = ArmState(geom);
}

// TEST(delete_joints_test) {
//     // TODO: Change the configuration path as needed
//     string config_path = getenv("MROVER_CONFIG");
//     string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

//     json geom = read_json_from_file(geom_file);
//     ArmState* arm = new ArmState(geom);

//     // Todo: Change the number of joints as needed
//     ASSERT_EQUAL(6, arm->num_joints());
//     delete arm;
//     // Just tests to make sure there's no bugs in the destructor
// }

TEST(set_joint_angles_test) {
    // Create the angles vector that will be used to test
    // TODO: Modify the length of set_angles as necessary
    vector<double> set_angles{1.1, 0.9, -0.5, 0.1, 0.01, -1.2};
    
    // Create the arm object:
    // TODO: Modify the configuration path as necessary
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    ArmState arm = ArmState(geom);

    // Set the angles on the arm to be from the set_angles vector
    arm.set_joint_angles(set_angles);
    // Retrieve the new arm angles and make sure they are identical to set_angles
    map<string, double> return_angles_map = arm.get_joint_angles();
    vector<double> return_angles_vec;
    int ind = 0;
    // print contents of return_angles_map:
    for (auto it = return_angles_map.begin(); it != return_angles_map.end(); ++it) {
        cout << it->first << " " << it->second << "\n";
    }
    // Set return angles vec:
    for (auto it = return_angles_map.begin(); it != return_angles_map.end(); ++it) {
        return_angles_vec.push_back(it->second);
        ++ind;
    }
    for (int i = 0; i < 6; ++i) {
        ASSERT_EQUAL(set_angles[i], return_angles_vec[i]);
    }
}

TEST(link_link_check_test) {
    // TODO: Modify the configuration paths as necessary
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    // Links and joints automatically initialized at onset of arm creation
    ASSERT_EQUAL(arm.obstacle_free(), 0);
}

TEST(json_read_test) {
    // TODO: Modify the configuration paths as necessary
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
}



TEST_MAIN()