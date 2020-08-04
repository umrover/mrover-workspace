#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.hpp"
#include "../utils.hpp"

#include <iostream>

using namespace nlohmann;
using namespace std;

TEST(initialization_test) {
    json geom;
    ArmState arm = ArmState(geom);
}

TEST(read_geom_file) {
    // string config_path = "../../config";
    // string geom_file = config_path + "/kinematics/mrover_arm_geom.json";
    string config_path = getenv("MROVER_CONFIG");
    string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    ASSERT_EQUAL(geom["name"], "mrover_arm");
}

TEST(joint_creation_test) {
    string config_path = getenv("MROVER_CONFIG");
    string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

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
    string config_path = getenv("MROVER_CONFIG");
    string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    json joint_a_json = geom["joints"]["joint_a"];

    ArmState arm = ArmState(geom);
}



TEST_MAIN()