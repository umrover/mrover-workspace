#include "unit_test_framework.h"
#include "nlohmann/json.hpp"
#include "../arm_state.hpp"
#include "../utils.hpp"

#include <iostream>

using namespace nlohmann;

TEST(arm_initialization_test) {
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
}

TEST(read_geom_file) {
    json geom = read_json_from_file(get_mrover_arm_geom());

    ASSERT_EQUAL(geom["name"], "mrover_arm");
}

TEST(joint_creation_test) {
    json geom = read_json_from_file(get_mrover_arm_geom());

    json joint_a_json = geom["joints"]["joint_a"];

    ArmState arm = ArmState(geom);

    Vector3d joint_c_local(-0.05840, 0.58039, 0.009525);
    Vector3d joint_c_com(0, 0, 0);
    Vector3d joint_c_axis(1, 0, 0);

    double joint_c_lower_limit = -3.01;
    double joint_c_upper_limit = 2.73;

    ASSERT_EQUAL(joint_c_local, arm.get_joint_pos_local(2));
    ASSERT_EQUAL(joint_c_com, arm.get_joint_com(2));
    ASSERT_EQUAL(joint_c_axis, arm.get_joint_axis(2));

    ASSERT_EQUAL(joint_c_lower_limit, arm.get_joint_limits(2)[0]);
    ASSERT_EQUAL(joint_c_upper_limit, arm.get_joint_limits(2)[1]);
}

TEST(avoidance_link_creation_test) {
    json geom = read_json_from_file(get_mrover_arm_geom());

    json joint_a_json = geom["joints"]["joint_a"];

    ArmState arm = ArmState(geom);
}

TEST(set_joint_angles_test) {
    // Create the angles vector that will be used to test
    // TODO: Modify the length of set_angles as necessary
    std::vector<double> set_angles{1.1, 0.9, -0.5, 0.1, 0.01, -1.2};
    
    // Create the arm object:
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);

    // Set the angles on the arm to be from the set_angles vector
    arm.set_joint_angles(set_angles);
    // Retrieve the new arm angles and make sure they are identical to set_angles
    std::vector<double> return_angles = arm.get_joint_angles();
    // print contents of return_angles:
    for (double x : return_angles) {
        std::cout << x << "\n";
    }

    for (int i = 0; i < 6; ++i) {
        ASSERT_EQUAL(set_angles[i], return_angles[i]);
    }
}

TEST(json_read_test) {
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
}


TEST_MAIN()
