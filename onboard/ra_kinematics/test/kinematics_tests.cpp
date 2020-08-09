#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.hpp"
#include "../kinematics.hpp"

#include <iostream>


TEST(initialization_test) {
    json geom;
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver(arm);
}

TEST(fk_test) {
    // Create the arm to be tested on:
    json geom;
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver(arm);
    
}

TEST(apply_joint_xform_test) {}

TEST(ik_test) {}

TEST(ik_step_test) {}

TEST(is_safe_test) {}

TEST(limit_check_test) {}

TEST_MAIN()