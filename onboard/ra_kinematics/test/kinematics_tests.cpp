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




TEST_MAIN()