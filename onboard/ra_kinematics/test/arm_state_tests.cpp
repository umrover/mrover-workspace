#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.h"

#include <iostream>

using namespace nlohmann;
using namespace std;

// int main() {
//     cout << "c++ fucking sucks";
// }

TEST(initialization_test) {
    json geom;
    ArmState arm = ArmState(geom);
    arm.get_joint_com("joint_a");
    // ASSERT_EQUAL(arm.get_joint_axis("joint_a"), 0);
}

TEST_MAIN()