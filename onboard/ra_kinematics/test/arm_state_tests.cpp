#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.hpp"
#include "../utils.hpp"

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

TEST(read_geom_file) {
    string config_path = "../../config";
    string geom_file = config_path + "/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    ASSERT_EQUAL(geom["name"], "mrover_arm");
}

TEST_MAIN()