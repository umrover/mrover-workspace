#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.hpp"
#include "../utils.hpp"
#include "../kinematics.hpp"

#include <iostream>
#include <string>

using namespace nlohmann;
using namespace std;

/* 
Here are some varius tests we can use to test kinematics.cpp once it is written (using 2019 configuration):

NOTE: The algorithm should be able to approach the target point but will not
necessarily pass the safety check!

NOTE: These points were reached by setting random_angles = true and e_locked = false

Point 1: [0.3975877299536904,-0.595437721432015,0.05701726525706301,-2.888598515425053,0.5056772404722869,-0.2702381850292698]
            NOTE: This point returns and passes the safety check.

Point 2: [0.41735297576788866,-0.6301581425659601,-0.09231003891980295,-1.302881860149742,0.13410241193690717,-1.8221069366052942]
            NOTE: This point returns and passes the safety check.

Point 3: [0.43561896709482717,-0.5849202310118653,-0.23898894427981895,-0.19091988273941293,0.5705597354134033,-2.8999168062356357]
            Note: This point returns and passes the safety check

NOTE: For more testing points, see valid_configurations_2.csv located towards the end of the mrover directory.


*/


TEST(initialization_test) {
    string config_path = getenv("MROVER_CONFIG");
    string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";
    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    KinematicsSolver solver(arm);
}

TEST(fk_test) {
    // Create the arm to be tested on:
    // string config_path = getenv("MROVER_CONFIG");
    // string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

    // json geom = read_json_from_file(geom_file);
    // ArmState arm = ArmState(geom);
    // KinematicsSolver solver = KinematicsSolver(arm);
    
}

TEST(apply_joint_xform_test) {}

TEST(ik_test) {}

TEST(ik_step_test) {}

TEST(is_safe_test) {}

TEST(limit_check_test) {}

TEST_MAIN()