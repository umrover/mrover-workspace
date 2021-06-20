#include "unit_test_framework.h"
#include "nlohmann/json.hpp"
#include "../arm_state.hpp"
#include "../kinematics.hpp"
#include "../motion_planner.hpp"
#include <lcm/lcm-cpp.hpp>
#include "../utils.hpp"
#include <iostream>

using namespace std;

TEST(motion_planner_init) {
    // read arm geometry
    json geom = read_json_from_file(get_mrover_arm_geom());

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();
    MotionPlanner planner = MotionPlanner(arm, solver);
    
    vector<double> set_angles{0, 1, 1, 0, 0, 0};

    ASSERT_TRUE(solver.is_safe(arm, set_angles));
}


TEST(rrt_connect_simple) {

    // read arm geometry
    json geom = read_json_from_file(get_mrover_arm_geom());

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();
    MotionPlanner planner = MotionPlanner(arm, solver);

    // set angles and confirm there are no collisions
    vector<double> set_angles{0, 1, 1, 0, 0, 0};
    ASSERT_TRUE(solver.is_safe(arm, set_angles));
    arm.set_joint_angles(set_angles);
    solver.FK(arm);

    // move target slightly away from starting position
    map<string, double> start = arm.get_joint_angles();

    vector<double> target;
    target.resize(6);

    target[0] = start["joint_a"] + 0.1;
    target[1] = start["joint_b"] + 0.1;
    target[2] = start["joint_c"] + 0.1;
    target[3] = start["joint_d"] - 0.1;
    target[4] = start["joint_e"] - 0.1;
    target[5] = start["joint_f"] - 0.1;

    ASSERT_TRUE(solver.is_safe(arm, target));

    Vector6d target6d = vecTo6d(target);

    cout << "entering rrt...\n";

    // run rrt_connect and confirm it found a path
    ASSERT_TRUE(planner.rrt_connect(arm, target6d));

    // confirm spline positions
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start["joint_a"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[1], start["joint_b"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[2], start["joint_c"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[3], start["joint_d"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[4], start["joint_e"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[5], start["joint_f"], 0.01);

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[0], target6d(0), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[1], target6d(1), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[2], target6d(2), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[3], target6d(3), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[4], target6d(4), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[5], target6d(5), 0.01);
}

/**
 * target is the same as starting postition
 * */
TEST(anti_motion_planner) {
    // read arm geometry
    json geom = read_json_from_file(get_mrover_arm_geom());

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();
    MotionPlanner planner = MotionPlanner(arm, solver);

    // set angles and confirm there are no collisions
    vector<double> set_angles{0, 1, 1, 0, 0, 0};
    ASSERT_TRUE(solver.is_safe(arm, set_angles));
    arm.set_joint_angles(set_angles);
    solver.FK(arm);

    // move target slightly away from starting position
    map<string, double> start = arm.get_joint_angles();

    vector<double> target;
    target.resize(6);

    target[0] = start["joint_a"];
    target[1] = start["joint_b"];
    target[2] = start["joint_c"];
    target[3] = start["joint_d"];
    target[4] = start["joint_e"];
    target[5] = start["joint_f"];

    ASSERT_TRUE(solver.is_safe(arm, target));

    Vector6d target6d = vecTo6d(target);

    // run rrt_connect and confirm it found a path
    ASSERT_TRUE(planner.rrt_connect(arm, target6d));

    // confirm spline positions
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start["joint_a"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[1], start["joint_b"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[2], start["joint_c"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[3], start["joint_d"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[4], start["joint_e"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[5], start["joint_f"], 0.01);

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[0], target6d(0), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[1], target6d(1), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[2], target6d(2), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[3], target6d(3), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[4], target6d(4), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[5], target6d(5), 0.01);
}


TEST(rrt_connect) {
    // read arm geometry
    json geom = read_json_from_file(get_mrover_arm_geom());

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();
    MotionPlanner planner = MotionPlanner(arm, solver);

    // set angles and confirm there are no collisions
    vector<double> set_angles{1, 0, 0, -1, 0, 0};
    ASSERT_TRUE(solver.is_safe(arm, set_angles));
    arm.set_joint_angles(set_angles);
    solver.FK(arm);

    // move target slightly away from starting position
    map<string, double> start = arm.get_joint_angles();

    vector<double> target;
    target.resize(6);

    target[0] = start["joint_a"] + 0.1;
    target[1] = start["joint_b"] + 0.1;
    target[2] = start["joint_c"] + 0.1;
    target[3] = start["joint_d"] - 0.1;
    target[4] = start["joint_e"] - 0.1;
    target[5] = start["joint_f"] - 0.1;

    ASSERT_TRUE(solver.is_safe(arm, target));

    Vector6d target6d = vecTo6d(target);

    // run rrt_connect and confirm it found a path
    ASSERT_TRUE(planner.rrt_connect(arm, target6d));

    // confirm spline positions
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start["joint_a"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[1], start["joint_b"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[2], start["joint_c"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[3], start["joint_d"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[4], start["joint_e"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[5], start["joint_f"], 0.01);

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[0], target6d(0), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[1], target6d(1), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[2], target6d(2), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[3], target6d(3), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[4], target6d(4), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[5], target6d(5), 0.01);
    
    arm.set_joint_angles(target);

    // move target slightly away from starting position
    start = arm.get_joint_angles();

    target[0] = start["joint_a"] - 0.05;
    target[1] = start["joint_b"] - 0.05;
    target[2] = start["joint_c"] - 0.05;
    target[3] = start["joint_d"] + 0.05;
    target[4] = start["joint_e"] + 0.05;
    target[5] = start["joint_f"] + 0.05;

    ASSERT_TRUE(solver.is_safe(arm, target));

    target6d = vecTo6d(target);

    // run rrt_connect and confirm it found a path
    ASSERT_TRUE(planner.rrt_connect(arm, target6d));

    // confirm spline positions
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start["joint_a"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[1], start["joint_b"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[2], start["joint_c"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[3], start["joint_d"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[4], start["joint_e"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[5], start["joint_f"], 0.01);

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[0], target6d(0), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[1], target6d(1), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[2], target6d(2), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[3], target6d(3), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[4], target6d(4), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[5], target6d(5), 0.01);
}
TEST_MAIN()