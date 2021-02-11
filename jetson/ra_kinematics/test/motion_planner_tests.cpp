#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.hpp"
#include "../kinematics.hpp"
#include "../motion_planner.hpp"
#include <lcm/lcm-cpp.hpp>
#include "../utils.hpp"
#include <iostream>

using namespace std;

TEST(init_test) {
    json geom;
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    MotionPlanner planner = MotionPlanner(arm, solver);
}

TEST(rrt_connect_simple) {

    // read arm geometry
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";
    json geom = read_json_from_file(geom_file);

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();
    MotionPlanner planner = MotionPlanner(arm, solver);

    // set angles and confirm there are no collisions
    vector<double> set_angles{0, 1, 1, 0, 0, 0};
    //ASSERT_TRUE(solver.is_safe(arm, set_angles));
    arm.set_joint_angles(set_angles);
    solver.FK(arm);

    Vector6d target;

    // move target slightly away from starting position
    map<string, double> start = arm.get_joint_angles();
    target(0) = start["joint_a"] + 0.1;
    target(1) = start["joint_b"] + 0.1;
    target(2) = start["joint_c"] + 0.1;
    target(3) = start["joint_d"] + 0.1;
    target(4) = start["joint_e"] + 0.1;
    target(5) = start["joint_f"] + 0.1;

    // run rrt_connect and confirm it found a path
    ASSERT_TRUE(planner.rrt_connect(arm, target));

    // confirm spline positions
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start["joint_a"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[1], start["joint_b"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[2], start["joint_c"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[3], start["joint_d"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[4], start["joint_e"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[5], start["joint_f"], 0.01);

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[0], target(0), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[1], target(1), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[2], target(2), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[3], target(3), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[4], target(4), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[5], target(5), 0.01);
}


TEST(rrt_connect) {

    // read arm geometry
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";
    json geom = read_json_from_file(geom_file);

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();
    MotionPlanner planner = MotionPlanner(arm, solver);

    // set angles and confirm there are no collisions
    vector<double> set_angles{0, 1, 1, 0, 0, 0};
    //ASSERT_TRUE(solver.is_safe(arm, set_angles));
    arm.set_joint_angles(set_angles);
    solver.FK(arm);

    Vector6d target;

    // move target slightly away from starting position
    map<string, double> start = arm.get_joint_angles();
    target(0) = start["joint_a"] + -0.2;
    target(1) = start["joint_b"] + 0.1;
    target(2) = start["joint_c"] + 0.1;
    target(3) = start["joint_d"] + 0.1;
    target(4) = start["joint_e"] + 0.1;
    target(5) = start["joint_f"] + 0.1;

    // run rrt_connect and confirm it found a path
    ASSERT_TRUE(planner.rrt_connect(arm, target));

    // confirm spline positions
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start["joint_a"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[1], start["joint_b"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[2], start["joint_c"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[3], start["joint_d"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[4], start["joint_e"], 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[5], start["joint_f"], 0.01);

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[0], target(0), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[1], target(1), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[2], target(2), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[3], target(3), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[4], target(4), 0.01);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(1)[5], target(5), 0.01);
}

TEST_MAIN()