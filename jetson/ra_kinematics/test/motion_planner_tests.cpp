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
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);

    vector<double> angles_start;
    angles_start.push_back(0);
    angles_start.push_back(0);
    angles_start.push_back(0);
    arm.set_joint_angles(angles_start);

    KinematicsSolver solver = KinematicsSolver();

    MotionPlanner planner = MotionPlanner(arm, solver);

    cout << planner.get_spline_pos(0.0)[0] << ' '
         << planner.get_spline_pos(0.0)[1] << ' '
         << planner.get_spline_pos(0.0)[2] << '\n';
    
    cout << planner.get_spline_pos(1.0)[0] << ' '
         << planner.get_spline_pos(1.0)[1] << ' '
         << planner.get_spline_pos(1.0)[2] << '\n';
    
}

/*
TEST(rrt_connect) {
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    MotionPlanner planner = MotionPlanner(arm, solver);

    Vector6d start = arm.get_ef_pos_world();
    Vector6d target;
    target(0) = start(0) +1;
    target(1) = start(1) + 1;
    target(2) = start(2) + 1;
    target(3) = start(3) + 1;
    target(4) = start(4) + 1;
    target(5) = start(5) + 1;

    planner.rrt_connect(arm, target);

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start(0), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start(1), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start(2), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start(3), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start(4), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start(5), 0.05);

    int spline_last = planner.getSplineSize() - 1;

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[spline_last], target(0), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[spline_last], target(1), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[spline_last], target(2), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[spline_last], target(3), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[spline_last], target(4), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[spline_last], target(5), 0.05);
}*/

TEST_MAIN()