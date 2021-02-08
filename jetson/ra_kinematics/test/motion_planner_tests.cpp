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
    string geom_file = "/vagrant/config/kinematics/mrover_arm_test_geom.json";

    json geom = read_json_from_file(geom_file);
    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);

    vector<double> angles_start;
    angles_start.push_back(0);
    angles_start.push_back(0);
    angles_start.push_back(0);
    arm.set_joint_angles(angles_start);

    KinematicsSolver solver = KinematicsSolver();
    solver.FK(arm);

    MotionPlanner planner = MotionPlanner(arm, solver);

    cout << arm.get_ef_pos_world()(0) << ' '
         << arm.get_ef_pos_world()(1) << ' '
         << arm.get_ef_pos_world()(2) << '\n';

    cout << arm.get_ef_ang_world()(0) << ' '
         << arm.get_ef_ang_world()(1) << ' '
         << arm.get_ef_ang_world()(2) << '\n';
    
    
/*
    cout << planner.get_spline_pos(0.0)[0] << ' '
         << planner.get_spline_pos(0.0)[1] << ' '
         << planner.get_spline_pos(0.0)[2] << '\n';
    
    cout << planner.get_spline_pos(1.0)[0] << ' '
         << planner.get_spline_pos(1.0)[1] << ' '
         << planner.get_spline_pos(1.0)[2] << '\n';*/
    
}


TEST(rrt_connect) {
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    vector<double> set_angles{0.1, 0.4, -0.1, 0.1, 0.01, -0.2};
    ASSERT_TRUE(solver.is_safe(arm, set_angles));
    arm.set_joint_angles(set_angles);
    solver.FK(arm);


    MotionPlanner planner = MotionPlanner(arm, solver);

    map<string, double> start = arm.get_joint_angles();

    cout << "joint a start: " << start["joint_a"] << "\n";
    cout << "joint b start: " << start["joint_b"] << "\n";
    cout << "joint c start: " << start["joint_c"] << "\n";
    cout << "joint d start: " << start["joint_d"] << "\n";
    cout << "joint e start: " << start["joint_e"] << "\n";
    cout << "joint f start: " << start["joint_f"] << "\n";
    
    Vector6d target;
    target(0) = start["joint_a"] + 0.1;
    target(1) = start["joint_b"] + 0.1;
    target(2) = start["joint_c"] + 0.1;
    target(3) = start["joint_d"] + 0.1;
    target(4) = start["joint_e"] + 0.1;
    target(5) = start["joint_f"] + 0.1;


    if (planner.rrt_connect(arm, target)) {
        cout << "PATH FOUND\n";
    }
    else {
        cout << "path NOT found\n";
    }
    
    int spline_last = planner.getSplineSize() - 1;

    cout << "get_spline_pos(0)[0]: " << planner.get_spline_pos(0)[0] << "\n";
    cout << "get_spline_pos(spline_last)[0]: " << planner.get_spline_pos(spline_last)[0] << "\n\n";

    cout << "get_spline_pos(0)[1]: " << planner.get_spline_pos(0)[1] << "\n";
    cout << "get_spline_pos(spline_last)[1]: " << planner.get_spline_pos(spline_last)[1] << "\n";

    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[0], start["joint_a"], 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[1], start["joint_b"], 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[2], start["joint_c"], 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[3], start["joint_d"], 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[4], start["joint_e"], 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(0)[5], start["joint_f"], 0.05);

/*
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(spline_last)[0], target(0), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(spline_last)[1], target(1), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(spline_last)[2], target(2), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(spline_last)[3], target(3), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(spline_last)[4], target(4), 0.05);
    ASSERT_ALMOST_EQUAL(planner.get_spline_pos(spline_last)[5], target(5), 0.05);*/
}

TEST_MAIN()