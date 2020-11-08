#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.hpp"
#include "../kinematics.hpp"
#include "../motion_planner.hpp"
#include <lcm/lcm-cpp.hpp>
#include "../utils.hpp"

using namespace std;

TEST(init_test) {
    json geom;
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver(arm);

    lcm::LCM lcm;

    MotionPlanner planner = MotionPlanner(arm, lcm, solver);
}

TEST(rrt_connect_simple) {
    string config_path = getenv("MROVER_CONFIG");
    string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);

    Vector6d angles_start;
    angles_start(0) = 45;
    angles_start(1) = 45;
    angles_start(2) = 45;
    angles_start(3) = 45;
    angles_start(4) = 45;
    angles_start(5) = 45;
    arm.set_joint_angles(angles_start);

    KinematicsSolver solver = KinematicsSolver(arm);
    lcm::LCM lcm;

    solver.FK(arm);

    //MotionPlanner planner = MotionPlanner(arm, lcm, solver);
}

TEST(rrt_connect) {
    string config_path = getenv("MROVER_CONFIG");
    string geom_file = config_path + "/config_kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);

    ASSERT_EQUAL(geom["name"], "mrover_arm");

    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver(arm);

    lcm::LCM lcm;

    MotionPlanner planner = MotionPlanner(arm, lcm, solver);

    Vector6d start = arm.get_ef_pos_world();
    Vector6d target;
    target(0) = start(0) +1;
    target(1) = start(1) + 1;
    target(2) = start(2) + 1;
    target(3) = start(3) + 1;
    target(4) = start(4) + 1;
    target(5) = start(5) + 1;

    vector<tk::spline> splines = planner.rrt_connect(target);

    ASSERT_ALMOST_EQUAL(splines[0](0), start(0), 0.05);
    ASSERT_ALMOST_EQUAL(splines[1](0), start(1), 0.05);
    ASSERT_ALMOST_EQUAL(splines[2](0), start(2), 0.05);
    ASSERT_ALMOST_EQUAL(splines[3](0), start(3), 0.05);
    ASSERT_ALMOST_EQUAL(splines[4](0), start(4), 0.05);
    ASSERT_ALMOST_EQUAL(splines[5](0), start(5), 0.05);

    int spline_last = planner.getSplineSize() - 1;

    ASSERT_ALMOST_EQUAL(splines[0](spline_last), target(0), 0.05);
    ASSERT_ALMOST_EQUAL(splines[1](spline_last), target(1), 0.05);
    ASSERT_ALMOST_EQUAL(splines[2](spline_last), target(2), 0.05);
    ASSERT_ALMOST_EQUAL(splines[3](spline_last), target(3), 0.05);
    ASSERT_ALMOST_EQUAL(splines[4](spline_last), target(4), 0.05);
    ASSERT_ALMOST_EQUAL(splines[5](spline_last), target(5), 0.05);
}

TEST_MAIN()