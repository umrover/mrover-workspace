#include "motion_planner.hpp"
#include <cmath>

MotionPlanner::MotionPlanner(ArmState robot_state_in, lcm::LCM& lcm_in, KinematicsSolver solver_in) :
        robot(robot_state_in),
        lcm(lcm_in),
        solver(solver_in) {
    
    // add limits for each joint to all_limits, after converting to degrees
    for (string& joint : robot.get_all_joints()) {
      map<string, double> limits = robot.get_joint_limits(joint);

      limits["lower"] = limits["lower"] * 180 / M_PI;
      limits["upper"] = limits["upper"] * 180 / M_PI;

      all_limits.push_back(limits);
    }

    step_limits.push_back(1);
    step_limits.push_back(1);
    step_limits.push_back(2);
    step_limits.push_back(3);
    step_limits.push_back(5);
    step_limits.push_back(1);

    neighbor_dist = 3;
    max_iterations = 1000;
    i = 0;
}