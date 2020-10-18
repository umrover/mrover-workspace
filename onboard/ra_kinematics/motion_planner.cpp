#include "motion_planner.hpp"
#include <random>
#include <deque>

using namespace eigen;

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

vector<double> MotionPlanner::sample() {

    vector<double> z_rand;

    for (int i = 0; i < all_limits.size(); ++i) {
        std::uniform_real_distribution<double> distr(all_limits[i]["lower"], all_limits[i]["upper"]);
        std::default_random_engine eng;

        z_rand.push_back(distr(eng));
    }

    return z_rand;
}

MotionPlanner::Node* MotionPlanner::nearest(MotionPlanner::Node* tree_root, Vector6d rand) {

    deque<Node*> q;
    q.push_back(tree_root);

    double min_dist = numeric_limits<double>::max();
    Node* min_node = nullptr;

    while (!q.empty()) {
        Node* node = q.front();
        q.pop_front();

        double dist = (node->config - rand).norm();

        if (dist < min_dist) {
          min_dist = dist;
          min_node = node;
        }

        for (Node* child : node->children) {
          q.push_back(child);
        }
    }

    return min_node;
}
