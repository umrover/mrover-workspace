#include "motion_planner.hpp"
#include <random>
#include <deque>


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

Vector6d MotionPlanner::sample() {

    Vector6d z_rand;

    for (int i = 0; i < all_limits.size(); ++i) {
        std::uniform_real_distribution<double> distr(all_limits[i]["lower"], all_limits[i]["upper"]);
        std::default_random_engine eng;

        z_rand(i) = (distr(eng));
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


Vector6d MotionPlanner::steer(MotionPlanner::Node* start, Vector6d end) {
    Vector6d line_vec = end - start->config;

    for (int i = 0; i < step_limits.size(); ++i) {
        if (step_limits[i] - abs(line_vec(i)) >= 0) {
            return end;
        }
    }

    double min_t = numeric_limits<double>::max();

    Vector6d new_config = start->config;

    //parametrize the line
    for (int i = 0; i < line_vec.size(); ++i) {
      
        // TODO can line_vec[i] be 0?
        double t = step_limits[i] / abs(line_vec[i]);
        if (t < min_t) {
            min_t = t;
        }
    }

    for (int i = 0; i < line_vec.size(); ++i) {
        new_config(i) += min_t * line_vec[i];
    }

    return new_config;
}

vector<Vector6d> MotionPlanner::backtrace_path(MotionPlanner::Node* end, MotionPlanner::Node* root) {
    vector<Vector6d> path;

    // starting at end, add each position config to path
    while (end != root) {
        path.push_back(get_radians(end->config));
        end = end->parent;
    }

    // include the config for the root node
    path.push_back(get_radians(end->config));
    
    return path;
}

Vector6d MotionPlanner::get_radians(Vector6d config) {
    for (int i = 0; i < 6; ++i) {
        config(i) *= (M_PI / 180);
    }

    return config;
}

MotionPlanner::Node* MotionPlanner::extend(Node* tree, Vector6d z_rand) {
    inc_i();
    Node* z_nearest = nearest(tree, z_rand);
    Vector6d z_new = steer(z_nearest, z_rand);

    vector<double> z_new_angs;
    for (int i = 0; i < 6; ++i) {
        z_new_angs.push_back(z_new(i));
    }
    if (!solver.is_safe(z_new_angs)){
        // Can we use return NULL?
        return NULL;
    }
    Node* new_node = &Node(z_new);
    new_node->parent = z_nearest;
    z_nearest->children.push_back(new_node);
    new_node->cost = z_nearest->cost + (z_nearest->config - z_new).norm();
    return new_node;
}

MotionPlanner::Node* MotionPlanner::connect(Node* tree, Vector6d a_new) {
    Node* extension = extend(tree, a_new);
    Vector6d config = extension->config;

    while (config)
}

// How do we substitute scipy for this method?
// tk::spline MotionPlanner::spline_fitting(vector<double> path){}
