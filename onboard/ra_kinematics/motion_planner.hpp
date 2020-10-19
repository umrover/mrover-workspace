#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <Eigen/Dense>
#include <vector>
#include "arm_state.hpp"
#include "kinematics.hpp"
#include <map>

#include <lcm/lcm-cpp.hpp>
#include "spline.h"

using namespace Eigen;
using namespace std;

/**
 * DO NOT ASSUME THIS INTERFACE IS CORRECT WHEN IMPLEMENTING
 * (Should be reviewed)
 * 
 * TODO check if Node is implemented properly
 * TODO Check if all functions need to be private
 * */
class MotionPlanner {
private:

    ArmState robot;
    KinematicsSolver solver;

    vector< map<string, double> > all_limits;

    vector<int> step_limits;

    int neighbor_dist;
    int max_iterations;
    int i;

    lcm::LCM& lcm;

    // Should Node be outside the MotionPlanner class
    class Node {
    friend class MotionPlanner;
    private:
        Vector6d config;
        
        Node* parent;
        vector<Node*> children;
        double cost;

    public:
        Node(vector<double> config_in) : config(config_in), cost(0) { }
    }; // Node class

public:

    
    MotionPlanner(ArmState robot_state_in, lcm::LCM& lcm_in, KinematicsSolver solver_in);

    tk::spline rrt_connect(vector<double> target);


private:

    /**
     * Generate a random config based on the joint limits
     * */
    vector<double> sample();

    /**
     * Find nearest node in tree to a given random node in config space
     * */
    Node* nearest(Node* tree_root, Vector6d rand);

    Vector6d steer(Node* start, Vector6d end);

    vector<double> backtrace_path(Node* end, Node* root);

    Node* extend(Node* tree, Node* z_rand);

    Node* connect(Node* tree, Node* a_new);

    tk::spline spline_fitting(vector<double> path);


    Node* root;

};


#endif