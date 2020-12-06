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

    vector<tk::spline> splines;

    vector< map<string, double> > all_limits;

    vector<int> step_limits;

    int neighbor_dist;
    int max_iterations;
    int i;

    lcm::LCM& lcm;

    class Node {
        friend class MotionPlanner;
    private:
        Vector6d config;
        
        Node* parent;
        vector<Node*> children;
        double cost;

    public:
        Node(Vector6d config_in) : config(config_in), cost(0) { }
    }; // Node class

    Node* start_root;
    Node* goal_root;

    // for testing only
    int spline_size;

public:

    
    MotionPlanner(ArmState& robot_state_in, lcm::LCM& lcm_in, KinematicsSolver& solver_in);

    void rrt_connect(Vector6d& target);

    /**
     * @param spline_t a time between 0 and 1
     * @return a vector of six doubles representing the angles of each joint at time spline_t
     * */
    vector<double> get_spline_pos(double spline_t);

    // for testing only
    int getSplineSize() {
        return spline_size;
    }


private:

    /**
     * Generate a random config based on the joint limits
     * */
    Vector6d sample();

    /**
     * Find nearest node in tree to a given random node in config space
     * */
    Node* nearest(Node* tree_root, Vector6d& rand);

    Vector6d steer(Node* start, Vector6d& end);

    vector<Vector6d> backtrace_path(Node* end, Node* root);

    Vector6d get_radians(Vector6d& config);

    Node* extend(Node* tree, Vector6d& z_rand);

    Node* connect(Node* tree, Vector6d& a_new);

    vector<tk::spline> spline_fitting(vector<Vector6d>& path);

    Node* root;

    void inc_i(){
        ++i;
    }
};


#endif