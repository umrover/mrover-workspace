#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <Eigen/Dense>
#include <vector>
#include "arm_state.hpp"
#include "kinematics.hpp"
#include <map>

#include <lcm/lcm-cpp.hpp>

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

public:

    // Should Node be outside the MotionPlanner class
    class Node {
    private:
        // Is config just a vector of doubles?
        vector<double> config;
        
        Node* parent;
        vector<Node*> children;
        double cost;

    public:
        Node(vector<double> config_in) : config(config_in), cost(0) { }
    }; // Node class

    
    MotionPlanner(ArmState robot_state_in, lcm::LCM& lcm_in, KinematicsSolver solver_in);

    /**
     * Generate a random config based on the joint limits
     * */
    vector<double> sample();

    /**
     * Find nearest node in tree to a given random node in config space
     * */
    Node* nearest(Node* tree_root, Node* rand);

    /**
     * Find neighbors of rand
     * */
    vector<Node*> near(vector<double> z_new);

    vector<Node*> steer(Node* start, Node* end);

    /**
     * Finds best parent, which has least (cost + distance to rand).
     * Connects new node to the chosen parent.
     * 
     * Calls rewire to find shortest path optimization for rrt*
     * */
    Node* choose_parent(vector<Node*> z_near, Node* z_nearest, vector<Node*> z_new);


    /**
     * Finds shortest path optimization for rrt*
     * */
    void rewire(vector<Node*> z_near, vector<Node*> z_new);

    vector<double> backtrace_path(Node* end, Node* root);

    Node* extend(Node* tree, Node* z_rand);

    Node* connect(Node* tree, Node* a_new);


    // TODO Should return a cubic spline
    void rrt_connect(vector<double> target);

    // What object are we using for a cubic spline?
    // TODO Should return a cubic spline
    void spline_fitting(vector<double> path);



};


#endif