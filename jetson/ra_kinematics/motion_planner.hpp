#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <Eigen/Dense>
#include <vector>
#include "arm_state.hpp"
#include "kinematics.hpp"
#include <map>

#include "spline.h"

using namespace Eigen;
using namespace std;

/**
 * Use rrt_connect() to map out a path to the target position.
 * */
class MotionPlanner {
private:

    KinematicsSolver solver;

    vector<tk::spline> splines;

    vector< map<string, double> > all_limits;

    vector<int> step_limits;

    int neighbor_dist;
    int max_iterations;
    int i;

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
    
    MotionPlanner(const ArmState &robot, KinematicsSolver& solver_in);

    void rrt_connect(ArmState& robot, Vector6d& target);

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

    void delete_tree(Node* twig);
    void delete_tree(Node* root);

    Vector6d get_radians(Vector6d& config);

    Node* extend(ArmState &robot, Node* tree, Vector6d& z_rand);

    Node* connect(ArmState &robot, Node* tree, Vector6d& a_new);

    vector<tk::spline> spline_fitting(vector<Vector6d>& path);

    Node* root;

    void inc_i(){
        ++i;
    }
};


#endif