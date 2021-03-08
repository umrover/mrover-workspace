#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <Eigen/Dense>
#include <vector>
#include "arm_state.hpp"
#include "kinematics.hpp"
#include <map>
#include <random>

#include "spline.h"

using namespace Eigen;
using namespace std;

static constexpr int MAX_RRT_ITERATIONS = 500;

/**
 * Use rrt_connect() to map out a path to the target position.
 * */
class MotionPlanner {
private:

    // TODO convert everything to radians

    /**
     * RRT Node
     * */
    class Node {
        friend class MotionPlanner;
    private:
        Vector6d config;
        
        Node* parent;
        vector<Node*> children;
        double cost;

    public:
        Node(Vector6d config_in) : config(config_in), parent(nullptr), cost(0) { }
    }; // Node class

    KinematicsSolver solver;

    vector<tk::spline> splines;

    /**
     * joint_limits[i]["lower | upper"] returns i'th limit in degrees
     * */
    vector< map<string, double> > joint_limits;

    vector<double> step_limits;

    Node* start_root;
    Node* goal_root;

    // random engine for sample()
    std::default_random_engine eng;

    // for testing only
    int spline_size;

public:
    
    MotionPlanner(const ArmState &robot, KinematicsSolver& solver_in);

    /**
     * Implements RRT path planning to create path
     * 
     * @param robot starting state with current joint angles
     * @param target the set of joint angles the algorithm must reach
     * 
     * @return true if a path was found
     * */
    bool rrt_connect(ArmState& robot, const Vector6d& target_angles);

    /**
     * @param spline_t a time between 0 and 1
     * @return a vector of six doubles representing the angles of each joint at time spline_t
     * */
    vector<double> get_spline_pos(double spline_t);


private:

    /**
     * Generate a random config based on the joint limits
     * */
    Vector6d sample();

    /**
     * @param tree_root the root of the RRT tree
     * @param rand a random set of angles in the possible space
     * 
     * @return nearest node in tree to a given random node in config space
     * */
    Node* nearest(Node* tree_root, Vector6d& rand);

    /**
     * @param start an RRT node that has been found to be near end
     * @param end the target set of angles
     * 
     * @return a set of angles branching from start towards end without violating step_limits
     * */
    Vector6d steer(Node* start, Vector6d& end);

    vector<Vector6d> backtrace_path(Node* end, Node* root);

    void delete_tree(Node* twig);
    void delete_tree_helper(Node* root);

    Vector6d get_radians(Vector6d& config);

    Node* extend(ArmState &robot, Node* tree, Vector6d& z_rand);

    Node* connect(ArmState &robot, Node* tree, Vector6d& a_new);

    void spline_fitting(vector<Vector6d>& path);

};


#endif