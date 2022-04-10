#include "motion_planner.hpp"

#include <random>
#include <queue>
#include <time.h>
#include <cmath>


MotionPlanner::MotionPlanner(const ArmState &robot, KinematicsSolver &solver_in) :
        solver(solver_in) { 
    step_limits.reserve(robot.num_joints());

    // add limits for each joint to joint_limits, after converting to degrees
    for (size_t i = 0; i < robot.num_joints(); ++i) {
      std::vector<double> limits = robot.get_joint_limits(i);

      joint_limits.push_back(limits);
      step_limits.push_back(robot.get_joint_max_speed(i) / 50.0);
    }

    //time_t timer;
    std::default_random_engine eng(clock());
}

std::vector<double> MotionPlanner::sample(std::vector<double> start, const ArmState &robot) {
    std::vector<double> z_rand;

    for (size_t i = 0; i < joint_limits.size(); ++i) {
        
        if (robot.get_joint_locked(i)) {
            // if joint is locked, z_rand matches start
            z_rand.push_back(start[i]);
        }
        else { 
            // create distribution of angles between limits and choose an angle
            std::uniform_real_distribution<double> distr(joint_limits[i][0], joint_limits[i][1]);
            z_rand.push_back(distr(eng));
        }
    }

    return z_rand;
}

MotionPlanner::Node* MotionPlanner::nearest(MotionPlanner::Node* tree_root, const std::vector<double> &rand) {
    std::queue<Node*> q;
    q.push(tree_root);

    double min_dist = std::numeric_limits<double>::max();
    Node* min_node = nullptr;

    // Run breadth-first search for nearest node in tree
    while (!q.empty()) {
        Node* node = q.front();
        q.pop();

        double dist = (vecTo6d(node->config) - vecTo6d(rand)).norm();

        if (dist < min_dist) {
          min_dist = dist;
          min_node = node;
        }

        for (Node* child : node->children) {
          q.push(child);
        }
    }

    return min_node;
}


std::vector<double> MotionPlanner::steer(MotionPlanner::Node* start, const std::vector<double> &end) {

    // calculate the vector from start position to end (each value is a change in angle)
    std::vector<double> vec;
    for (size_t i = 0; i < start->config.size(); ++i) {
        vec.push_back(end[i] - start->config[i]);
    }
    
    // check for any steps that are outside acceptable range
    bool step_too_big = false;
    for (size_t i = 0; i < step_limits.size(); ++i) {
        if (step_limits[i] - std::abs(vec[i]) < 0) {
            step_too_big = true;
            break;
        }
    }

    // if end is within reach of all joints, return end
    if (!step_too_big) {
        return end;
    }

    // min_t will be the reciprocal of the largest number of steps required
    // to reach from start to end
    double min_t = std::numeric_limits<double>::max();

    //parametrize the line
    for (size_t i = 0; i < vec.size(); ++i) {
        double t = std::numeric_limits<double>::max();

        // if the difference (from start to end) in angles at joint i is not 0
        if (vec[i] != 0) {
            t = step_limits[i] / std::abs(vec[i]);
        }

        if (t < min_t) {
            min_t = t;
        }
    }
    
    // find the biggest possible step from start directly towards end
    std::vector<double> new_config = start->config;
    for (size_t i = 0; i < vec.size(); ++i) {
        new_config[i] += min_t * vec[i];
    }

    return new_config;
}

std::vector< std::vector<double> > MotionPlanner::backtrace_path(MotionPlanner::Node* end, MotionPlanner::Node* root) {
    std::vector< std::vector<double> > path;

    // starting at end, add each position config to path
    while (true) {
        path.push_back(end->config);

        if (end == root) {
            break;
        }

        end = end->parent;
    }

    return path;
}

void MotionPlanner::delete_tree(MotionPlanner::Node* twig) {
    if (!twig) {
        return;
    }
    while (twig->parent) {
        twig = twig->parent;
    }

    delete_tree_helper(twig);
}

void MotionPlanner::delete_tree_helper(MotionPlanner::Node* root) {
    if (root) {
        for (Node* child : root->children) {
            delete_tree_helper(child);
        }

        delete root;
    }
}

MotionPlanner::Node* MotionPlanner::extend(ArmState &robot, Node* tree, const std::vector<double> &z_rand) {

    // z_nearest is the nearest node in the tree to z_rand
    Node* z_nearest = nearest(tree, z_rand);

    // z_new is the set of angles extending from z_nearest towards z_rand, within step_limits
    std::vector<double> z_new = steer(z_nearest, z_rand);

    // check that we have not violated joint limits or created collisions
    if (!solver.is_safe(robot, z_new)) {
        return nullptr;
    }

    // create dynamic node branching from tree
    Node* new_node = new Node(z_new);
    new_node->parent = z_nearest;

    // cost is previous cost + distance to new set of angles
    new_node->cost = z_nearest->cost + (vecTo6d(z_nearest->config) - vecTo6d(z_new)).norm();

    // add new_node to tree
    z_nearest->children.push_back(new_node);

    return new_node;
}

MotionPlanner::Node* MotionPlanner::connect(ArmState &robot, Node* tree, const std::vector<double> &a_new) {
    Node* extension;
    int ct = 0;

    do {
        extension = extend(robot, tree, a_new);
        ++ct;
    } while (extension && !vec_almost_equal(extension->config, a_new, VEC_ANGLE_EPSILON) && ct < 2000);

    if (ct == 2000) {
        std::cout << "Error: Motion planner connect exceeded 2000 iterations\n";
    }

    return extension;
}

bool MotionPlanner::rrt_connect(ArmState &robot, const std::vector<double> &target_angles) {

    // retrieve starting and target joint angles
    std::vector<double> start;
    start = robot.get_joint_angles();

    std::vector<double> target = target_angles;

    start_root = new Node(start);
    goal_root =  new Node(target);

    for (int i = 0; i < MAX_RRT_ITERATIONS; ++i) {
        Node* a_root = i % 2 == 0 ? start_root : goal_root;
        Node* b_root = i % 2 == 0 ? goal_root : start_root;
        std::vector<double> z_rand = sample(start, robot);

        Node* a_new = extend(robot, a_root, z_rand);

        if (a_new) {

            Node* b_new = connect(robot, b_root, a_new->config);

            // if the trees are connected
            if (b_new && a_new->config == b_new->config) {
                std::vector< std::vector<double> > a_path = backtrace_path(a_new, a_root);
                std::vector< std::vector<double> > b_path = backtrace_path(b_new, b_root);

                // reverse a_path
                for (size_t j = 0; j < a_path.size() / 2; ++j) {
                    std::swap(a_path[j], a_path[a_path.size() - 1 - j]);
                }

                // add the intersection of the paths to a_path
                std::vector<double> middle;
                for (size_t j = 0; j < robot.num_joints(); ++j) {
                    middle.push_back(a_new->config[j]);
                }

                a_path.push_back(middle);
                a_path.reserve(a_path.size() + b_path.size());

                for (std::vector<double> b : b_path) {
                    a_path.push_back(b);
                }

                // reverse entire path if a_root is the goal
                if (i % 2 != 0) {
                    for (size_t j = 0; j < a_path.size() / 2; ++j) {
                        std::swap(a_path[j], a_path[a_path.size() - 1 - j]);
                    }
                }

                spline_size = a_path.size();

                spline_fitting(a_path);

                // delete trees before returning
                delete_tree(start_root);
                delete_tree(goal_root);

                return true;
            }

        }
        
    } // for loop

    delete_tree(start_root);
    delete_tree(goal_root);

    // if no path found, make sure splines is empty
    splines = std::vector<tk::spline>();
    return false;

}

void MotionPlanner::spline_fitting(const std::vector< std::vector<double> > &path) {

    // six vectors, each with the path of a single component
    std::vector< std::vector<double> > separate_paths;
    separate_paths.resize(step_limits.size(), std::vector<double>(path.size()));

    // convert path to vectors
    for (size_t i = 0; i < path.size(); ++i) {
        for (size_t j = 0; j < step_limits.size(); ++j) {   //for each joint
            separate_paths[j][i] = path[i][j];
        }
    }

    // create a linear space between 0 and 1 with path.size() increments
    std::vector<double> x_;
    x_.reserve(path.size());

    // find ideal step size for x_, handling edge cases
    double spline_step;
    if (path.size() > 1) {
        spline_step = 1.0 / (path.size() - 1);
    }
    else if (path.size() == 1) {
        for (size_t i = 0; i < step_limits.size(); ++i) {   //for each joint
            separate_paths[i].push_back(separate_paths[i][0]);
        }
        spline_step = 1.0;
    }
    else {
        std::cout << "Error! Path is of size 0.\n";
    }

    // create evenly distributed range from 0 to 1 with path.size() steps
    for (double i = 0.0; i <= 1.0; i += spline_step) {
        x_.push_back(i);
    }

    // recover from possible floating point issues
    if (x_.size() == path.size()) {
        x_[x_.size() - 1] = 1.0;
    }
    else if (x_.size() == path.size() - 1) {
        x_.push_back(1.0);
    }
    else {
        std::cout << "Error! Could not create properly sized spline.\n";
    }

    // use tk to create six different splines, representing a spline in 6 dimensions
    splines.clear();
    splines.resize(step_limits.size());
    for (size_t i = 0; i < step_limits.size(); ++i) {   //for each joint
        splines[i].set_points(x_, separate_paths[i]);
    }
}

std::vector<double> MotionPlanner::get_spline_pos(double spline_t) {
    std::vector<double> angles;
    angles.reserve(step_limits.size());

    // double mod_spline_t = modify_spline_t(spline_t);

    for (const tk::spline &spline : splines) {
        angles.emplace_back(spline(spline_t));
    }

    // invert 5th angle for use purposes
    //angles[4] *= -1;

    return angles;
}

double MotionPlanner::modify_spline_t(double spline_t) {

    return spline_t;

    // for 0 <= spline_t <= 0.2
    // if (spline_t <= 0.2) {
    //     // Ramp up spline_t from (0, 0) to (0.2, 0.1)
    //     return pow(spline_t, 1.4306765581);
    // }
    
    // // for 0.2 < spline_t <= 0.8
    // if (spline_t <= 0.8) {
    //     // Give linear scaling of spline_t from (0.2, 0.1) to (0.8, 0.9)
    //     return (4.0 / 3.0) * (spline_t - 0.5) + 0.5;
    // }

    // // for 0.2 < spline_t <= 0.8, ramp down spline_t from (0.8, 0.9) to (1, 1)
    // return std::min(1.0 - pow(1.0 - spline_t, 1.4306765581), 1.0);
}
