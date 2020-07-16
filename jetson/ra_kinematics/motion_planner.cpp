#include "motion_planner.hpp"

#include <random>
#include <queue>
#include <time.h>


MotionPlanner::MotionPlanner(const ArmState &robot, KinematicsSolver &solver_in) :
        solver(solver_in) {
    
    // add limits for each joint to joint_limits, after converting to degrees
    for (string &joint : robot.get_all_joints()) {
      map<string, double> limits = robot.get_joint_limits(joint);

      limits["lower"] = limits["lower"] * 180 / M_PI;
      limits["upper"] = limits["upper"] * 180 / M_PI;

      joint_limits.push_back(limits);
    }

    step_limits.reserve(6);

    // limits for an individual step for each joint in degrees
    step_limits.push_back(1);
    step_limits.push_back(1);
    step_limits.push_back(2);
    step_limits.push_back(3);
    step_limits.push_back(5);
    step_limits.push_back(1);

    //time_t timer;
    std::default_random_engine eng(clock());
}

Vector6d MotionPlanner::sample() {
    Vector6d z_rand;

    for (size_t i = 0; i < joint_limits.size(); ++i) {
        // create distribution of angles between limits and choose an angle
        std::uniform_real_distribution<double> distr(joint_limits[i]["lower"], joint_limits[i]["upper"]);
        z_rand(i) = distr(eng);
    }

    return z_rand;
}

MotionPlanner::Node* MotionPlanner::nearest(MotionPlanner::Node* tree_root, const Vector6d &rand) {
    queue<Node*> q;
    q.push(tree_root);

    double min_dist = numeric_limits<double>::max();
    Node* min_node = nullptr;

    // Run breadth-first search for nearest node in tree
    while (!q.empty()) {
        Node* node = q.front();
        q.pop();

        // TODO consider taking into account cost
        double dist = (node->config - rand).norm();

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


Vector6d MotionPlanner::steer(MotionPlanner::Node* start, const Vector6d &end) {

    // calculate the vector from start position to end (each value is a change in angle)
    Vector6d vec = end - start->config;

    // check for any steps that are outside acceptable range
    bool step_too_big = false;
    for (size_t i = 0; i < step_limits.size(); ++i) {
        if (step_limits[i] - abs(vec(i)) < 0) {
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
    double min_t = numeric_limits<double>::max();

    //parametrize the line
    for (int i = 0; i < vec.size(); ++i) {
        double t = numeric_limits<double>::max();

        // if the difference (from start to end) in angles at joint i is not 0
        if (vec[i] != 0) {
            t = step_limits[i] / abs(vec[i]);
        }

        if (t < min_t) {
            min_t = t;
        }
    }
    
    // find the biggest possible step from start directly towards end
    Vector6d new_config = start->config;
    for (int i = 0; i < vec.size(); ++i) {
        new_config(i) += min_t * vec[i];
    }

    return new_config;
}

vector<Vector6d> MotionPlanner::backtrace_path(MotionPlanner::Node* end, MotionPlanner::Node* root) {
    vector<Vector6d> path;

    // starting at end, add each position config to path
    while (true) {
        path.push_back(get_radians(end->config));

        if (end == root) {
            return path;
        }

        end = end->parent;
    }
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

Vector6d MotionPlanner::get_radians(Vector6d &config) {
    for (int i = 0; i < 6; ++i) {
        config[i] *= (M_PI / 180);
    }

    return config;
}

MotionPlanner::Node* MotionPlanner::extend(ArmState &robot, Node* tree, const Vector6d &z_rand) {

    // z_nearest is the nearest node in the tree to z_rand
    Node* z_nearest = nearest(tree, z_rand);

    // z_new is the set of angles extending from z_nearest towards z_rand, within step_limits
    Vector6d z_new = steer(z_nearest, z_rand);

    // convert z_new to radians
    vector<double> z_new_radians;
    z_new_radians.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        z_new_radians[i] = z_new(i) * M_PI / 180;
    }

    // check that we have not violated joint limits or created collisions
    if (!solver.is_safe(robot, z_new_radians)) {
        return nullptr;
    }

    // create dynamic node branching from tree
    Node* new_node = new Node(z_new);
    new_node->parent = z_nearest;

    // cost is previous cost + distance to new set of angles
    new_node->cost = z_nearest->cost + (z_nearest->config - z_new).norm();

    // add new_node to tree
    z_nearest->children.push_back(new_node);

    return new_node;
}

MotionPlanner::Node* MotionPlanner::connect(ArmState &robot, Node* tree, const Vector6d &a_new) {
    Node* extension;

    do {
        extension = extend(robot, tree, a_new);
    } while (extension && extension->config != a_new);

    return extension;
}

bool MotionPlanner::rrt_connect(ArmState &robot, const Vector6d &target_angles) {

    // retrieve starting and target joint angles
    Vector6d start;
    start(0) = robot.get_joint_angles()["joint_a"];
    start(1) = robot.get_joint_angles()["joint_b"];
    start(2) = robot.get_joint_angles()["joint_c"];
    start(3) = robot.get_joint_angles()["joint_d"];
    start(4) = robot.get_joint_angles()["joint_e"];
    start(5) = robot.get_joint_angles()["joint_f"];

    Vector6d target = target_angles;

    // convert target to degrees
    for (int i = 0; i < target.size(); ++i) {
        target(i) = target(i) * 180 / M_PI;
        start(i) = start(i) * 180 / M_PI;
    }

    start_root = new Node(start);
    goal_root =  new Node(target);

    for (int i = 0; i < MAX_RRT_ITERATIONS; ++i) {
        Node* a_root = i % 2 == 0 ? start_root : goal_root;
        Node* b_root = i % 2 == 0 ? goal_root : start_root;
        Vector6d z_rand = sample();

        Node* a_new = extend(robot, a_root, z_rand);

        if (a_new) {

            Node* b_new = connect(robot, b_root, a_new->config);

            // if the trees are connected
            if (b_new && a_new->config == b_new->config) {
                vector<Vector6d> a_path = backtrace_path(a_new, a_root);
                vector<Vector6d> b_path = backtrace_path(b_new, b_root);

                // reverse a_path
                for (size_t j = 0; j < a_path.size() / 2; ++j) {
                    swap(a_path[j], a_path[a_path.size() - 1 - j]);
                }

                // add the intersection of the paths to a_path
                Vector6d middle;
                for (int j = 0; j < 6; ++j) {
                    middle(j) = a_new->config(j);
                }

                a_path.push_back(middle);
                a_path.reserve(a_path.size() + b_path.size());

                for (Vector6d b : b_path) {
                    a_path.push_back(b);
                }

                // reverse entire path if a_root is the goal
                if (i % 2 != 0) {
                    for (size_t j = 0; j < a_path.size() / 2; ++j) {
                        swap(a_path[j], a_path[a_path.size() - 1 - j]);
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
    splines = vector<tk::spline>();
    return false;

}

void MotionPlanner::spline_fitting(const vector<Vector6d> &path) {

    // six vectors, each with the path of a single component
    vector< vector<double> > separate_paths;
    separate_paths.resize(6, vector<double>(path.size()));

    // convert path to vectors
    for (size_t i = 0; i < path.size(); ++i) {
        for (size_t j = 0; j < 6; ++j) {
            separate_paths[j][i] = path[i](j);
        }
    }

    // create a linear space between 0 and 1 with path.size() increments
    vector<double> x_;
    x_.reserve(path.size());

    // find ideal step size for x_, handling edge cases
    double spline_step;
    if (path.size() > 1) {
        spline_step = 1.0 / (path.size() - 1);
    }
    else if (path.size() == 1) {
        for (size_t i = 0; i < 6; ++i) {
            separate_paths[i].push_back(separate_paths[i][0]);
        }
        spline_step = 1.0;
    }
    else {
        cout << "Error! Path is of size 0.\n";
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
        cout << "Error! Could not create properly sized spline.\n";
    }

    // use tk to create six different splines, representing a spline in 6 dimensions
    splines.clear();
    splines.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        splines[i].set_points(x_, separate_paths[i]);
    }
}

vector<double> MotionPlanner::get_spline_pos(double spline_t) {
    vector<double> angles;
    angles.reserve(6);

    for (const tk::spline &spline : splines) {
        angles.emplace_back(spline(spline_t));
    }

    // invert 5th angle for use purposes
    //angles[4] *= -1;

    return angles;
}
