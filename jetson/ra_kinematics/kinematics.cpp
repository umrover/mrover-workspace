#include "kinematics.hpp"
#include "utils.hpp"
#include "arm_state.hpp"
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <iomanip>

typedef Eigen::Matrix<double, -1, -1> MatrixXd;

using namespace Eigen;


void KinematicsSolver::FK(ArmState &robot_state) {
    Matrix4d global_transf = Matrix4d::Identity();

    // TODO: Edit name of base ("chassis-a") as necessary:
    robot_state.set_link_transform(0, Matrix4d::Identity());
    for (size_t i = 0; i < 6; ++i) {
        Matrix4d rot_theta = get_joint_xform(robot_state, i, robot_state.get_joint_angle(i));

        // Set up transformation matrix based on position of joint with respect to previous joint.
        Matrix4d local_transf = Matrix4d::Identity();
        local_transf.block(0,3,3,1) = robot_state.get_joint_pos_local(i);

        // Find transformation of current joint in global frame, using transformation from previous joint.
        global_transf = global_transf * (local_transf * rot_theta);

        robot_state.set_joint_transform(i, global_transf);
        robot_state.set_link_transform(i+1, global_transf);
    }

    // Get end effector position in frame of joint f
    Matrix4d T = Matrix4d::Identity();

    // set 4th row (index 3) as ef_xyz
    T.block(0,3,3,1) = robot_state.get_ef_xyz();

    robot_state.set_ef_transform(global_transf * T);

    // Torque calculations (not currently used)

    // Vector3d prev_com(0,0,0);
    // double total_mass = 0;

    // for (auto it = robot_state.joints.rbegin(); it != robot_state.joints.rend(); ++it) {
    //     Vector3d joint_pos = robot_state.get_joint_pos_world(it->first);
    //     Vector3d com_cur_link = robot_state.get_joint_com(it->first);
    //     double cur_link_mass = robot_state.get_joint_mass(it->first);

    //     // Calculate center of mass of current link:
    //     Vector3d curr_com = prev_com * total_mass + com_cur_link * cur_link_mass;
    //     total_mass += cur_link_mass;
    //     curr_com /= total_mass;

    //     // Calculate torque for current joint:
    //     Vector3d r = curr_com - joint_pos;
    //     Vector3d rot_axis_world = robot_state.get_joint_axis_world(it->first);
    //     Vector3d torque = calculate_torque(r, total_mass, rot_axis_world);
    //     robot_state.set_joint_torque(it->first, torque);
    //     prev_com = curr_com;
    // }
}
Matrix4d KinematicsSolver::get_joint_xform(const ArmState& robot_state, size_t joint_index, double theta) {
    // Get rotation axis of current joint.
    Vector3d rot_axis_child = robot_state.get_joint_axis(joint_index);

    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    Matrix4d rot_theta = Matrix4d::Identity();

    // Make rotation about rot axis by theta:

    // if rotating about x-axis (x-axis points right)
    // joints b, c, and e
    if (rot_axis_child(0) == 1) {
        rot_theta(1,1) = cos_theta;
        rot_theta(2,1) = sin_theta;
        rot_theta(1,2) = -1*sin_theta;
        rot_theta(2,2) = cos_theta;
    }
    // if rotating about y-axis (y-axis points forward)
    // joints d, f
    else if(rot_axis_child(1) == 1) {
        rot_theta(0,0) = cos_theta;
        rot_theta(2,0) = -1*sin_theta;
        rot_theta(0,2) = sin_theta;
        rot_theta(2,2) = cos_theta;
    }
    // if rotating about z-axis (z-axis points up)
    // joint a
    else if(rot_axis_child(2) == 1) {
        rot_theta(0,0) = cos_theta;
        rot_theta(1,0) = sin_theta;
        rot_theta(0,1) = -1*sin_theta;
        rot_theta(1,1) = cos_theta;
    }

    return rot_theta;
}

Matrix4d KinematicsSolver::apply_joint_xform(const ArmState& robot_state, size_t joint_index, double theta) {

    Vector3d xyz = robot_state.get_joint_pos_local(joint_index);

    // get intended direction of rotation
    Vector3d rot_axis_child = robot_state.get_joint_axis(joint_index);

    Matrix4d rot_theta = Matrix4d::Identity();
    Matrix4d trans = Matrix4d::Identity();

    double ctheta = cos(theta);
    double stheta = sin(theta);

    // copy trans from position
    trans(0, 3) = xyz(0);
    trans(1, 3) = xyz(1);
    trans(2, 3) = xyz(2);

    Vector3d rot_arr_one(1, 0, 0);
    Vector3d rot_arr_two(0, 1, 0);
    Vector3d rot_arr_three(0, 0, 1);

    // make rotation around rot_axis by theta
    if (rot_axis_child == rot_arr_one) {
        rot_theta(1, 1) = ctheta;
        rot_theta(1, 2) = -stheta;
        rot_theta(2, 1) = stheta;
        rot_theta(2, 2) = ctheta;
    }
    else if (rot_axis_child == rot_arr_two) {
        rot_theta(0, 0) = ctheta;
        rot_theta(2, 0) = -stheta;
        rot_theta(0, 2) = stheta;
        rot_theta(2, 2) = ctheta;
    }
    else if (rot_axis_child == rot_arr_three) {
        rot_theta(0, 0) = ctheta;
        rot_theta(0, 1) = -stheta;
        rot_theta(1, 0) = stheta;
        rot_theta(1, 1) = ctheta;
    }

    Matrix4d parent_mat = robot_state.get_ef_transform();

    Matrix4d T = trans * rot_theta;
    return parent_mat * T;
}

std::pair<Vector6d, bool> KinematicsSolver::IK(ArmState &robot_state, const Vector6d& target_point, bool set_random_angles, bool use_euler_angles) {
    // Print inital joint angles:
    std::vector<double> init_joint_angs = robot_state.get_joint_angles();

    /*
        Inverse kinematics for MRover arm using cyclic
        coordinate descent (CCD)
        Params:
            target_point (np.array([x, y, z, alpha, beta, gamma])):
            target point in 3d space
                for end effector
            set_random_angles: asks solver to set joint angles to random
                angles. Used to escape local minima
        Returns:
            joint_angles (list)
            bool : whether the robot arm was able to find
                joint angles that allow it to reach within a threshold
                of the target location
        Reference:
            http://www.cs.cmu.edu/~15464-s13/assignments/assignment2/jlander_gamedev_nov98.pdf
            http://www.cs.cmu.edu/~15464-s13/lectures/lecture6/IK.pdf
    */

    num_iterations = 0;
    int num_iterations_low_movement = 0;
    Vector3d target_pos_world = target_point.head(3);
    Vector3d target_ang_world = target_point.tail(3);

    // backup current angles
    perform_backup(robot_state);

    std::vector<std::string> joints_vec = robot_state.get_all_joints();
    if (set_random_angles) {
        std::vector<double> rand_angs;
        rand_angs.resize(6);

        // ((double) rand() / (RAND_MAX)) yields random number [0,1)
        // RAND_MAX is defined in a library

        // TODO: make this actually depend on joint limits, and test whether using full range of values is best

        if (robot_state.get_joint_locked(0)) {
            rand_angs[0] = robot_state.get_joint_angle(0);
        }
        else {
            rand_angs[0] = ((double) rand() / (RAND_MAX) - 0.5) * 6;  // -3 ... 3
        }

        if (robot_state.get_joint_locked(1)) {
            rand_angs[1] = robot_state.get_joint_angle(1);
        }
        else {
            rand_angs[1] = ((double) rand() / (RAND_MAX)) + 0.05;     // 0.05 ... 1.05
        }

        if (robot_state.get_joint_locked(2)) {
            rand_angs[2] = robot_state.get_joint_angle(2);
        }
        else {
            rand_angs[2] = ((double) rand() / (RAND_MAX) - 0.5) * 5;  // -2.5 ... 2.5
        }

        if (robot_state.get_joint_locked(3)) {
            rand_angs[3] = robot_state.get_joint_angle(3);
        }
        else {
            rand_angs[3] = ((double) rand() / (RAND_MAX) - 0.5) * 6;  // -3   ... 3
        }

        if (robot_state.get_joint_locked(4)) {
            rand_angs[4] = robot_state.get_joint_angle(4);
        }
        else {
            rand_angs[4] = ((double) rand() / (RAND_MAX) - 0.5) * 5;  // -2.5 ... 2.5
        }

        if (robot_state.get_joint_locked(5)) {
            rand_angs[5] = robot_state.get_joint_angle(5);
        }
        else {
            rand_angs[5] = ((double) rand() / (RAND_MAX) - 0.5) * 6;  // -3   ... 3
        }

        robot_state.set_joint_angles(rand_angs);
    }

    // Update transforms using FK
    FK(robot_state);
    
    // Get position of the end effector
    Vector3d ef_pos_world = robot_state.get_ef_pos_world();
    Vector3d ef_ang_world = robot_state.get_ef_ang_world();

    // Compute euclidean distance to target
    double dist = (ef_pos_world - target_pos_world).norm();

    // Compute angular distance to target (using each euler angle)
    double angle_dist = 0;
    for (size_t i = 0; i < 3; ++i) {
        double abs_angle_dist = abs(ef_ang_world[i] - target_ang_world[i]);
        angle_dist += pow(std::min(abs_angle_dist, 2*M_PI - abs_angle_dist), 2);
    }

    Vector6d d_ef;

    // While distance is bad or angle is bad
    while (dist > POS_THRESHOLD || (angle_dist > ANGLE_THRESHOLD && use_euler_angles)) {

        // If we need to break out of loop
        if (num_iterations_low_movement > MAX_ITERATIONS_LOW_MOVEMENT || num_iterations > MAX_ITERATIONS) {
            std::cout << "FAILURE --- broke out of loop in " << num_iterations << " iterations --- dist: " << dist << "\t";
            std::cout << "angle dist: " << angle_dist << "\n";

            Vector6d joint_angles;
            for (int i = 0; i < 6; ++i) {
                joint_angles(i) = robot_state.get_joint_angle(i);
            }

            // restore previous robot_state angles
            recover_from_backup(robot_state);

            return std::pair<Vector6d, bool> (joint_angles, false);
        }

        // d_ef is the vector we want the end effector to move in this step

        // First 3 values of d_ef represent euclidean direction we want to move
        d_ef.head(3) = k_position_step * (target_point.head(3) - ef_pos_world);

        // Last 3 values of d_ef represent direction for each euler angle
        if (use_euler_angles) {

            // For each euler angle
            for (size_t i = 3; i < 6; ++i) {

                // If not crossing the 2pi --> 0 line
                if (abs(target_point[i] - ef_ang_world[i - 3]) <= M_PI) {
                    d_ef[i] = k_angle_step * (target_point[i] - ef_ang_world[i - 3]);
                }
                else {
                    double targ_ang = target_point[i];
                    double curr_ang = ef_ang_world[i - 3];

                    // Adjust angles as necessary
                    if (targ_ang < 0) {
                        targ_ang += 2 * M_PI;
                    }
                    if (curr_ang < 0) {
                        curr_ang += 2 * M_PI;
                    }

                    d_ef[i] = k_angle_step * (targ_ang - curr_ang);
                }
            }
        }
        else {
            d_ef.tail(3) = Vector3d{0, 0, 0};
        }

        // Move the end effector a small amount towards the target
        IK_step(robot_state, d_ef, use_euler_angles);

        double dist_original = dist;
        double angle_dist_original = angle_dist;

        // Calculate progress towards target

        ef_pos_world = robot_state.get_ef_pos_world();
        ef_ang_world = robot_state.get_ef_ang_world();

        dist = (ef_pos_world - target_pos_world).norm();

        angle_dist = 0;
        for (size_t i = 0; i < 3; ++i) {
            double abs_angle_dist = abs(ef_ang_world[i] - target_ang_world[i]);
            angle_dist += pow(std::min(abs_angle_dist, 2*M_PI - abs_angle_dist), 2);
        }

        // If only a very small change has been made (change in angles approximated by change in distance)
        if (abs(dist - dist_original) < EPSILON_DIST && abs(angle_dist - angle_dist_original) < EPSILON_ANGLE_DIST) {
            ++num_iterations_low_movement;
        }
        else {
            num_iterations_low_movement = 0;
        }

        ++num_iterations;
    }

    std::vector<double> angles_vec = robot_state.get_joint_angles();

    // Check for collisions
    if (!is_safe(robot_state, angles_vec)) {
        std::cout << "UNSAFE IK solution!\n";

        recover_from_backup(robot_state);
        return std::pair<Vector6d, bool> (vecTo6d(angles_vec), false);
    }

    std::cout << "SUCCESS in " << num_iterations << " iterations --- dist: " << dist << "\t";
    std::cout << "angle dist: " << angle_dist << "\n";

    // restore robot_state to previous values
    recover_from_backup(robot_state);
    return std::pair<Vector6d, bool> (vecTo6d(angles_vec), true);
}

void KinematicsSolver::IK_step(ArmState& robot_state, const Vector6d& d_ef, bool use_euler_angles) {
    Vector3d ef_pos_world = robot_state.get_ef_pos_world();
    Vector3d ef_euler_world = robot_state.get_ef_ang_world();

    MatrixXd jacobian(6, 6);
    jacobian.setZero();

    // 6-D matrix
    for (size_t i = 0; i < 6; ++i) {

        // calculate vector from joint i to end effector
        Vector3d joint_pos_world = robot_state.get_link_point_world(i+1);
        Vector3d joint_to_ef_vec_world = ef_pos_world - joint_pos_world;

        // find rotation axis of joint in world frame
        Vector3d rot_axis_local = robot_state.get_joint_axis(i);
        Matrix4d joint_xform = robot_state.get_joint_transform(i);
        Vector3d rot_axis_world = apply_transformation(joint_xform, rot_axis_local);

        Vector6d joint_col;
        joint_col.head(3) = rot_axis_world.cross(joint_to_ef_vec_world);

        // calculate end effector orientation jacobian values
        if (use_euler_angles) {

            double curr_angle = robot_state.get_joint_angle(i);

            // Calculate euler angle of end effector after moving joint i by DELTA_THETA
            double new_angle = robot_state.get_joint_angle(i) + DELTA_THETA;
            robot_state.set_joint_angle(i, new_angle);
            FK(robot_state);

            Vector3d euler_angles = compute_euler_angles(robot_state.get_ef_transform().block(0, 0, 3, 3));

            // Calculate angle difference and scale by DELTA_THETA
            Vector3d diff_angs = euler_angles - ef_euler_world;
            joint_col.tail(3) = diff_angs / DELTA_THETA;

            // Reset arm
            robot_state.set_joint_angle(i, curr_angle);
            FK(robot_state);
        }
        else {
            joint_col.tail(3) = Vector3d(0, 0, 0);
        }

        // write column i of the jacobian
        for (size_t j = 0; j < 6; ++j) {
            jacobian(j, i) = joint_col[j];
        }
    }

    FK(robot_state);

    MatrixXd jacobian_inverse;
    // if using pseudo inverse (usually corresponds to using euler angles)
    if (use_euler_angles) {
        jacobian_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    }
    // otherwise, simply transposing the vector is sufficient
    else {
        jacobian_inverse = jacobian.transpose();
    }

    Vector6d d_theta = jacobian_inverse * d_ef;

    // find the angle of each joint
    std::vector<double> angle_vec;
    for (size_t i = 0; i < 6; ++i) {

        // don't move joint i if it's locked
        if (robot_state.get_joint_locked(i)) {
            d_theta[i] = 0;
        }

        std::vector<double> limits = robot_state.get_joint_limits(i);

        double angle = robot_state.get_joint_angle(i) + d_theta[i];

        // clip angle to within joint limits
        if (angle < limits[0]) {
            // If joint can reach all 2pi options
            if (robot_state.is_continuous(i)) {
                angle = limits[1];
            }
            else {
                angle = limits[0];   
            }
        }
        else if (angle > limits[1]) {
            // If joint can reach all 2pi options
            if (robot_state.is_continuous(i)) {
                angle = limits[0];
            }
            else {
                angle = limits[1];   
            }
        }

        angle_vec.push_back(angle);
    }

    // run forward kinematics
    robot_state.set_joint_angles(angle_vec);
    FK(robot_state);
}

bool KinematicsSolver::is_safe(ArmState &robot_state, const std::vector<double> &angles) {
    perform_backup(robot_state);

    robot_state.set_joint_angles(angles);
    bool safe = is_safe(robot_state);

    recover_from_backup(robot_state);
    return safe;
}

bool KinematicsSolver::is_safe(ArmState &robot_state) {
    // if any angles are outside bounds
    if (!limit_check(robot_state, robot_state.get_joint_angles())) {
        return false;
    }

    // run FK algorithm to ensure update state
    FK(robot_state);

    return robot_state.obstacle_free();
}

bool KinematicsSolver::limit_check(ArmState &robot_state, const std::vector<double> &angles) {
    for (size_t i = 0; i < 6; ++i) {
        std::vector<double> limits = robot_state.get_joint_limits(i);
        
        // if any angle is outside of bounds
        if (!(limits[0] - LIMIT_CHECK_MARGIN <= angles[i]
              && angles[i] < limits[1] + LIMIT_CHECK_MARGIN)) {
            return false;
        }
    }

    // if all angles are in bounds
    return true;
}

void KinematicsSolver::perform_backup(ArmState &robot_state) {
    std::vector<double> backup = robot_state.get_joint_angles();

    // save all angles to stack
    arm_state_backup.push(backup);
}

void KinematicsSolver::recover_from_backup(ArmState &robot_state) {
    if (arm_state_backup.empty()) {
        std::cout << "ERROR: no backup arm_state to revert to!\n";
    }
    else {
        // pop angles from stack
        robot_state.set_joint_angles(arm_state_backup.top());
        arm_state_backup.pop();

        // update state based on new angles
        FK(robot_state);
    }
}

int KinematicsSolver::get_num_iterations() {
    return num_iterations;
}

