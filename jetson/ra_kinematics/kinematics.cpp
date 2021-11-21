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

pair<Vector6d, bool> KinematicsSolver::IK(ArmState &robot_state, const Vector6d& target_point, bool set_random_angles, bool use_euler_angles) {
    // Print inital joint angles:
    vector<double> init_joint_angs = robot_state.get_joint_angles();
    cout << "Initial Joint angs:\n";
    for (size_t i = 0; i < 6; ++i) {
        cout << init_joint_angs[i] << " "; 
    }
    cout << "\n";

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
    // cout << "Running IK!" << endl;
    // cout << "target point: " << "\n" << target_point[0] << "\n" << target_point[1] << "\n" << target_point[2] << "\n";
    // cout << "target orientation: " << "\n" << target_point[3] << "\n" << target_point[4] << "\n" << target_point[5] << "\n";

    num_iterations = 0;
    int num_iterations_low_movement = 0;
    Vector3d target_pos_world = target_point.head(3);
    Vector3d target_ang_world = target_point.tail(3);

    // backup current angles
    perform_backup(robot_state);

    vector<string> joints_vec = robot_state.get_all_joints();
    if (set_random_angles) {
        vector<double> rand_angs;
        rand_angs.resize(6);

        // ((double) rand() / (RAND_MAX)) yields random number [0,1)
        // RAND_MAX is defined in a library

        // TODO: make this actually depend on joint limits, and test whether using full range of values is best
        // rand_angs[0] = ((double) rand() / (RAND_MAX) - 0.5) * 6.28;
        // rand_angs[1] = ((double) rand() / RAND_MAX) * 1.57;
        // rand_angs[2] = ((double) rand() / (RAND_MAX) - 0.5) * 4.72;
        // rand_angs[3] = ((double) rand() / (RAND_MAX) - 0.5) * 6.28;
        // rand_angs[4] = ((double) rand() / RAND_MAX) * 3.23 - 2.36;
        // rand_angs[5] = ((double) rand() / (RAND_MAX) - 0.5) * 6.28;

        rand_angs[0] = ((double) rand() / (RAND_MAX) - 0.5)*2*M_PI;
        rand_angs[1] = ((double) rand() / (RAND_MAX))*0.25*M_PI;
        rand_angs[2] = ((double) rand() / (RAND_MAX) - 0.5)*M_PI;
        rand_angs[3] = ((double) rand() / (RAND_MAX) - 0.5)*M_PI;;
        rand_angs[4] = ((double) rand() / (RAND_MAX) - 0.5)*M_PI;
        rand_angs[5] = ((double) rand() / (RAND_MAX) - 0.5)*2*M_PI;

        robot_state.set_joint_angles(rand_angs);
    }

    // Update transforms using FK
    FK(robot_state);
    
    // Get position of the end effector
    Vector3d ef_pos_world = robot_state.get_ef_pos_world();
    Vector3d ef_ang_world = robot_state.get_ef_ang_world();

    double dist = (ef_pos_world - target_pos_world).norm();
    double angle_dist = (ef_ang_world - target_ang_world).norm();

    Vector6d d_ef;

    // While we have not reached an acceptable solution
    while (dist > POS_THRESHOLD || (angle_dist > ANGLE_THRESHOLD && use_euler_angles)) {
        
        if (num_iterations_low_movement > MAX_ITERATIONS_LOW_MOVEMENT || num_iterations > MAX_ITERATIONS) {
            // cout << "final dist:" << dist << "\n";
            // cout << "final angle dist:" << angle_dist << "\n\n";

            Vector6d joint_angles;
            for (int i = 0; i < 6; ++i) {
                joint_angles(i) = robot_state.get_joint_angle(i);
            }

            // restore previous robot_state angles
            recover_from_backup(robot_state);

            return pair<Vector6d, bool> (joint_angles, false);
        }

        // d_ef is the vector we want the end effector to move in this step
        d_ef.head(3) = k_position_step * (target_point.head(3) - ef_pos_world);
        if (use_euler_angles) {
            d_ef.tail(3) = k_angle_step * (target_point.tail(3) - ef_ang_world);
        }
        else {
            d_ef.tail(3) = Vector3d{0, 0, 0};
        }

        // Move the end effector a small amount towards the target
        IK_step(robot_state, d_ef, use_euler_angles);
        
        ef_pos_world = robot_state.get_ef_pos_world();
        ef_ang_world = robot_state.get_ef_ang_world();

        double dist_original = dist;
        double angle_dist_original = angle_dist;

        dist = (ef_pos_world - target_pos_world).norm();
        angle_dist = (ef_ang_world - target_ang_world).norm();

        if (abs(dist - dist_original) < EPSILON_DIST && abs(angle_dist - angle_dist_original) < EPSILON_ANGLE_DIST) {
            ++num_iterations_low_movement;
        }
        else {
            num_iterations_low_movement = 0;
        }

        ++num_iterations;
    }

    // cout << angle_dist << "  <=  " << ANGLE_THRESHOLD << "\n";
    // cout << "bool: " << (angle_dist > ANGLE_THRESHOLD && use_euler_angles) << "\n";
    // cout << "AHH, safe checking!!!\n";

    // Vector3d ef_pos = robot_state.get_ef_pos_world();
    // cout << "End effector positions [ " << ef_pos(0) << " " << ef_pos(1) << " " << ef_pos(2) << "]\n";

    vector<double> angles_vec = robot_state.get_joint_angles();

    if (!is_safe(robot_state, angles_vec)){
        // cout << "Found IK solution, but solution is not safe!\n";

        recover_from_backup(robot_state);
        return pair<Vector6d, bool> (vecTo6d(angles_vec), false);
    }

    // restore robot_state to previous values
    recover_from_backup(robot_state);
    return pair<Vector6d, bool> (vecTo6d(angles_vec), true);
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
            Matrix4d n_xform = apply_joint_xform(robot_state, i, DELTA_THETA);

            Vector3d euler_angles;
            euler_angles = compute_euler_angles(n_xform.block(0,0,3,3));
            // cout << " blocking\n";
            Vector3d diff_angs = (euler_angles - ef_euler_world);
            Vector3d delta_angs = diff_angs / DELTA_THETA;

            Transpose<Vector3d> joint_col_euler = delta_angs.transpose();

            joint_col.tail(3) = joint_col_euler;
        }
        else {
            joint_col.tail(3) = Vector3d(0, 0, 0);
        }

        // write column i of the jacobian
        for (size_t j = 0; j < 6; ++j) {
            jacobian(j, i) = joint_col[j];
        }
    }

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
    vector<double> angle_vec;
    for (size_t i = 0; i < 6; ++i) {

        // don't move joint i if it's locked

        if (robot_state.get_joint_locked(i)) {
            d_theta[i] = 0;
        }

        vector<double> limits = robot_state.get_joint_limits(i);

        double angle = robot_state.get_joint_angle(i) + d_theta[i];

        // clip angle to within joint limits
        if (angle < limits[0]) {
            angle = limits[0];
        }
        else if (angle > limits[1]) {
            angle = limits[1];
        }

        angle_vec.push_back(angle);
    }

    // run forward kinematics
    robot_state.set_joint_angles(angle_vec);
    FK(robot_state);
}

bool KinematicsSolver::is_safe(ArmState &robot_state, const vector<double> &angles) {
    perform_backup(robot_state);

    // if any angles are outside bounds
    if (!limit_check(robot_state, angles)) {
        recover_from_backup(robot_state);
        return false;
    }

    // run FK algorithm to update state
    robot_state.set_joint_angles(angles);
    FK(robot_state);

    bool obstacle_free = robot_state.obstacle_free();

    recover_from_backup(robot_state);
    return obstacle_free;
}

bool KinematicsSolver::limit_check(ArmState &robot_state, const vector<double> &angles) {
    for (size_t i = 0; i < 6; ++i) {
        vector<double> limits = robot_state.get_joint_limits(i);
        
        // if any angle is outside of bounds
        if (!(limits[0] <= angles[i] && angles[i] < limits[1])) {
            return false;
        }
    }

    // if all angles are in bounds
    return true;
}

void KinematicsSolver::perform_backup(ArmState &robot_state) {
    vector<double> backup = robot_state.get_joint_angles();

    // save all angles to stack
    arm_state_backup.push(backup);
}

void KinematicsSolver::recover_from_backup(ArmState &robot_state) {
    if (arm_state_backup.empty()) {
        cout << "ERROR: no backup arm_state to revert to!\n";
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
