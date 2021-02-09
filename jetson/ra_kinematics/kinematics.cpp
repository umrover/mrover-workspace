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

KinematicsSolver::KinematicsSolver() : e_locked(false) { }

Vector3d KinematicsSolver::FK(ArmState &robot_state) {
    // Set global transform as identity
    Matrix4d global_transform = Matrix4d::Identity();
    // TODO: Edit name of base ("chassis-a") as necessary:
    robot_state.set_link_transform("base", Matrix4d::Identity());
    Matrix4d parent_mat = Matrix4d::Identity();
    for (auto it = robot_state.joints.begin(); it != robot_state.joints.end(); ++it) {
        Vector3d xyz = robot_state.get_joint_pos_local(it->first);
        Vector3d rot_axis_child = robot_state.get_joint_axis(it->first);
        double theta = it->second.angle;

        Matrix4d rot_theta = Matrix4d::Identity();
        Matrix4d trans = Matrix4d::Identity();

        double ctheta = cos(theta);
        double stheta = sin(theta);

        // PB: This needs to be tested for correct indice manipulation
        trans.block(0,3,3,1) = xyz;

        // Make rotation about rot axis by theta:
        if (rot_axis_child(0) == 1) {
            rot_theta(1,1) = ctheta;
            rot_theta(2,1) = stheta;
            rot_theta(1,2) = -1*stheta;
            rot_theta(2,2) = ctheta;
        }
        else if(rot_axis_child(1) == 1) {
            rot_theta(0,0) = ctheta;
            rot_theta(2,0) = -1*stheta;
            rot_theta(0,2) = stheta;
            rot_theta(2,2) = ctheta;
        }
        else if(rot_axis_child(2) == 1) {
            rot_theta(0,0) = ctheta;
            rot_theta(1,0) = stheta;
            rot_theta(0,1) = -1*stheta;
            rot_theta(1,1) = ctheta;
        }
        else if (rot_axis_child(2) == -1) {
            rot_theta(0,0) = ctheta;
            rot_theta(0,1) = stheta;
            rot_theta(1,0) = -1*stheta;
            rot_theta(1,1) = ctheta;
        }

        Matrix4d T = trans*rot_theta;
        Matrix4d global_transform = parent_mat*T;

        string link = robot_state.get_child_link(it->first);

        robot_state.set_joint_transform(it->first, global_transform);
        robot_state.set_link_transform(link, global_transform);

        parent_mat = global_transform;
    }
    // Set ef position and orientation:
    Vector3d ef_xyz = robot_state.get_ef_pos_world();
    Matrix4d T = Matrix4d::Identity();
    T.block(0,3,3,1) = ef_xyz;
    global_transform = parent_mat*T;
    robot_state.set_ef_transform(global_transform);
    Vector4d mult(0,0,0,1);
    Vector4d ef_pos_world = global_transform*mult;
    Vector3d set_ef_pos (ef_pos_world(0), ef_pos_world(1), ef_pos_world(2));
    robot_state.set_ef_pos_world(set_ef_pos);

    Vector3d prev_com(0,0,0);
    double total_mass = 0;

    for (auto it = robot_state.joints.rbegin(); it != robot_state.joints.rend(); ++it) {
        Vector3d joint_pos = robot_state.get_joint_pos_world(it->first);
        Vector3d com_cur_link = robot_state.get_joint_com(it->first);
        double cur_link_mass = robot_state.get_joint_mass(it->first);

        // Calculate center of mass of current link:
        Vector3d curr_com = prev_com * total_mass + com_cur_link * cur_link_mass;
        total_mass += cur_link_mass;
        curr_com /= total_mass;

        // Calculate torque for current joint:
        Vector3d r = curr_com - joint_pos;
        Vector3d rot_axis_world = robot_state.get_joint_axis_world(it->first);
        Vector3d torque = calculate_torque(r, total_mass, rot_axis_world);
        robot_state.set_joint_torque(it->first, torque);
        prev_com = curr_com;
    }
    Vector3d return_vec(ef_pos_world(0), ef_pos_world(1), ef_pos_world(2));
    return return_vec;
}

Matrix4d KinematicsSolver::apply_joint_xform(ArmState &robot_state, string joint, double theta) {

    // deep copy position of joint
    double xyz[3];
    double *data = robot_state.get_joint_pos_world(joint).data();
    for (size_t i = 0; i < 3; ++i) {
        xyz[i] = data[i];
    }

    // get intended direction of rotation
    double *rot_axis_child = robot_state.get_joint_axis(joint).data();

    Matrix4d rot_theta = Matrix4d::Identity();
    Matrix4d trans = Matrix4d::Identity();

    double ctheta = cos(theta);
    double stheta = sin(theta);

    // copy trans from position
    trans(0, 3) = xyz[0];
    trans(1, 3) = xyz[1];
    trans(2, 3) = xyz[2];

    double rot_arr_one[] = {1, 0, 0};
    double rot_arr_two[] = {0, 1, 0};
    double rot_arr_three[] = {0, 0, 1};
    double rot_arr_four[] = {0, 0, -1};


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
    else if (rot_axis_child == rot_arr_four) {
        rot_theta(0, 0) = ctheta;
        rot_theta(0, 1) = stheta;
        rot_theta(1, 0) = -stheta;
        rot_theta(1, 1) = ctheta;
    }

    Matrix4d parent_mat = robot_state.get_ef_transform();
    Matrix4d T = trans * rot_theta;
    return parent_mat * T;
}

pair<Vector6d, bool> KinematicsSolver::IK(ArmState &robot_state, Vector6d target_point, bool set_random_angles, bool use_euler_angles) {
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
    cout << "Running IK!" << endl;
    cout << "target point: " << target_point(0) << " " <<  target_point(1) << " " <<  target_point(2)
         << " " <<  target_point(3) << " " <<  target_point(4) << " " <<  target_point(5) << "\n";

    int num_iterations = 0;
    Vector3d target_pos_world = target_point.head(3);
    Vector3d target_ang_world = target_point.tail(3);

    FK(robot_state);
    cout << "FK RAN!!\n";

    // backup current angles

    perform_backup(robot_state);

    vector<string> joints_vec = robot_state.get_all_joints();
    vector<string> links_vec = robot_state.get_all_links();
    if (set_random_angles) {
        // ((double) rand() / (RAND_MAX)) yields random number [0,1)
        Vector6d rand_angs;
        rand_angs(0) = ((double) rand() / (RAND_MAX) - 0.5)*2*M_PI;
        rand_angs(1) = ((double) rand() / (RAND_MAX))*0.25*M_PI;
        rand_angs(2) = ((double) rand() / (RAND_MAX) - 0.5)*M_PI;
        rand_angs(3) = ((double) rand() / (RAND_MAX) - 0.5)*M_PI;;
        rand_angs(4) = ((double) rand() / (RAND_MAX) - 0.5)*M_PI;
        rand_angs(5) = ((double) rand() / (RAND_MAX) - 0.5)*2*M_PI;

        int j_idx = 0;
        for (auto it = robot_state.joints.begin(); it != robot_state.joints.end(); ++it) {
            it->second.angle = rand_angs(j_idx);
            ++j_idx;
        }
        // Update transforms using FK:
        FK(robot_state);
    }
    // Get position the end effector:
    Vector3d ef_ang_world = robot_state.get_ef_ang_world();
    Vector3d ef_pos_world = robot_state.get_ef_pos_world();
    
    Vector6d ef_vec_world;
    ef_vec_world.head(3) = ef_pos_world;
    ef_vec_world.tail(3) = ef_ang_world;

    double dist = (ef_pos_world - target_pos_world).norm();
    double angle_dist = (ef_ang_world - target_ang_world).norm();

    double ef_v = 0;

    cout << "Current EF position: ";
    for (int i = 0; i < 3; ++i) {
        cout << ef_pos_world(i) << " ";
    }
    for (int i = 0; i < 3; ++i) {
        cout << ef_ang_world(i) << " ";
    }
    cout << "\n";
    cout << "Current Joint Angles: ";
    for (auto it = robot_state.joints.begin(); it != robot_state.joints.end(); ++it) {
        cout << it->second.angle << " ";
    }
    cout << "\n";


    while (dist > POS_THRESHOLD or angle_dist > ANGLE_THRESHOLD) {
        if (num_iterations > MAX_ITERATIONS) {
            cout << "Max ik iterations hit\n";
            vector<double> ef_pos = robot_state.get_ef_pos_and_euler_angles();
            cout << "position reached: ";
            for (int i = 0; i < 6; ++i) {
                cout << ef_pos[i] << " ";
            }
            cout << "\n";
            for (int i = 0; i < 6; ++i) {
                cout << target_point(i) << " ";
            }
            cout << "\n";
            Vector6d joint_angles;
            int index = 0;
            for (auto it = robot_state.joints.begin(); it != robot_state.joints.end(); ++it) {
                joint_angles(index++) = it->second.angle;
            }

            // restore previous robot_state angles
            recover_from_backup(robot_state);

            return pair<Vector6d, bool> (joint_angles, false);
        }
        Vector6d ef_to_target_b_weights = target_point - ef_vec_world;
        Vector6d ef_to_target_vec_world;
        ef_to_target_vec_world.head(3) = ef_to_target_b_weights.head(3) * get_pos_weight();
        ef_to_target_vec_world.tail(3) = ef_to_target_b_weights.tail(3);

        // d_ef is the vector we want the ef to move in this step 
        // d_ef is a 6d vector
        Vector6d d_ef = j_kp * ef_to_target_b_weights - j_kd * (ef_v * (ef_to_target_vec_world)/ef_to_target_vec_world.norm());
        IK_step(robot_state, d_ef, true, use_euler_angles);
        // Iterate:
        // ef_vec_world is a 6d vector:
        ef_vec_world = vecTo6d(robot_state.get_ef_pos_and_euler_angles());
        ef_ang_world = ef_vec_world.tail(3);

        dist = (ef_pos_world - target_pos_world).norm();
        angle_dist = (ef_ang_world - target_ang_world).norm();

        ++num_iterations;
    }
    cout << "AHH, safe checking!!!\n";

    vector<double> angles_vec;
    int index = 0;
    auto joint_angs = robot_state.get_joint_angles();
    for (auto it = joint_angs.begin(); it != joint_angs.end(); ++it) {
        angles_vec[index++] = it->second;
    }
    if (!is_safe(robot_state, angles_vec)){
        cout << "ik not safe\n";
        return pair<Vector6d, bool> (vecTo6d(angles_vec), false);
    }
    cout << "ik safe about to return\n";

    // restore robot_state to previous values
    recover_from_backup(robot_state);
    return pair<Vector6d, bool> (vecTo6d(angles_vec), false);
}

void KinematicsSolver::IK_step(ArmState &robot_state, Vector6d d_ef, bool use_pi, bool use_euler_angles) {

    // may cause issue creating a type that refers to private member Link
    auto links = robot_state.links;
    vector<string> joints = robot_state.get_all_joints();
    Vector6d ef_world = vecTo6d(robot_state.get_ef_pos_and_euler_angles());
    Vector3d ef_pos_world = robot_state.get_ef_pos_world();
    Vector3d ef_euler_world(ef_world(3), ef_world(4), ef_world(5));

    MatrixXd jacobian(6, 6);
    jacobian.setZero();

    // 6-D matrix
    MatrixXd jacobian_inverse;

    for (int i = 0; i < (int)joints.size(); ++i) {

        // don't move joint e if it's locked
        if (e_locked && joints[i] == "joint_e") {
            for (int j = 0; j < 6; ++j) {
                jacobian(j, i) = 0;
            }
        }

        // otherwise, calculate the jacobian
        else {
            // Error occuring somewhere below this line
            Vector3d rot_axis_local = robot_state.get_joint_axis(joints[i]);
            Matrix4d joint_xform = robot_state.get_joint_transform(joints[i]);
            Vector3d joint_pos_world = robot_state.get_link_point_world(robot_state.get_child_link(joints[i]));
            Vector3d rot_axis_world = apply_transformation(joint_xform, rot_axis_local);
            Vector3d joint_to_ef_vec_world = ef_pos_world - joint_pos_world;
            Transpose<Vector3d> joint_col_xyz = (rot_axis_world.cross(joint_to_ef_vec_world)).transpose();

            Vector6d joint_col;

            // calculate end effector orientation jacobian values
            if (use_euler_angles) {
                Matrix4d n_xform = apply_joint_xform(robot_state, joints[i], delta_theta);

                Vector3d euler_angles;
                euler_angles = compute_euler_angles(n_xform.block(0,0,3,3));
                // cout << " blocking\n";
                Vector3d diff_angs = euler_angles - ef_euler_world;
                Vector3d delta_angs = diff_angs / delta_theta;

                joint_col_xyz = joint_col_xyz.transpose();

                Transpose<Vector3d> joint_col_euler = delta_angs.transpose();

                joint_col.head(3) = joint_col_xyz;
                joint_col.tail(3) = joint_col_euler;
            }
            else {
                // cout << "Attempting joint_col comb\n";
                joint_col.head(3) = joint_col_xyz;
                joint_col.tail(3) = Vector3d();
            }
            for (size_t j = 0; j < 6; ++j) {
                jacobian(j, i) = joint_col[j];
            }
        }
    }

    cout << "Filled in the Jacobian matrix\n";

    // if using pseudo inverse (usually corresponds to using euler angles)
    if (use_pi) {
        jacobian_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    }

    // otherwise, simply transpose the vector
    else {
        jacobian_inverse = jacobian.transpose();
    }

    cout << "Applied use_pi\n";

    Vector6d d_theta = jacobian_inverse * d_ef;

    vector<double> angle_vec;
    
    // find the angle of each joint
    for (int i = 0; i < (int)joints.size(); ++i) {
        angle_vec.push_back(robot_state.get_joint_angles()[joints[i]] + d_theta[i]);

        map<string, double> limits = robot_state.get_joint_limits(joints[i]);

        if (angle_vec[i] < limits["lower"]) {
            angle_vec.push_back(limits["lower"]);
        }
        else if (angle_vec[i] < limits["upper"]) {
            angle_vec.push_back(limits["upper"]);
        }
    }

    // cout << "Found the angle of each joint\n";

    // run forward kinematics
    robot_state.set_joint_angles(angle_vec);
    FK(robot_state);
    // cout << "Ran forward kinematics from IK step function\n";
}

bool KinematicsSolver::is_safe(ArmState &robot_state, vector<double> angles) {
    
    perform_backup(robot_state);

    // if any angles are outside bounds
    if (!limit_check(robot_state, angles)) {
        recover_from_backup(robot_state);
        return false;
    }

    // run FK algorithm to determine if there is a collision
    robot_state.set_joint_angles(angles);
    FK(robot_state);
    bool obstacle_free = robot_state.obstacle_free();

    recover_from_backup(robot_state);
    return obstacle_free;
}

bool KinematicsSolver::limit_check(ArmState &robot_state, const vector<double> &angles) {
    vector<string> joints = robot_state.get_all_joints();

    for (int i = 0; i < (int)joints.size(); ++i) {
        map<string, double> limits = robot_state.get_joint_limits(joints[i]);
        
        // if any angle is outside of bounds
        if (!(limits["lower"] <= angles[i] && angles[i] < limits["upper"])) {
            return false;
        }
    }

    // if all angles are in bounds
    return true;
}

void KinematicsSolver::perform_backup(ArmState &robot_state) {
    vector<double> backup;

    for (auto const& pair : robot_state.get_joint_angles()) {
        backup.push_back(pair.second);
    }

    arm_state_backup.push(backup);
}

void KinematicsSolver::recover_from_backup(ArmState &robot_state) {
    if (arm_state_backup.empty()) {
        cout << "ERROR: no backup arm_state to revert to!\n";
    }
    else {
        robot_state.set_joint_angles(arm_state_backup.top());
        arm_state_backup.pop();

        FK(robot_state);
    }
}