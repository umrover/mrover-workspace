#include "kinematics.hpp"
#include <cmath>


using namespace Eigen;

KinematicsSolver::KinematicsSolver(ArmState robot_state_in) : robot_state(robot_state_in), e_locked(false) {
    robot_ik = robot_state_in;
    new_state = robot_state_in;
    robot_safety = robot_state_in;
    // Try robot fk:
    FK(robot_state);
    Vector3d target_pos_world(0, 0, 0);
    e_locked = false;
}

void KinematicsSolver::FK(ArmState &robot_state) {
    // Set global transfprm as identity
    Matrix4d global_transform = Matrix4d::Identity();
    robot_state.set_link_transform(robot_state.links_json['base'], Matrix4d::Identity());
    Matrix4d parent_mat = Matrix4d::Identity();
    for (auto it = robot_state.joints.begin(); it != robot_state.joints.end(); ++it) {
        // PB: Check if this should be get_joint_pos_world or get_joint_pos_local:
        Vector3d xyz = robot_state.get_joint_pos_world(it->first);
        // PB: Check if this should be get_joint_axis or get_joint_axis_world:
        Vector3d rot_axis_child = robot_state.get_joint_axis(it->first);
        double theta = it->second->angle;

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

        
    }
}

Matrix4d KinematicsSolver::apply_joint_xform(string joint, double theta) {}

pair<vector<double>, bool> KinematicsSolver::IK() {}

void KinematicsSolver::IK_step(Vector6d d_ef, bool use_euler_angles) {

    // TODO vector<string> links = robot_ik.get_all_links()??
    vector<string> joints = robot_ik.get_all_joints();
    // TODO other declarations
    Vector3d ef_pos_world = robot_ik.get_ef_pos_world();

}

bool KinematicsSolver::is_safe(vector<double> angles) {
    vector<string> joints = robot_ik.get_all_joints();
    
    // if any angles are outside bounds
    // TODO check if we need to run FK before returning, regardless of limit_check
    if (!limit_check(angles, joints)) {
        return false;
    }
    
    // TODO check if we need to exclude joint f from angles
    /*for (int i = 0; i < joints.size(); ++i) {
        if (joints[i] != "joint_f") {}
    }*/

    // run FK algorithm to determine if there is a collision
    robot_safety.set_joint_angles(angles);
    FK(robot_safety);
    return robot_safety.obstacle_free();
}

bool KinematicsSolver::limit_check(const vector<double> &angles, const vector<string> &joints) {

    // TODO check that we should actually use all values of joint
    for (int i = 0; i < joints.size(); ++i) {
        map<string, double> limits = robot_safety.get_joint_limits(joints[i]);
        
        // if any angle is outside of bounds
        if (! (limits["lower"] <= angles[i] && angles[i] < limits["upper"])) {
            return false;
        }
    }
    // if all angles are in bounds
    return true;
}