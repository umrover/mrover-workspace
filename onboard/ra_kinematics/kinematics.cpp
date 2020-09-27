#include "kinematics.hpp"


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
        // double theta = robot_state.angles[it->first];
    }
}

Matrix4d KinematicsSolver::apply_joint_xform(string joint, double theta) {}

pair<vector<double>, bool> KinematicsSolver::IK() {}

void KinematicsSolver::IK_step(Vector6d d_ef, bool use_euler_angles) {}

bool KinematicsSolver::is_safe(vector<double> angles) {}

bool KinematicsSolver::limit_check(vector<double> angles) {}