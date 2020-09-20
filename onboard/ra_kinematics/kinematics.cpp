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

void KinematicsSolver::FK(ArmState robot_state) {}

Matrix4d KinematicsSolver::apply_joint_xform(string joint, double theta) {}

pair<vector<double>, bool> KinematicsSolver::IK() {}

void KinematicsSolver::IK_step(Vector6d d_ef, bool use_euler_angles) {}

bool KinematicsSolver::is_safe(vector<double> angles) {}

bool KinematicsSolver::limit_check(vector<double> angles) {}