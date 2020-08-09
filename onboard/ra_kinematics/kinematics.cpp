#include "kinematics.hpp"


using namespace Eigen;

KinematicsSolver::KinematicsSolver(ArmState robot_state_in) : robot_state(robot_state_in), e_locked(false) {}

void KinematicsSolver::FK() {}

Matrix4d KinematicsSolver::apply_joint_xform(string joint, double theta) {}

pair<vector<double>, bool> KinematicsSolver::IK() {}

void KinematicsSolver::IK_step(Vector6d d_ef, bool use_euler_angles) {}

bool KinematicsSolver::is_safe(vector<double> angles) {}

bool KinematicsSolver::limit_check(vector<double> angles) {}