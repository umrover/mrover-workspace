#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include "arm_state.hpp"

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;


class KinematicsSolver {

private:
    static const int MAX_ITERATIONS = 500;
    static const int POS_THRESHOLD = 0.01;
    static const int ANGLE_THRESHOLD = 10;
    static const int POS_WEIGHT = 1;
    static const double j_kp = 0.1;
    static const double j_kd = 0;
    static const double delta_theta = 0.0001;

    Vector3d target_pos_world;
    bool e_locked;

    ArmState robot_state;
    ArmState robot_ik;
    ArmState new_state;
    ArmState robot_safety;
    bool e_locked;

public:

    KinematicsSolver(ArmState robot_state_in);

    void FK(ArmState robot_state);

    Matrix4d apply_joint_xform(string joint, double theta);

    pair<vector<double>, bool> IK();

    void IK_step(Vector6d d_ef, bool use_euler_angles);

    bool is_safe(vector<double> angles);

    bool limit_check(vector<double> angles);

};



#endif