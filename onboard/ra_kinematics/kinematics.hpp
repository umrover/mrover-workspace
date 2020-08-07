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


    ArmState robot_state;
    bool e_locked;

public:

    KinematicsSolver(ArmState robot_state_in);

    void FK();

    Matrix4d apply_joint_xform(string joint, double theta);

    pair<vector<double>, bool> IK();

    void IK_step(Vector6d d_ef, bool use_euler_angles);

    bool is_safe(vector<double> angles);

    bool limit_check(vector<double> angles);

};



#endif