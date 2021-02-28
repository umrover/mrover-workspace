#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include "arm_state.hpp"
#include <stack>

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

static const int MAX_ITERATIONS = 500;
static constexpr double POS_THRESHOLD = 0.01;
static constexpr double ANGLE_THRESHOLD = 10.0;
static const int POS_WEIGHT = 1;
static constexpr double j_kp = 0.1;
static constexpr double j_kd = 0;
static constexpr double delta_theta = 0.0001;


class KinematicsSolver {

private:

    bool e_locked;

    stack<vector<double>> arm_state_backup;

    /**
     * Push the angles of robot_state into the arm_state_backup stack
     * */
    void perform_backup(ArmState &robot_state);

    /**
     * Pop a backup of the angles from robot_state and copy them into robot_state
     * Then run FK
     * */
    void recover_from_backup(ArmState &robot_state);

public:

    Vector3d target_pos_world;
    Vector3d target_angle_world;

    int get_pos_weight() {
        return POS_WEIGHT;
    }

    KinematicsSolver();

    Vector3d FK(ArmState &robot_state);

    Matrix4d apply_joint_xform(ArmState &robot_state, string joint, double theta);

    pair<Vector6d, bool> IK(ArmState &robot_state, Vector6d target_point, bool set_random_angles, bool use_euler_angles);

    void IK_step(ArmState &robot_state, Vector6d d_ef, bool use_pi, bool use_euler_angles);


    /**
     * @param angles the set of angles for a theoretical arm position
     * @return true if all angles are within bounds and don't cause collisions
     * */
    bool is_safe(ArmState &robot_state, vector<double> angles);

    /**
     * called by is_safe to check that angles are within bounds
     * @param angles the set of angles for a theoretical arm position
     * @return true if all angles are within bounds
     * */
    bool limit_check(ArmState &robot_state, const vector<double> &angles);

};



#endif