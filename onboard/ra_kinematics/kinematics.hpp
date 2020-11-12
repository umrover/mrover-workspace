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

    ArmState robot_state;
    //ArmState robot_ik;
    //ArmState robot_safety;
    bool e_locked;

    stack<Vector6d> arm_state_backup;

    /**
     * Push the angles of robot_state into the arm_state_backup stack
     * */
    void perform_backup();

    /**
     * Pop a backup of the angles from robot_state and copy them into robot_state
     * Then run FK
     * */
    void recover_from_backup();

public:

    Vector3d target_pos_world;
    Vector3d target_angle_world;

    int get_pos_weight() {
        return POS_WEIGHT;
    }

    KinematicsSolver(const ArmState& robot_state_in);

    Vector3d FK();

    Matrix4d apply_joint_xform(string joint, double theta);

    pair<Vector6d, bool> IK(Vector6d target_point, bool set_random_angles, bool use_euler_angles);

    void IK_step(Vector6d d_ef, bool use_pi, bool use_euler_angles);

    pair<Vector6d, bool> IK_delta(Vector6d delta, int iterations);


    /**
     * @param angles the set of angles for a theoretical arm position
     * @return true if all angles are within bounds and don't cause collisions
     * */
    bool is_safe(Vector6d angles);

    /**
     * called by is_safe to check that angles are within bounds
     * @param angles the set of angles for a theoretical arm position
     * @return true if all angles are within bounds
     * */
    bool limit_check(const Vector6d &angles);

};



#endif