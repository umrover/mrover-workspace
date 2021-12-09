#ifndef MROVER_ARM_H
#define MROVER_ARM_H

// 3rd party
#include <lcm/lcm-cpp.hpp>
#include "nlohmann/json.hpp"
#include <eigen3/Eigen/Dense>
#include "kluge/spline.h"

#include <deque>

#include "mrover_arm.hpp"
#include "arm_state.hpp"
#include "motion_planner.hpp"
#include "kinematics.hpp"

// LCM messages
#include "rover_msgs/ArmPosition.hpp"
#include "rover_msgs/MotionExecute.hpp"
#include "rover_msgs/FKTransform.hpp"
#include "rover_msgs/TargetOrientation.hpp"
#include "rover_msgs/IkEnabled.hpp"
#include "rover_msgs/SimulationMode.hpp"
#include "rover_msgs/IkArmControl.hpp"
#include "rover_msgs/LockJoints.hpp"
#include "rover_msgs/DebugMessage.hpp"
 
using namespace rover_msgs;
 
using nlohmann::json;
using namespace std;
using namespace Eigen;
 
typedef Matrix<double, 6, 1> Vector6d;

// percentage of path to calculate move time
static constexpr double D_SPLINE_T = 0.01;

// in ms, wait time for execute_spline loop
static constexpr double SPLINE_WAIT_TIME = 100;

// Angle in radians to determine when encoders are sending faulty values
static constexpr double ENCODER_ERROR_THRESHOLD = 0.2;

static constexpr size_t MAX_NUM_PREV_ANGLES = 5;
static constexpr size_t MAX_FISHY_VALS = 1;

static constexpr double ZERO_ENCODER_VALUE = 0;
static constexpr double ZERO_ENCODER_EPSILON = 0.00000001;


/**
* This is the MRoverArm class, responsible for
* initiating callback functions and sending calculations
* over LCM.
*/
class MRoverArm {
 
private:
    ArmState state;
    KinematicsSolver solver;
    MotionPlanner motion_planner;
    lcm::LCM &lcm_;
    bool enable_execute;
    bool sim_mode;
    bool ik_enabled;
    bool previewing;

    bool encoder_error;
    string encoder_error_message;

    vector< deque<double> > prev_angles;
    vector<bool> faulty_encoders;

public:

    /**
     * MRoverArm constructor
     * 
     * @param geom standard mrover config file for RA
     * @param lcm empty lcm object to communicate with other systems
     * */
    MRoverArm(json &geom, lcm::LCM &lcm);

    /**
     * Handle message with updated joint angles from encoders,
     * update state and call FK() to adjust transforms
     * 
     * @param channel expected: "/arm_position"
     * @param msg format: double joint_a, joint_b, ... , joint_f
     * */
    void arm_position_callback(string channel, ArmPosition msg);

    /**
     * Handle new target position by calculating angles and plotting path,
     * then preview path
     * 
     * @param channel expected: "/target_orientation"
     * @param msg float x, y, z, alpha, beta, gamma; bool use_orientation
     * */
    void target_orientation_callback(string channel, TargetOrientation msg);

    /**
     * Handle request to move arm through previously calculated path
     * 
     * @param channel expected: "/motion_execute"
     * @param msg format: bool preview
     * */
    void motion_execute_callback(string channel, MotionExecute msg);

    void ik_enabled_callback(string channel, IkEnabled msg);

    void simulation_mode_callback(string channel, SimulationMode msg);

    /**
     * Asynchronous function, when enable_execute is set to true:
     * Executes current path on physical rover, unless sim_mode is true
     * */
    void execute_spline();

    void preview();

    /**
    * Allows for locking individual joints
    * */
    void lock_joints_callback(string channel, LockJoints msg);

    /**
     * Handle request to go to specific set of angles
     * 
     * @param channel expected: "/preset_angles", but could handle others
     * @param msg format: double joint_a, joint_b, joint_c, joint_d, joint_e, joint_f
     * */
    void target_angles_callback(string channel, ArmPosition msg);

    void cartesian_control_callback(string channel, IkArmControl msg);

private:
    void publish_config(const vector<double> &config, string channel);

    void publish_transforms(const ArmState &state);

    void plan_path(Vector6d goal);

    void matrix_helper(double arr[4][4], const Matrix4d &mat);

    bool check_zero_encoder(const vector<double> &angles) const;

    bool check_joint_limits(const vector<double> &angles) const;
};


#endif
