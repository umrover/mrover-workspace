#ifndef MROVER_ARM_H
#define MROVER_ARM_H
 
#include "json.hpp"
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>
#include "mrover_arm.hpp"
#include "arm_state.hpp"
#include "motion_planner.hpp"
#include "kinematics.hpp"
#include "spline.h"
#include "rover_msgs/ArmPosition.hpp"
#include "rover_msgs/MotionExecute.hpp"
#include "rover_msgs/FKTransform.hpp"
#include "rover_msgs/TargetOrientation.hpp"
#include "rover_msgs/IkEnabled.hpp"
#include "rover_msgs/SimulationMode.hpp"
#include "rover_msgs/IkArmControl.hpp"
#include "rover_msgs/LockJointE.hpp"
#include "rover_msgs/DebugMessage.hpp"
 
using namespace rover_msgs;
 
using nlohmann::json;
using namespace std;
using namespace Eigen;
 
typedef Matrix<double, 6, 1> Vector6d;
 
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
   // current_spline = [];  // list
   lcm::LCM &lcm_;
   bool done_previewing;
   bool enable_execute;
   bool sim_mode;
   bool ik_enabled;
 
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

    void lock_e_callback(string channel, LockJointE msg);

    /**
     * Handle request to go to specific set of angles
     * 
     * @param channel expected: "/preset_angles", but could handle others
     * @param msg format: double joint_a, joint_b, joint_c, joint_d, joint_e, joint_f
     * */
    void target_angles_callback(string channel, ArmPosition msg);

    void cartesian_control_callback(string channel, IkArmControl msg);

private:
    void publish_config(vector<double> &config, string channel);

    void publish_transforms(const ArmState &state);

    void plan_path(Vector6d goal);

    void matrix_helper(double arr[4][4], const Matrix4d &mat);
};
 
 
#endif
