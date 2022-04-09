#ifndef MROVER_ARM_H
#define MROVER_ARM_H

// 3rd party
#include <lcm/lcm-cpp.hpp>
#include "nlohmann/json.hpp"
#include <eigen3/Eigen/Dense>
#include "kluge/spline.h"

#include <deque>
#include <mutex>

#include "mrover_arm.hpp"
#include "arm_state.hpp"
#include "motion_planner.hpp"
#include "kinematics.hpp"

// LCM messages
#include "rover_msgs/RAPosition.hpp"
#include "rover_msgs/SAPosition.hpp"
#include "rover_msgs/MotionExecute.hpp"
#include "rover_msgs/FKTransform.hpp"
#include "rover_msgs/TargetOrientation.hpp"
#include "rover_msgs/IkEnabled.hpp"
#include "rover_msgs/SimulationMode.hpp"
#include "rover_msgs/IkArmControl.hpp"
#include "rover_msgs/LockJoints.hpp"
#include "rover_msgs/DebugMessage.hpp"
#include "rover_msgs/ArmControlState.hpp"
#include "rover_msgs/RAOpenLoopCmd.hpp"
#include "rover_msgs/HandCmd.hpp"
#include "rover_msgs/SAOpenLoopCmd.hpp"
#include "rover_msgs/FootCmd.hpp"
#include "rover_msgs/Signal.hpp"
#include "rover_msgs/UseOrientation.hpp"
#include "rover_msgs/ArmPreset.hpp"
#include "rover_msgs/ArmPresetPath.hpp"
#include "rover_msgs/CustomPreset.hpp"
#include "rover_msgs/ArmAdjustments.hpp"
#include "rover_msgs/WristTurnCount.hpp"

using namespace rover_msgs;
 
using nlohmann::json;
using namespace Eigen;
 
typedef Matrix<double, 6, 1> Vector6d;

// percentage of path to calculate move time
static constexpr double D_SPLINE_T = 0.05;
static constexpr double MAX_SPLINE_T_IT = 0.02;

// in ms, wait time for execute_spline loop
static constexpr int SPLINE_WAIT_TIME = 20;

// Angle in radians to determine when encoders are sending faulty values
static constexpr double ENCODER_ERROR_THRESHOLD = 0.1;

static constexpr size_t MAX_NUM_PREV_ANGLES = 5;
static constexpr size_t MAX_FISHY_VALS = 1;

static constexpr double DUD_ENCODER_EPSILON = 0.005;

//Angle in radians that physical arm can be without causing problems
static constexpr double ACCEPTABLE_BEYOND_LIMIT = 0.05;

static constexpr double JOINT_B_STABILIZE_MULTIPLIER = 0.5;
static constexpr double JOINT_B_STABILIZE_BAD_MULTIPLIER = 0.96;

/**
* This is the MRoverArm class, responsible for
* initiating callback functions and sending calculations
* over LCM.
*/
class MRoverArm {
protected:
    ArmState arm_state;
    KinematicsSolver solver;
    lcm::LCM &lcm_;

    bool sim_mode;

    enum ControlState {
        OFF,                // Not in closed-loop mode
        WAITING_FOR_TARGET, // In closed-loop mode, waiting for target position
        CALCULATING,        // Running IK and motion planning
        PREVIEWING,         // Showing the preview on hypothetical arm
        READY_TO_EXECUTE,   // Previewed, ready for execution
        EXECUTING           // Executing arm movement
    };

    ControlState control_state;

    std::mutex encoder_angles_sender_mtx;
 
private:
    MotionPlanner motion_planner;

    bool zero_encoders;

    bool encoder_error;
    std::string encoder_error_message;

    std::vector< std::deque<double> > prev_angles;
    std::vector<bool> faulty_encoders;
    
    std::vector<double> DUD_ENCODER_VALUES;

public:

    /**
     * MRoverArm constructor
     * 
     * @param geom standard mrover config file for RA
     * @param lcm empty lcm object to communicate with other systems
     * */
    MRoverArm(json &geom, lcm::LCM &lcm);

    virtual ~MRoverArm() = default;

    /**
     * Handle message with updated joint angles from encoders,
     * update arm_state and call FK() to adjust transforms
     * 
     * @param channel expected: "/ra_position"
     * @param msg format: double joint_a, joint_b, ... , joint_f   
     * */
    void arm_position_callback(std::string channel, RAPosition msg);

    /**
     * Handle message with updated joint angles from encoders,
     * update arm_state and call FK() to adjust transforms
     * 
     * @param channel expected: "/sa_position"
     * @param msg format: double joint_a, joint_b, joint_c, joint_e     
     * */
    void arm_position_callback(std::string channel, SAPosition msg);
    
    /**
     * Handle message meant to update control_state
     * 
     * @param channel expected: "/arm_control_state"
     * @param msg format: string state ("open-loop", "closed-loop", or "off")
     */
    void ra_control_callback(std::string channel, ArmControlState msg);

    /**
     * Handle request to move arm through previously calculated path, or to cancel
     * 
     * @param channel expected: "/motion_execute"
     * @param msg format: bool execute
     * */
    void motion_execute_callback(std::string channel, MotionExecute msg);

    /**
     * Handle request to move in or out of simulation mode
     * 
     * @param channel expected: "/simulation_mode"
     * @param msg format: bool sim_mode
     */
    void simulation_mode_callback(std::string channel, SimulationMode msg);

    /**
     * Handle request to set current position as the zero position
     * 
     * @param channel expected: "/zero_position"
     * @param msg format: empty
     */
    void zero_position_callback(std::string channel, Signal msg);

    /**
     * Handle request to go to a preset position
     * 
     * @param channel expected: "/custom_preset"
     * @param msg format: string preset
     */
    void custom_preset_callback(std::string channel, CustomPreset msg);

    /**
     * Handle request to lock specific joints
     * 
     * @param channel expected: "/locked_joints"
     * @param msg format: bool joint_a ... joint_f
     */
    virtual void lock_joints_callback(std::string channel, LockJoints msg) = 0;

    /**
     * Handle request to go to a preset position
     * 
     * @param channel expected: "/arm_preset"
     * @param msg format: string preset
     */
    virtual void arm_preset_callback(std::string channel, ArmPreset msg) = 0;

    /**
     * Update arm_state and call FK() to adjust transforms
     * 
     * @param angles arm position from encoders
     * @param standard true for standard arm, false for science arm
     */
    void set_arm_position(std::vector<double> &angles);

    /**
     * Asynchronous function, runs when control_state is "EXECUTING"
     * Executes current path on physical rover, unless sim_mode is true
     * */
    void execute_spline();

    /**
     * Asynchronous function, runs when sim_mode is true
     * Sends mock encoder values based on arm_state
     */
    virtual void encoder_angles_sender() = 0;

protected:

    virtual void publish_config(const std::vector<double> &config, std::string channel) = 0;

    virtual void send_kill_cmd() = 0;

    void plan_path(ArmState& hypo_state, std::vector<double> &goal);
    
    void preview(ArmState& hypo_state);

    void publish_transforms(const ArmState& arbitrary_state);

    void check_dud_encoder(std::vector<double> &angles) const;

    void check_joint_limits(std::vector<double> &angles);

    void set_to_closed_loop();

    bool interrupt(ControlState expected_state, std::string action);
};

class StandardArm : public MRoverArm {
public:
    StandardArm(json &geom, lcm::LCM &lcm);

    ~StandardArm() = default;
    
    void lock_joints_callback(std::string channel, LockJoints msg) override;

    void arm_preset_callback(std::string channel, ArmPreset msg) override;

    void encoder_angles_sender() override;

    /**
     * Handle new target position by calculating angles and plotting path,
     * then preview path
     * 
     * @param channel expected: "/target_orientation" or "/arm_adjustments"
     * @param msg float x, y, z, alpha, beta, gamma
     * */
    void target_orientation_callback(std::string channel, TargetOrientation msg);

    /**
     * Handle request to go to a preset position
     * 
     * @param channel expected: "/wrist_turn_count_callback"
     * @param msg format: int8_t turnCount
     */
    void wrist_turn_count_callback(std::string channel, WristTurnCount msg);

    /**
     * Handle request to move in or out of simulation mode
     * 
     * @param channel expected: "/use_orientation"
     * @param msg format: bool use_orientation
     */
    void use_orientation_callback(std::string channel, UseOrientation msg);

    /**
     * Calculate new target orientation using current pos and adjustments and
     * call target_orientation_callback 
     * 
     * @param channel expected: "/arm_adjustments"
     * @param msg float x, y, z, alpha, beta, gamma
     * */
    void arm_adjust_callback(std::string channel, ArmAdjustments msg);


private:

    /**
     * Used by arm_preset_callback() to go to a specific set of angles
     * @param msg format: double joint_a, joint_b, joint_c, joint_d, joint_e, joint_f
     * */
    void go_to_target_angles(RAPosition msg);

    void publish_config(const std::vector<double> &config, std::string channel) override;

    void send_kill_cmd() override;

    int wrist_turn_count;
    bool use_orientation;
};

class ScienceArm : public MRoverArm {
public:

    ScienceArm(json &geom, lcm::LCM &lcm);

    ~ScienceArm() = default;
    
    void lock_joints_callback(std::string channel, LockJoints msg) override;

    void arm_preset_callback(std::string channel, ArmPreset msg) override;

    /**
     * Handle request to go to various positions on a preset path
     * 
     * @param channel expected: "/arm_preset_path"
     * @param msg format: string preset
     */
    void arm_preset_path_callback(std::string channel, ArmPresetPath msg);

    void encoder_angles_sender() override;

private:

    /**
     * Used by arm_preset_callback() to go to a specific set of angles
     * @param msg format: double joint_a, joint_b, joint_c, joint_e
     * */
    void go_to_target_angles(SAPosition msg);

    void publish_config(const std::vector<double> &config, std::string channel) override;

    void send_kill_cmd() override;
};


#endif
