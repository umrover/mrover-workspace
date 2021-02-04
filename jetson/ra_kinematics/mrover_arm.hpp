#ifndef MROVER_ARM_H
#define MROVER_ARM_H
 
#include "json.hpp"
// #include <lcm/lcm-cpp.hpp>
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
#include "rover_msgs/TargetAngles.hpp"
#include "rover_msgs/LockJointE.hpp"
 
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
   json geom;
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
   MRoverArm(json &geom, lcm::LCM &lcm);
 
   void arm_position_callback(string channel, ArmPosition msg);
 
   void publish_config(vector<double> &config, string channel);
 
   void publish_transforms(ArmState state);
 
   void motion_execute_callback(string channel, MotionExecute msg);
 
   void target_orientation_callback(string channel, TargetOrientation msg);
 
   void plan_path(Vector6d goal);
 
   void execute_spline();
 
   void preview();
 
   void lock_e_callback(string channel, LockJointE msg);
 
   void k_enabled_callback(string channel, IkEnabled msg);
 
   void target_angles_callback(string channel, TargetAngles msg);
 
   void simulation_mode_callback(string channel, SimulationMode msg);
 
   void cartesian_control_callback(string channel, IkArmControl msg);
 
   void matrix_helper(double arr[4][4], const Matrix4d &mat);
 
};
 
 
#endif
