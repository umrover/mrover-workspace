Code to compute Inverse Kinematics and path planning for the robotic arm
========================================================================
### About ###

ra_kinematics is responsible for listening to GUI input (in base_station/kineval_stencil/) and subsequently sending arm angle configurations to ESW and the GUI.

main.cpp creates an MRoverArm object and sets various functions of this arm object to handle LCM messages (see below).

mrover_arm.hpp defines the MRoverArm class, which contains all functions of the ra_kinematics package that handle and publish LCM messages. MRoverArm contains ArmState, Kinematics, and MotionPlanner objects which together allow for successful inverse kinematics.

arm_state.hpp defines the ArmState class, which stores a particular state of the robotic arm. The state includes the physical geometry of each joint and link, a set of joint angles, and transformation matrices of each link.

kinematics.hpp defines the Kinematics class, which includes functions to interact with an ArmState parameter. Kinematics does not include any state of its own.
- FK() computes the ArmState object's end effector position/orientation and updates the arm's transformation matrices based on the arm's joint angles.
- IK() computes a set of joint angles that cause the end effector of the given ArmState to reach the target position.
- is_safe() checks that a given set of angles falls within the ArmState's joint limits and does not cause a collision.

motion_planner.hpp defines the MotionPlanner class, which includes functions to plan a path for a robotic arm.
- rrt_connect() finds a path between an ArmState parameter's current state and a set of target angles and stores this path as a member variable.
- get_spline_pos() returns the set of joint angles at a time between 0 and 1 for the last path planned with rrt_connect().

### Usage ###

To build the ra_kinematics package, run `$ ./jarvis build jetson/ra_kinematics/ ` from the mrover-workspace directory.

To run the ra_kinematics package, run `$ ./jarvis exec jetson/ra_kinematics/ ` from the mrover-workspace directory.

Currently, output is logged to stdout.

#### Additional Usage ####

If you are using the ra_kinematics package, you may also wish to run the associated GUI, which is found in the base_station/kineval_stencil/ directory. Alternatively, you may use base_station/gui/ to access lcm_echo. To build and run either, run \
`$ ./jarvis build base_station/kineval_stencil/ ` and `$ ./jarvis exec base_station/kineval_stencil/ ` \
or \
`$ ./jarvis build base_station/kineval_stencil/ ` and `$ ./jarvis exec base_station/kineval_stencil/ ` \
from the mrover-workspace directory.

Also, be sure to build and run lcm_bridge by running \
`$ ./jarvis build lcm_bridge/server/ ` and `$ ./jarvis exec lcm_bridge/server/ ` \
from the mrover-workspace directory.

### Known Possible Errors ###

If MotionPlanner is unable to find a path, undefined behavior could arise when trying to access points along the path, though this has not been witnessed. This should be addressed soon.

No other error conditions are currently known.

### Testing ###

The ra_kinematics package uses the EECS 280 testing framework. Testing files are found in the test directory.

To test the package, run `$ make tests ` from the ra_kinematics directory. To test more specific files, see the Makefile for more commands.

### LCM Publications ###

#### Arm Position \[Publisher\] "/arm_position" ####
Message: [ArmPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ArmPosition.lcm) \
Publisher: jetson/ra_kinematics \
Subscriber: jetson/ra_kinematics

When in Simulation Mode, jetson/ra_kinematics publishes to /arm_position to simulate receiving arm position message from nucleo_bridge.

#### Debug Message \[Publisher\] "/debug_message" ####
Message: [DebugMessage.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/DebugMessage.lcm) \
Publisher: jetson/ra_kinematics \
Subscriber: base_station/kineval_stencil

#### RA Closed Loop \[Publisher\] "/ik_ra_control" ####
Message: [ArmPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ArmPosition.lcm) \
Publisher: jetson/ra_kinematics \
Subscriber: jetson/nucleo_bridge

#### FK Transform \[Publisher\] "/fk_transform" ####
Message: [FKTransform.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/FKTransform.lcm) \
Publisher: jetson/ra_kinematics \
Subscriber: base_station/kineval_stencil

### LCM Subscriptions ###

#### Arm Position \[Subscriber\] "/arm_position" ####
Message: [ArmPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ArmPosition.lcm) \
Publisher: jetson/nucleo_bridge \
Subscriber: jetson/ra_kinematics

#### Target Orientation \[Subscriber\] "/target_orientation" ####
Message: [TargetOrientation.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/TargetOrientation.lcm) \
Publisher: base_station/kineval_stencil \
Subscriber: jetson/ra_kinematics

#### Lock Joints \[Subscriber\] "/locked_joints" ####
Message: [LockJoints.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/LockJoints.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/ra_kinematics

#### Arm Preset \[Subscriber\] "/arm_preset" ####
Message: [ArmPreset.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ArmPreset.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/ra_kinematics

#### Motion Execute \[Subscriber\] "/motion_execute" ####
Message: [MotionExecute.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/MotionExecute.lcm) \
Publisher: base_station/kineval_stencil \
Subscriber: jetson/ra_kinematics

#### Simulation Mode \[Subscriber\] "/simulation_mode" ####
Message: [SimulationMode.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/SimulationMode.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/ra_kinematics

#### Arm Control State \[Subscriber\] "/arm_control_state" ####
Message: [ArmControlState.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ArmControlState.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/ra_kinematics

#### Use Orientation \[Subscriber\] "/use_orientation" ####
Message: [UseOrientation.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/UseOrientation.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/ra_kinematics

#### Zero Position \[Subscriber\] "/zero_position" ####
Message: [ArmControlState.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ZeroPosition.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/ra_kinematics

#### Arm Adjustments \[Subscriber\] "/arm_adjustments" ####
Message: [ArmAdjustments.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ArmAdjustments.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/ra_kinematics
