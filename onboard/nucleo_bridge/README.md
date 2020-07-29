Code to control the RA/SA, using Nucleo F303REs
===============================================
### About

nucleo_bridge is responsible for connecting the rover network's LCM interface with the RA/SA system's Nucleo controller i2c interface

main.cpp binds LCM channels to handler functions in frontend.h \
main.cpp also creates a hash table of virtual Controller objects that represent each physical controller on the rover, across both RA/SA configurations

The virtual Controller class is defined in controller.h.\
Virtual Controllers store information about various controller-specific parameters (such as encoder cpr)\
The virtual Controller class also has functions representing the possible transactions that can be had with the physical controller. \
The virtual Controller will not attempt to communicate with it's physical controller unless "activated" by an appropriate LCM message relayed by frontend.h
(e.g. A virtual RA Controller will never attempt to communicate with its physical RA controller unless an RA-related LCM message is sent)

frontend.h is responsible for handling incoming and outgoing lcm messages. \
Incoming lcm messages will trigger functions which call the functions on the appropriate virtual Controllers. \
Outgoing lcm messages are triggered by a clock, which query the functions on the appropriate virtual Controllers for data.

backend.h is responsible for translating communications by virtual Controllers into i2c transactions understood by the linux drivers.

### LCM Channels
#### RA Open Loop \[Subscriber\] "/ra_openloop_cmd"
Message: [RAOpenLoopCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/RAOpenLoopCmd.lcm) \
Publisher: onboard/teleop \
Subscriber: onboard/nucleo_bridge

#### SA Open Loop \[Subscriber\] "/sa_openloop_cmd"
Message: [SAOpenLoopCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/SAOpenLoopCmd.lcm) \
Publisher: onboard/teleop \
Subscriber: onboard/nucleo_bridge

#### RA Closed Loop \[Subscriber\] "/ik_ra_control"
Message: [ArmPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ArmPosition.lcm) \
Publisher: onboard/kinematics \
Subscriber: onboard/nucleo_bridge

#### SA Closed Loop \[Subscriber\] "/sa_openloop_cmd"
Message: [SAClosedLoopCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/SAClosedLoopCmd.lcm) \
Publisher: onboard/kinematics \
Subscriber: onboard/nucleo_bridge

#### Gimbal Open Loop \[Subscriber\] "/gimbal_openloop_cmd"
Message: [GimbalCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/GimbalCmd.lcm) \
Publisher: onboard/teleop \
Subscriber: onboard/nucleo_bridge

#### Hand Open Loop \[Subscriber\] "/hand_openloop_cmd"
Message: [HandCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/HandCmd.lcm) \
Publisher: onboard/teleop \
Subscriber: onboard/nucleo_bridge

#### Foot Open Loop \[Subscriber\] "/foot_openloop_cmd"
Message: [FootCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/FootCmd.lcm) \
Publisher: onboard/teleop \
Subscriber: onboard/nucleo_bridge

#### RA Zero \[Subscriber\] "/ra_zero_trigger"
Message: [RAZeroTrigger.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/RAZeroTrigger.lcm) \
Publisher:??? \
Subscriber: onboard/nucleo_bridge

#### SA Zero \[Subscriber\] "/sa_zero_trigger"
Message: [SAZeroTrigger.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/SAZeroTrigger.lcm) \
Publisher: ??? \
Subscriber: onboard/nucleo_bridge 

#### RA Pos Data \[Publisher\] "/arm_posiiton"
Message: [ArmPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ArmPosition.lcm) \
Publisher: onboard/nucleo_bridge \
Subscriber: onboard/kinematics

#### RA Pos Data \[Publisher\] "/sa_pos_data"
Message: [SAPosData.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/SAPosData.lcm) \
Publisher: onboard/nucleo_bridge \
Subscriber: onboard/kinematics

### Usage

To build nucleo_bridge use `$./jarvis build onboard/nucleo_bridge/ ` from the mrover-workspace directory.

To run nucleo_bridge use `$./jarvis exec onboard/nucleo_bridge/ `
from the mrover-workspace directory.

The CLI will only show errors, since printing output to the console is expensive in terms of time. A blank CLI is a good thing.

To control the RA/SA through open-loop
* Ensure onboard/teleop is running on the same platform
* Ensure base_station/gui is running on the base station
* Operate the joysticks with the RA/SA Task page open on the base station GUI

To control the RA/SA through closed-loop
* Ensure onboard/teleop is running on the same platform
* Ensure onboard/kinematics is running on the same platform
* Ensure base_station/kineval is running on the base station
* Input commands into the arm control page on the base station GUI

### Common Errors

This routine typically only thows one type of error, when it has issues communicating with the Nucleo motor controllers. They will have the form "command failed on channel"

The Nucleo that is unresponsive is the first digit of the channel minus 1. For example, if the error message says "activate failed on 10", check Nucleo 0. Typically this is because the wiring on the i2c bus has failed nd needs to be fixed. Even after wiring is fixed or other related issues have been resolved, the Nucleos,To reset the Nucleo, press it's power button. If this does not fix nucleo_bridge, you may have to restart nucleo_bridge.

These communication errors can be caused by a failure anywhere on the i2c bus, so it is not unlikely that issues with only one Nucleo will cause the all the Nucleos to fail to communicate properly. 

### Notes

To reference the firmware that is loaded onto the Nucleos, [see this] (https://github.com/raytitan/NucleoHAL/tree/master/Core). A README for the firmware is coming soon, which will point out details such as watchdog timers and common issues.

### Todo

- [ ] Re-enable disabled arm joints and lcm messages.
- [ ] Hardcode the encoder configurations, PID constants, and other miscellanious variables for each arm joint.
- [ ] Test predictive torques.


