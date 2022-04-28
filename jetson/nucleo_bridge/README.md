Code to control the RA/SA, using Nucleo F303REs
===============================================
### About

nucleo_bridge is responsible for connecting the rover network's LCM interface with the RA/SA system's Nucleo controller i2c interface

main.cpp calls init() on the static LCMHandler class \
main.cpp calls init() on the static ControllerMap class \
main.cpp calls init() on the static I2C class \
main.cpp creates two threads to run an outgoing function and an incoming function
The outgoing function calls on the LCMHandler's handle_outgoing() function every millisecond
The incoming function calls on the LCMHandler's handle_incoming() function continuously

The ControllerMap class creates a hash table of virtual Controller objects from the config file located at "mrover-workspace/config_nucleo_bridge/controller_config.json".These virtual Controllers are used to contact the physical controller on the rover, across both RA/SA configurations.

The virtual Controller class is defined in Controller.h.\
Virtual Controllers store information about various controller-specific parameters (such as encoder cpr)\
The virtual Controller class also has functions representing the possible transactions that can be had with the physical controller. \
The virtual Controller will not attempt to communicate with its physical controller unless "activated" by an appropriate LCM message relayed by LCMHandler.h
(e.g. A virtual RA Controller will never attempt to communicate with its physical RA controller unless an RA-related LCM message is sent. This is to prevent multiple virtual Controller objects from trying to contact the same physical Controller object.)

LCMHandler.h is responsible for handling incoming and outgoing lcm messages. \
Incoming lcm messages will trigger functions which call the functions on the appropriate virtual Controllers. \
Outgoing lcm messages are triggered by a clock, which query the functions on the appropriate virtual Controllers for data.

I2C.h is responsible for translating communications by virtual Controllers into i2c transactions understood by the linux drivers.

The following watchdog is implemented: If the nucleos do not receive any I2C messages for a given amount of time (currently about 443 ms), then they reset.


### LCM Channels Publishing/Subscribed To
#### RA Open Loop \[Subscriber\] "/ra_openloop_cmd"
Message: [RAOpenLoopCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/RAOpenLoopCmd.lcm) \
Publisher: jetson/teleop \
Subscriber: jetson/nucleo_bridge

#### SA Open Loop \[Subscriber\] "/sa_openloop_cmd"
Message: [SAOpenLoopCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/SAOpenLoopCmd.lcm) \
Publisher: jetson/teleop \
Subscriber: jetson/nucleo_bridge

#### Carousel Open Loop \[Subscriber\] "/carousel_openloop_cmd"
Message: [CarouselOpenLoopCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/CarouselOpenLoopCmd.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/nucleo_bridge

#### RA Closed Loop \[Subscriber\] "/ra_ik_cmd"
Message: [RAPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/RAPosition.lcm) \
Publisher: jetson/ra_kinematics \
Subscriber: jetson/nucleo_bridge

#### SA Closed Loop \[Subscriber\] "/sa_ik_cmd"
Message: [SAPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/SAPosition.lcm) \
Publisher: jetson/ra_kinematics \
Subscriber: jetson/nucleo_bridge

#### Carousel Closed Loop \[Subscriber\] "/carousel_closedloop_cmd"
Message: [CarouselPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/CarouselPosition.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/nucleo_bridge

#### Mast Gimbal Open Loop \[Subscriber\] "/mast_gimbal_cmd"
Message: [MastGimbalCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/MastGimbalCmd.lcm) \
Publisher: jetson/teleop \
Subscriber: jetson/nucleo_bridge

#### Hand Open Loop \[Subscriber\] "/hand_openloop_cmd"
Message: [HandCmd.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/HandCmd.lcm) \
Publisher: jetson/teleop \
Subscriber: jetson/nucleo_bridge

#### Foot Open Loop \[Subscriber\] "/foot_openloop_cmd"
Message: [FootCmd.lcm](https://github.com/umrover/mrover-workspace/blob/rnucleo/rover_msgs/FootCmd.lcm) \
Publisher: jetson/teleop \
Subscriber: jetson/nucleo_bridge

#### RA Pos Data \[Publisher\] "/ra_pos_data"
Message: [RAPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/RAPosition.lcm) \
Publisher: jetson/nucleo_bridge \
Subscriber: jetson/ra_kinematics + base_station/gui

#### SA Pos Data \[Publisher\] "/sa_pos_data"
Message: [SAPosData.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/SAPosData.lcm) \
Publisher: jetson/nucleo_bridge \
Subscriber: jetson/ra_kinematics + base_station/kineval_stencil + base_station/gui

#### Carousel Pos Data \[Publisher\] "/carousel_pos_data"
Message: [CarouselData.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/CarouselData.lcm) \
Publisher: jetson/nucleo_bridge \
Subscriber: base_station/gui

#### Wrist Turn Count \[Publisher\] "/wrist_turn_count"
Message: [WristTurnCount.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/WristTurnCount.lcm) \
Publisher: jetson/nucleo_bridge \
Subscriber: base_station/gui + jetson/teleop

#### Carousel Calibration Data \[Publisher\] "/carousel_calib_data"
Message: [Calibrate.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Calibrate.lcm) \
Publisher: jetson/nucleo_bridge \
Subscriber: base_station/gui

#### RA Joint B Calibration Data \[Publisher\] "/ra_b_calib_data"
Message: [Calibrate.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Calibrate.lcm) \
Publisher: jetson/nucleo_bridge \
Subscriber: base_station/gui + jetson/ra_kinematics

#### SA Joint B Calibration Data \[Publisher\] "/sa_b_calib_data"
Message: [Calibrate.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Calibrate.lcm) \
Publisher: jetson/nucleo_bridge \
Subscriber: base_station/gui + jetson/ra_kinematics

#### Scoop Limit Switch Enable Cmd \[Subscriber\] "/scoop_limit_switch_enable_cmd"
Message: [ScoopLimitSwitchEnable.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ScoopLimitSwitchEnable.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/nucleo_bridge

#### Zero Carousel Cmd \[Subscriber\] "/carousel_zero_cmd"
Message: [Signal.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Signal.lcm) \
Publisher: base_station/gui \
Subscriber: jetson/nucleo_bridge

### Watchdog Timeout Period

See above, but the watchdog timeout period is currently set to 443 ms.

### Usage

To build nucleo_bridge use `$./jarvis build jetson/nucleo_bridge/ ` from the mrover-workspace directory.

To run nucleo_bridge use `$./jarvis exec jetson/nucleo_bridge/ `
from the mrover-workspace directory.

After initializing the LCM bus, I2C bus, and virtual Controller objects, the CLI will only show errors, since printing output to the console is time expensive. A blank CLI is a good thing.

To control the RA/SA through open-loop
* Ensure jetson/teleop is running on the same platform
* Ensure base_station/gui is running on the base station
* Operate the joysticks with the RA/SA Task page open on the base station GUI

To control the RA/SA through closed-loop
* Ensure jetson/teleop is running on the same platform
* Ensure jetson/ra_kinematics is running on the same platform
* Ensure base_station/kineval_stencil is running on the base station
* Input commands into the arm control page on the base station GUI

### Off Nominal Behavior Handling

None so far.

### Common Errors

This routine typically only thows one type of error, when it has issues communicating with the motor nucleos. They will have the form "<command> failed on channel"

#### <Command> failed on channel
The Nucleo that is unresponsive is the first digit of the channel. For example, if the error message says "activate failed on 10", check Nucleo 1. Typically this is because the wiring on the i2c bus has failed and needs to be fixed. Even after wiring is fixed or other related issues have been resolved, the Nucleos may stay unresponsive. To reset the Nucleo, press its reset button. If this does not fix nucleo_bridge, you may have to restart nucleo_bridge.

These communication errors can be caused by a failure anywhere on the i2c bus, so it is not unlikely that issues with only one Nucleo will cause the all the Nucleos to fail to communicate properly. 

nucleo_bridge will continue to attempt i2c bus transactions as commands come in from teleop, but currently has no other way to diagnose/remedy an i2c failure.

#### "Assertation failed" while initializing a virtual controller
There is an issue with mrover-workspace/config/nucleo_bridge/Controller.cpp. Resolve the configuration file's issues before running the program.

#### Encoder values resetting
If two devices share the same i2c address but their functions are continuously called, then one may see the encoder values changing. This is because when only one device for a specific i2c address can be live at a time and when each device is first made live, it's quadrature encoder value is adjusted to its absolute encoder value.


### ToDo
- [x] Test limit switch code for science arm
- [ ] Test limit switch code for standard robotic arm
- [X] Test joint b calibration code
- [ ] Investigate encoder issues
- [ ] Create new shield next year and make sure correct pins are traced
- [ ] Add LCM tests to test.cpp
- [ ] Zero-index after URC + CIRC (after Rosie)
- [ ] Verify wrist encoder CPR
- [ ] Perhaps custom nucleos (to replace the current 3 nucleos taking up so much space)
- [X] Create zero lcm with teleop for carousel motor
- [X] Create closed loop for carousel motor

### Notes

To reference the firmware that is loaded onto the Nucleos, [see this] (https://github.com/umrover/embedded-testbench/tree/motor_nucleo/motor_nucleo). An updated README for the firmware is coming soon, which will point out details such as watchdog timers and common issues.
