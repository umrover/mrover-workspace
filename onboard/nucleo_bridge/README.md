Code to control the RA/SA, using Nucleo F303REs
===============================================
### About

nucleo_bridge is responsible for connecting the rover network's LCM interface with the RA/SA system's Nucleo controller i2c interface

main.cpp binds LCM channels to handler functions in frontend.h
main.cpp also creates a hash table of virtual Controller objects that represent each physical controller on the rover, across both RA/SA configurations

The virtual Controller class is defined in controller.h.
Virtual Controllers store information about various controller-specific parameters (such as encoder cpr)
The virtual Controller class also has functions representing the possible transactions that can be had with the physical controller.
The virtual Controller will not attempt to communicate with it's physical controller unless "activated" by an appropriate LCM message relayed by frontend.h
(e.g. A virtual RA Controller will never attempt to communicate with its physical RA controller unless an RA-related LCM message is sent)

frontend.h is responsible for handling incoming and outgoing lcm messages.
Incoming lcm messages will trigger functions which call the functions on the appropriate virtual Controllers.
Outgoing lcm messages are triggered by a clock, which query the functions on the appropriate virtual Controllers for data.

backend.h is responsible for translating communications by virtual Controllers into i2c transactions understood by the physical transactions.


### Usage

To build nucleo_bridge use ```./jarvis build onboard/nucleo_bridge/ ``` from the mrover-workspace directory.

To run nucleo_bridge use ```./jarvis exec onboard/nucleo_bridge/ ```
from the mrover-workspace directory.

The CLI will only show errors, since printing output to the console is so time expensive. A blank CLI is a good thing.

To control the RA/SA through open-loop
Ensure onboard/teleop is running on the same platform
Ensure base_station/gui is running on the base station
Operate the joysticks with the RA/SA Task page open on the base station GUI

To control the RA/SA through closed-loop
Ensure onboard/teleop is running on the same platform
Ensure onboard/kinematics is running on the same platform
Ensure base_station/kineval is running on the base station
Input commands into the arm control page on the base station GUI

### Common Errors

This routine typically only throws errors when it has issues communicating with the Nucleo motor controllers. They will have the form "command failed on channel"

The Nucleo that is unresponsive is the first digit of the channel minus 1. For example, if the error message says "activate failed on 10", check Nucleo 0. The firmware may be corrupted and a reset may be necessary, or the wiring has failed and needs to be fixed. If a reset does not fix nucleo_bridge, you may have to restart nucleo_bridge.

### Todo

Re-enable several arm joints.
Hardcode the encoder configurations, PID constants, and other miscellanious variables for each arm joint.
Test predictive torques.


