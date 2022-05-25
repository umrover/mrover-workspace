## README Guide 
All of our code should have a README.md file associated with it. The README should have all the \
required sections listed in this guide 
in the order and formats shown here. 

### Title 
Should specify what sensor the program is written for \
Ex:\
_Code for the AS7265x Spectral Triad Sensor_


### About
This section should include a high level functional description of the software. 

#### LCM Channels Publishing/Subscribed To 
All the channels that the program is subscribed to and publishes to should be listed here along with a link to the rover common LCM struct \
Ex: 

_**Drive Velocity [subscriber]**_\
_Messages: [DriveVelCmd.lcm](https://github.com/raytitan/mrover-workspace/blob/master/rover_msgs/DriveVelCmd.lcm) “/drive_vel_cmd”_\
_Publishers: onboard/teleop_\
_Subscribers: onboard/odrive_


#### Watchdog Timeout Period (optional)
If the hardware has a watchdog timer, its time out period should be listed here


### Usage 
This section should contain all instructions and relevant commands for usage, including hardware specs\
(hardware required, what microprocessor the hardware should be wired up to, etc) and the configurability interface (if there is one)

#### LCM Commands
Type out all the LCM commands using markup code syntax  \`( $ command )`

#### Off Nominal Behavior Handling 
Explain all error handling that the program does in this section, listing the off nominal behavior as well as what is seen on the \
users end as a result\
Ex:

_**Odrive Errors**_\
_If the odrive throws an error, odrive_bridge will tell it to reboot. The error will be displayed in the terminal and the state \
will be displayed as ErrorState. Depending on which odrive this happens on, either the front or back motors will be unresponsive \
for a minute until the odrive finishes rebooting. The state will automatically change from DisconnectedState to Armed \
upon completion._

#### Debugging (optional)
This section should contain all relevant commands for debugging the code or debugging electrical/mechanical issues. \
For motor controllers, how to disable the device should also be listed here.


### Common Errors (optional)
This section should contain a list of all common problems that can occur when using the code, as well as how to fix them. Each error \
should be its own (h4) header, with a more detailed description below along with the relevant steps needed to fix it. 


### ToDo
This section should have a list of the remaining items needed to be finished for the code. The todo (and later on completed todo) \
items should be listed using this markup syntax:

[x] item 1\
[ ] item 2\
[ ] item 3\
They should be updated periodically during the duration of the project. If the project is more collaborative, the todos should \
include detailed descriptions so that other developers can more easily understand and implement what needs to be done. 


### Notes
Anything else relevant that does not constitute its own section. For example the version of python this code has been tested \
with, or a link to the sensor data sheet if it has one. 


### Other Sections (optional)
This is a very basic list of what needs to go into a README. Obviously projects might have other more specific things that \
should be included in their READMEs, so feel free to add other sections as needed. The odrive_bridge README, for example, \
has a separate _How To Set Up An Odrive_ section because this process is quite long and unique. 

## USB Issues
We access the imu at the port /dev/gps, to see the data coming in stop the gps program/service and type: \
```sudo picocom -b 115200 /dev/gps``` \
Data should be coming through. \
If /dev/gps doesn't exist & the gps is plugged, or we get a new gps, check out [this link](https://github.com/umrover/mrover-workspace/blob/auton-integration/ansible/README.md#usb-dev-rules) for now to update that. 


## ToDo
[] finish this readme (localization)
