Code to control the Drive Train Motors using the Odrive Brushless Motor Controller 
----

### About
For the 2019 Mars Rover we used two onboard odrive motor controllers to drive the wheels. Odrive_bridge is the program \
responsible for handling odrive control LCM messages recieved from the basestation. Upon connecting to the basestation \
the program begins running automatically. 

The program can handle velocity and state LCM commands. The states of the odrive are **DisconnectedState**,\
**DisarmedState**, **ArmedState**, **CalibrateState**, and **ErrorState.** \
From the basestation commands to Arm, Disarm, and Calibrate are possible, as well as requests to view encoder and current
draw data, and velocity commands. 

When the odrive is Armed it controlls the speed of the motors in the closed-loop velocity control mode, getting data about 
the motors' current speed via the encoders integrated into the controller. 

Within in the program most commands that can be found on the [odrive website](https://odriverobotics.com/) have been abstracted 
by the Modrive class. Each state is its own class, and is instantiated and used by the OdriveBridge state machine. 
The main function then updates the state machine. 

The program uses multi-threading and locks so that it can handle encoder/current draw data commands as well as speed/state
commands at the same time. Threading locks are used within the code to prevent the threads from running parts of the code 
simultaneously. 

### States

**DisconnectedState** - In this state the program is searching for the odrive via its ID number. Once it has found 
the odrive the state will immediately change to Armed. 

**ArmedState** - In this state the odrive will respond to velocity commands until instructed otherwise or errors out. 

**DisarmedState** - In this state the odrive is on, however will not respond to any velocity commands until instructed 
otherwise or errors out. 

**CalibrateState** - In this state the odrive will be no responsive as it calibrates the motors connected to it. It goes immediately to ArmedState upon completion of calibration. 

**ErrorState** - In this state the odrive has detected a system error and will reset, going from Disconnected to Armed immediately.


### Usage 
This code is meant to be run on the rover. For external testing follow the wiring up guidelines specified [here](https://docs.odriverobotics.com/#wiring-up-the-odrive)
on the odrive webite. \
The program automatically starts up upon connecting to the rover. \
In order to drive the rover, use the joystick, making sure its connected to the base station, you have communications, and no odrive errors are showing up. \
In order to get drive data make sure that you are in the mrover-workspace folder on the base station \
`$ cd ~/mrover-workspace` \
To get the current state of the odrive type \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_echo DriveStateData "./drive_state_data"` \
To get velocity estimates and current draw data type \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_echo DriveVelData "./drive_vel_data"` 

### Setting Up A New Odrive 

#### Electrical Modifications
<font size="4"> Typically the odrive does not have very good Hall Filtering capabilities, so some electrical modifications must be made prior to 
our usage of it. Six 47nF capacitors will have to be soldered on to the odrive as shown 
[here](https://discourse.odriverobotics.com/t/encoder-error-error-illegal-hall-state/1047/7?u=madcowswe). </font>

#### Getting The ID 
<font size="4"> Each odrive has a unique serial ID. In order to determine that this ID is, either you can follow the steps on 
[this](https://docs.odriverobotics.com/#downloading-and-installing-tools)  
odrive webiste page to get odrivetool on your own computer or connect the 
odrive to the jetson and follow the steps below to get it. You must change the USB permissions (how to in the next section)
before doing this though, and make sure the odrive_bridge program on the jetson is deactivated. \
`$ cd ~/.mrover` \
`$ source bin/activate` \
`$ odrivetool` \
This will start up odrivetool, and after a few seconds *Connected to [ID number] as odrvX* should appear on the screen. \
Type \
`$ quit()` \
`$ deactivate` \
to get out of this state. \
In the odrive_bridge program, go to the connect function in the OdriveBridge class, and look at the line that sets the IDs. 
Depending on which odrive you replaced, change its ID to the new one. The first ID on the 2020 rover controls the back motors
and the seconds controls the front. Rebuild the program. \
`$ ./jarvis build onboard/odrive_bridge`
</font>

#### Changing The USB Permissions
<font size="4"> USB permissions when connecting the odrive to the jetson have to be modified before you can successfully communicate with the odrive. This only has to be done once. \
Make sure the odrive is connected via USB and type \
`$ lsusb` . From list find the idVendor, idProduct, and MODE of the odrive. It will be listed under the info for the InterBiometrics
device. Type \
`$ sudo vi /etc/udev/rules.d/50-myusb.rules` \
`$ SUBSYSTEMS=="usb", ATTRS{idVendor}=="[__idVendor__]", ATTRS{idProduct}=="[__idProduct__]", GROUP="mrover", MODE="[__MODE__]" ` \
 Restart the jetson.
</font>

#### Calibrating The Odrive 
<font size="4"> Since the CalibrateState has not been tested, it must be done manually. Once again go to the .mrover folder 
and start running odrivetool. \
`$ cd ~/.mrover` \
`$ source bin/activate` \
`$ odrivetool` \
The odrives should automatically connect. Using the odrvX.axisY (0 or 1 depending on the odrive with the id) in place of m_axis, execute all of the following commands for axis0 and axis1. \
`$ m_axis.motor.config.pole_pairs = 15 `\
`$ m_axis.motor.config.resistance_calib_max_voltage = 4 ` \
`$ m_axis.motor.config.requested_current_range = 25 `\
`$ m_axis.motor.config.current_control_bandwidth = 100 `\
`$ m_axis.encoder.config.mode = ENCODER_MODE_HALL ` \
`$ m_axis.encoder.config.cpr = 90 ` \
`$ m_axis.encoder.config.bandwidth = 100`\
`$ m_axis.controller.config.pos_gain = 1 `\
`$ m_axis.controller.config.vel_gain = 0.02 `\
`$ m_axis.controller.config.vel_integrator_gain = 0.1 `\
`$ m_axis.controller.config.vel_limit = 1000 `\
`$ m_axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL `\
`$ odrvX.reboot()` \
A ChannelBrokenExceptionError will be thrown however in a few minutes the odrive will reconnect. Upon reconnection type \ 
`$ odrvX.axis0.requestedstate = AXIS_STATE_FULL_CALIBRATION_SEQUENCE ` \
 The motor should beep and start calibrating now. If it does not go to the **Errors** section below. 
 Once it has finished, type \
`$ odrvX.axis0.motor.config.pre_calibrated = True ` \
`$ odrvX.axis0.encoder.config.pre_calibrated = True ` \
 Repeat these three commands for axis1 as well. Then type \
`$ odrvX.reboot()` 

Go to the ~/mrover-workspace folder and re-enable the odrive_bridge program. \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl start service.mrover-onboard-odrive_bridge@{FRONT|BACK}_MOTOR `  (i think) 
</font>

### Common Errors 

#### ODrive is not responding to calibration 
<font size="4"> At this point you should be in odrivetool, if not, follow steps above to get there. Type \
  `$ dump_errors(odrvX, True)` \
  `$ dump_errors(odrvX, True)` \
  If an `ENCODER_HALL_ERROR` shows up only the first time, you are good to try calibration again. If no errors show up at all,
  or if the error persists, re-check your wiring. </font>

#### USB Forwarding on VM 
<font size="4">  Make sure the odrive is connected via USB and type \
`$ lsusb` . From list find the idVendor, idProduct, and MODE of the odrive. It will be listed under the info for the InterBiometrics
device. Type \
`$ sudo vi /etc/udev/rules.d/50-myusb.rules` \
`$ SUBSYSTEMS=="usb", ATTRS{idVendor}=="[__idVendor__]", ATTRS{idProduct}=="[__idProduct__]", GROUP="vagrant", MODE="[__MODE__]" ` \
 Restart the VM. </font>
 
#### ENCODER_HALL_ERROR
<font size="4"> Type \
  `$ dump_errors(odrvX, True)` \
  If the error persists, electrical fucked up. Tell them to recheck the wires\connectors. </font>

#### Suddenly No Resposne 
<font size="4">  In this case, stop and restart the odrive program. The problem is still being investigated \
  `$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl stop service.mrover-onboard-odrive_bridge@{FRONT|BACK}_MOTOR `  \
  `$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl start service.mrover-onboard-odrive_bridge@{FRONT|BACK}_MOTOR `  </font>
  
#### Unknown ACK and Failure 
<font size="4">  In this case, stop and restart the odrive program. The problem is also still being investigated \
  `$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl stop service.mrover-onboard-odrive_bridge@{FRONT|BACK}_MOTOR `  \
  `$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl start service.mrover-onboard-odrive_bridge@{FRONT|BACK}_MOTOR `  </font>
  
#### No LCM Tools Echo/Send 
<font size="4"> Type `$ ./jarvis build lcm_tools/{echo|send}` </font>

#### No Odrive Module
<font size="4"> Make sure you are connected to wifi. \
`$ cd ~/.mrover` \
`$ source bin/activate` \
`$ pip3 install odrive`  </font>

#### Nothing Seems To Be Running
<font size="4"> There have been recent issues with the teleop services not starting. Type \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl status service.mrover-onboard-teleop` \
If the service is not running, type \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl start service.mrover-onboard-teleop`

#### Other Errors
<font size="4"> Find Carmen, Raymond, or Justin. Or just go ahead and contact madcowswe himself! </font>

### Debugging on the Jetson 
Usually being able to arm/disarm manually is needed. To do so stop the program from running on the basestation. \
`$ cd ~/mrover-workspace` \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl stop service.mrover-onboard-odrive_bridge@FRONT_MOTOR ` \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" systemctl stop service.mrover-onboard-odrive_bridge@BACK_MOTOR ` \
To restart the programs manually type \
`$ start0 ` \
`$ start1 ` \
If these are unrecognized, type \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec onboard_odrive_bridge 0 BACK ` \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec onboard_odrive_bridge 1 FRONT ` \
By default the odrives are armed \
In order to see the joystick outputs type \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_echo DriveVelData "/drive_vel_data"` \
In order to Arm/Disarm the odrives manually type \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_send "/drive_state_cmd" "{'type': 'DriveStateCmd', 'controller': X, 'state': Y }" `\
X is the odrive, either 0 or 1, and Y is the state. 1 is DisarmedState, 2 is ArmedState, and 3 is CalibrateState
(not usable at the moment). \
In order to see if there are any odrive errors, \
`$ cd ~/.mrover` \
`$ source bin/activate` \
`$ odrivetool` \
`$ dump_errors(odrv0)` \
`$ dump_errors(odrv1)` 

### ToDo 
As of right now we are unsure whether or not we are using odrives for the 2021 Rover, or what/if testing still needs to be done with the 2020 rover for system validation. 

### Notes
I fucking hate odrives 





