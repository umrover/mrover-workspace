# GUI LCM Documentation 

### Base Station GUI LCMs

#### All GUIs:
ArmControlState
"/arm_control_state"
File: ArmControlState.lcm
Subscriptions: N/A
Publishers: ArmControls.vue
Usage: Selecting open/closed loop

ArmControlState
"/arm_control_state_to_gui"
File: ArmControlState.lcm
Subscriptions: ArmControls.vue, SAArm.vue,
Publishers: N/A
Usage: Determining open/closed loop

Camera (this is no longer used, but subscribed)
"/camera_servos"
File: CameraServos.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: 

Cameras
"/cameras_cmd"
File: Cameras.lcm
Subscriptions: N/A
Publishers: Cameras.vue
Usage: Selecting which camera is displayed on each port

DebugMessage
"/debug_message"
File: DebugMessage.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: Sends an error message

Drive Velocity
"/drive_vel_data"
File: DriveVelData.lcm
Subscriptions: DriveVelDataH.vue, DriveVelDataV.vue
Publishers: N/A
Usage: LCM data displayed in table on GUIs

Drive State
"/drive_state_data"
File: DriveStateData.lcm
Subscriptions: DriveVelDataH.vue, DriveVelDataV.vue
Publishers: N/A
Usage: LCM data displayed in table on GUIs

Encoder
"/encoder"
File: Encoder.lcm
Subscriptions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: 

KillSwitch
"/kill_switch"
File: KillSwitch.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: 

Microscope
"/microscope"
File: Microscope.lcm
Subscriptions: N/A
Publishers: Cameras.vue
Usage: Selecting whether the microscope is displayed

NavStatus
"/nav_status"
File: NavStatus.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: 

Odometry
"/odometry"
File: OdometryReading.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: Displays odometry data on GUIs

Sensors
"/sensors"
File: Sensors.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: 

Temperature
"/temperature"
File: Temperature.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: 

Xbox
"/ra_control"
File: Xbox.lcm
Subscriptions: N/A
Publishers: ArmControls.vue
Usage: Values based on xbox controller input

#### Science GUI:
CarouselData
"/carousel_data"
File: CarouselData.lcm
Subscriptions: Carousel.vue
Publishers: N/A
Usage: Verifies Carousel position 

CarouselClosedLoopCmd
"/carousel_closedloop_cmd"
File: CarouselClosedLoopCmd.lcm
Subscriptions: N/A
Publishers: Carousel.vue
Usage: Sends target carousel position

CarouselOpenLoopCmd
"/carousel_openloop_cmd"
File: CarouselOpenLoopCmd.lcm
Subscriptions: N/A
Publishers: Carousel.vue
Usage: Sends out throttle for Carousel control

FuseData
"/fuse_data"
File: FuseData.lcm
Subscriptions: PDBFuse.vue
Publishers: N/A
Usage: 

Heater
"/heater_state_data"
File: Heater.lcm
Subscriptions: Amino.vue
Publishers: N/A
Usage: Used to determine whether each heater is on or off

Heater
"/heater_cmd"
File: Heater.lcm
Subscriptions: N/A
Publishers: Amino.vue
Usage: Sends a message to change heater on/off

HeaterAutoShutdown
"/heater_auto_shutdown_data"
File: HeaterAutoShutdown.lcm
Subscriptions: Amino.vue
Publishers: N/A
Usage: Used to determine if heater autoshutdown is on/off

HeaterAutoShutdown
"/heater_auto_shutdown_cmd"
File: HeaterAutoShutdown.lcm
Subscriptions: N/A
Publishers: Amino.vue
Usage: Sends bool with newly selected heater autoshutdown on/off value

MosfetCmd
"/mosfet_cmd"
File: MosfetCmd.lcm
Subscriptions: Chrlophyll.vue
Publishers: Chrlophyll.vue, Amino.vue, OdometryReadingSA.vue, Raman.vue, ScoopUV.vue
Usage: Enabling and disabling Raman test, UV lights, Thermistor Heaters

PDBData
"/pdb_data"
File: PDBData.lcm
Subscriptions: PDBFuse.vue
Publishers: N/A
Usage: Values are displayed in PDBFuse table

SAPosition
"/sa_position"
File: SAPosition.lcm
Subscriptions: SAArm.vue
Publishers: N/A
Usage: Sends the current positions of each joint in radians

ServoCmd
"/servo_cmd"
File: ServoCmd.lcm
Subscriptions: N/A
Publishers: StripTest.vue
Usage: Sends target angle for each servo test

SpectralData
"/spectral_data"
File: SpectralData.lcm
Subscriptions: SATask.vue
Publishers: N/A
Usage: Values are passed to Chlorophyll vue component and read out in table on Science GUI

SpectralTriadData
"/spectral_triad_data"
File: SpectralData.lcm
Subscriptions: SATask.vue
Publishers: N/A
Usage: Data from spectral triad end effector displayed in SpectralData vue component

ThermistorData
"/thermistor_data"
ThermistorData.lcm
Subscriptions: Amino.vue, GenerateReport.vue
Publishers: N/A
Usage: Used to read in values for thermistor temperatures

Xbox
"/sa_control"
File: Xbox.lcm
Subscriptions: N/A
Publishers: SAArm.vue
Usage: Values based on xbox controller input

#### Auton GUI:
Autonomous
"/autonomous"
File: Autonomous.lcm
Subscriptions: 
Publishers: N/A
Usage: 
