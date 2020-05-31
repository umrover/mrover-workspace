Code to control the Drive Train Motors using the Odrive Brushless Motor Controller 

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
by the modrive class. Each state is its own class, and is instantiated and used by the OdriveBridge class. 
The main function then updates the state machine each loop. 

The program uses multi-threading and locks so that it can handle encoder/current draw data commands as well as speed/state
commands at the same time. Treading locks are used within the code to prevent the threads from running parts of the code 
simultaneously. 

### States

**DisconnectedState** - In this state the program is searching for the odrive via its ID number. Once it has found 
the odrive the state will immediately change to Armed. 

**ArmedState** - In this state the odrive will respond to velocity commands until instructed otherwise. 

**DisarmedState** - In this state the odrive is on, however will not respond to any velocity commands until instructed 
otherwise. 

**CalibrateState** - In this state the odrive will be no responsive as it calibrates the motors connected to it. It goes immediately to
ArmedState upon completion of calibration. 

**ErrorState** - In this state the odrive has detected a system error and will reset, going from Disconnected to Armed immediately.


### Usage 
This code is meant to be run on the rover. For external testing follow the wiring up guidelines specified [here](https://docs.odriverobotics.com/#wiring-up-the-odrive)
on the odrive webite. \
The program automatically starts up upon connecting to the rover. \
In order to drive the rover, use the joystick, making sure its connected to the basestation, you have communications, and no odrive errors are showing up. \
In order to get drive data make sure that you are in the mrover-workspace folder on the basestation \
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
[this](https://docs.odriverobotics.com/)  
odrive webiste page (starting at Downloading and Installing Tools) to get odrivetool on your own computer or connect the 
odrive to the jetson and follow the steps below to get it. You must set up the USB connection (how to in the next section)
before doing this though\
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
and the seconds controls the front. 
</font>

#### Setting Up The USB Connection
<font size="4"> USB permissions when connection the odrive to the jetson have to be modified before you can successfully communicate with the
odrive. \
Make sure the odrive is connected via USB and type \
`$ lsusb` . From list find the idVendor, idProduct, and MODE of the odrive. It will be listed under the info for the InterBiometrics
device. Type \
`$ sudo vi /etc/udev/rules.d/50-myusb.rules` \
`$ SUBSYSTEMS=="usb", ATTRS{idVendor}=="[idVendor]", ATTRS{idProduct}=="[idProduct]", GROUP="mrover", MODE="[mode]" `
</font>












