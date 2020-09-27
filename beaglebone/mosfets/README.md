Code for the Mosfet
===================
### About
This code controls the mosfet pinouts and sets pins high or low based on the information in the lcm struct.\
Controls the following devices as part of the science sensor suite.\
Three 12V heaters\
UV LED & Repeater 5V\
LED 24V\
RA Laser 3.3V

#### LCM Channels
MosfetCmd [subscriber]\
Messages: [MosfetCmd.lcm](https://github.com/Polishdudealan/mrover-workspace/blob/mosfet/rover_msgs/MosfetCmd.lcm) "/mosfet_cmd"\
Publishers: base_station/gui\
Subscriber: beaglebone/mosfet_cmd

### Usage
Devices needed: \
Mosfet Board (Custom made by EHW)\
Beaglebone Black

#### Building
SSH into the Beaglebone and open up the terminal. Type\
```$ cd ~/mrover-workspace/``` to move to the mrover-workspace directory\
```$ ./jarvis build beaglebone/mosfets``` to build the gps program\
```$ ./jarvis exec beaglebone_mosfets``` to run the gps program


