Code for the UM-7 IMU
----

### About
The UM-7 is an integrated IMU and GPS unit. It supports direct register access as well as NMEA protocol through UART. It will run on a _________ (unclear). Hopefully not a beaglebone as we are trying to move away from those. \
[Datasheet](https://www.pololu.com/file/0J1556/UM7%20Datasheet_v1-8_30.07.2018.pdf)

#### LCM Channels Publishing/Subscribed To
**something auton** \
Messages: [IMUData.lcm](https://github.com/jjtom34/mrover-workspace/blob/imu/rover_msgs/IMUData.lcm)  "/imu_data" \
Publishers: jetson/imu \
Subscribers: jetson/filter

### Usage
The UM-7 is hooked up using UART (RX and TX) wires. Sending an enable command (Via changing transmission rate from 0 to not 0) to the CREG_COM_RATES7 register
 will enable NMEA style messages to be sent over at the specified rate for the specified NMEA string.
 
#### Running
TODO - unknown if running off of beaglebone. If so then use the following \
$ cd ~/mrover-workspace \
$ ./jarvis build beaglebone/imu \
$ ./jarvis exec beaglebone_imu
 
#### LCM Commands
To get readings from the sensor: \
In a new terminal \
$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_echo IMUData "/imu_data" \
while the program is running.

#### Calibration
TODO - Unsure if necessary with this model

### To Do
- [ ] Use more robust python implementation of the code - similar to new gps and science bridge code.
- [ ] Verify what this imu will run on (Beaglebone? Jetson?)
- [ ] Verify if we need to calibrate the imu

