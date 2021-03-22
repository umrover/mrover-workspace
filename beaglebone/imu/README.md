Code for the UM-7 IMU
----

### About
The UM-7 is an integrated IMU and GPS unit. It supports direct register access as well as NMEA protocol through UART. It will run on the jetson or \
a beaglebone. \
[Datasheet](https://www.pololu.com/file/0J1556/UM7%20Datasheet_v1-8_30.07.2018.pdf)

#### LCM Channels Publishing/Subscribed To
**IMU Data** \
Messages: [IMUData.lcm](https://github.com/jjtom34/mrover-workspace/blob/imu/rover_msgs/IMUData.lcm)  "/imu_data" \
Publishers: beaglebone/imu \
Subscribers: jetson/filter

### Usage
The UM-7 is hooked up using UART (RX and TX) wires. Sending an enable command (Via changing transmission rate from 0 to not 0) to the CREG_COM_RATES7 register
 will enable NMEA style messages to be sent over at the specified rate for the specified NMEA string.
 
#### Running
TODO - unknown if running off of beaglebone. If so then use the following \
$ cd ~/mrover-workspace \
$ ./jarvis build beaglebone/imu \
$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec beaglebone_imu
 
#### LCM Commands
To get readings from the sensor: \
In a new terminal \
$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_echo IMUData "/imu_data" \
while the program is running.

#### Calibration
To calibrate you have to use the IMU's propritory software. [Here](https://redshiftlabs.com.au/support-services/serial-interface-software/) is the download link. \
Download and run the software. You'll have to connect the IMU to your computer's usb port. Do so using an FTDI uart to usb converter. \
There will be a drop down menu where you can select the serial port. Go to the magnitometer calibration tab and start the calibration. \
Once it takes the appropriate amount of samples you can stop the calibration. The number will turn green when enough samples have been taken for \
good calibration. Press write to RAM and you'll be all set. \
The other tabs give a couple other options for acclerometer and gyroscopic calibration. Since these require writing to RAM I don't recommend \
doing them every time however if the data from these sensors is starting to drift feel free to recalibrate. 

### To Do
- [x] Use more robust python implementation of the code - similar to new gps and science bridge code.
- [ ] Verify what this imu will run on (Beaglebone? Jetson?)
- [x] Verify if we need to calibrate the imu
- [ ] Add packet error checking in the code
- [ ] Speed up the transmission rate

