Code for measuring pitch, roll and yaw through the ICM 20948(IMU)
 ---
### About

The IMU is intended to read out accelerometer, gyroscope and magnetometer readings and translate them into pitch, roll, yaw and bearing for use in Auton's systems. First, the calibration script is run while the sensor is stable and not moving.  Data is taken for 10 seconds and averaged out. This is to account for any natural drift. This is followed by the main driver which utilizes the calibrated values in order to produce accurate data by subtracting the calibrated values.

### LCM Channels Publishing/Subscribed To
**IMU Data**[publisher] \
Messages:  [IMUData.lcm](https://github.com/jjtom34/mrover-workspace/blob/master/rover_msgs/IMUData.lcm) "/imu_data" \
Publishers: beaglebone/imu \
Subscribers: onboard/filter, onboard/sensor_logging


### Usage
#### Require Electronic Components:
- 1 ICM 20948 Sensor
- 1 Beaglebone Black board or similar board
- At least 4 male-female jumper wires for connecting the sensor to the Beaglebone Black

#### Wiring
When wiring, only the set of pins related to i2c should be used. The top left corner should indicate the correct side and the pins right below it should be used.
[https://cdn.sparkfun.com/r/600-600/assets/learn_tutorials/8/9/3/15335-GPIO_I2C.jpg](https://cdn.sparkfun.com/r/600-600/assets/learn_tutorials/8/9/3/15335-GPIO_I2C.jpg)
VIN,GND and DA, CL should be wired to your power, ground, SDA, and SCL respectively but AD0 and NC should not be connected to anything.
#### Running
Before running, the calibration script should be run in order to obtain proper calibration values before normal operation. This should be done while stable and level. \
`$ ~/cd mrover-workspace`
`$ /jarvis build beaglebone/imu` \
`$ /jarvis exec beaglebone_imu` \
Current usage is to run the calibration script followed by the driver script. This should only be needed once as the values are saved to a seperate text file. It should be run without Jarvis through once navigating to the appropriate directory 
`$ ~/cd mrover-workspace/beaglebone/imu/src`\
`$ python3 calibration.py` 
  

#### LCM Commands
To get readings from the sensor:
In a new terminal \
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_echo IMUData "/imu_data"` \
  while the program is running.

### To Do

This section should have a list of the remaining items needed to be finished for the code. The todo (and later on completed todo) items should be listed using this markup syntax:

-   [x] Proper readings for the Accelerometer and Gyroscope
    
-   [ ] Proper readings for the magnetometer (Testing Needed)
    
-   [ ] Calibration for all three sensors needs more testing

-   [ ] LCM implementation
- [ ] Test jarvis implementation for calibration script

### Notes

--

  

