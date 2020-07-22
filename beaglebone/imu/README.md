Code for measuring pitch, roll and yaw through the ICM 20948(IMU)
 ---
### About

The IMU is intended to read out magnetometer, accelerometer and magnetometer readings and translate them into pitch, yaw and roll for use in Auton's systems. First, the calibration script is run while the system is stable, followed by the main driver which utilizes the calibrated values in order to produce accurate data.

### LCM Channels Publishing/Subscribed To

Currently, no official LCM channel/struct has been created but the device should publish the current roll, pitch and yaw calculated from the accelerometer, gyroscope and magnetometer readings. The raw values given may also be passed along. 

### Usage
#### Wiring
When wiring, only the set of pins related to i2c should be used. The top left corner should indicate the correct side and the pins below should be used.
[https://cdn.sparkfun.com/r/600-600/assets/learn_tutorials/8/9/3/15335-GPIO_I2C.jpg](https://cdn.sparkfun.com/r/600-600/assets/learn_tutorials/8/9/3/15335-GPIO_I2C.jpg)
VIN,GND and DA, CL should be wired to your power, ground, SDA, and SCL respectively but AD0 and NC should not be connected to anything.
#### Running
Before running, the calibration script should be run in order to obtain proper calibration values before normal operation. This should be done while stable and level.

Implementation into Jarvis hasn't been tested but should be similar to other sensors using
`$ /jarvis build`
`$ /jarvis exec`
in the proper directory.

Current usage is to run the calibration script followed by the driver script
  

#### LCM Commands

LCM is currently unimplemented.
  

### Off Nominal Behavior Handling

Repeated attempts at startup should warrant checks to wiring

### To Do

This section should have a list of the remaining items needed to be finished for the code. The todo (and later on completed todo) items should be listed using this markup syntax:

-   [x] Proper readings for the Accelerometer and Gyroscope
    
-   [ ] Proper readings for the magnetometer (Testing Needed)
    
-   [ ] Calibration for all three sensors needs more testing

-   [ ] LCM implementation

### Notes

Due to the timing of when this project was being worked on much of the code is still rough and shallow in depth. The magnetometer was tested very lightly before quarantine but its unknown if the values given were correct. Come September, more of this should be polished, tested and integrated into our systems.

--

  

