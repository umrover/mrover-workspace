# Using the BNO055 IMU

## Background Info
The BNO055 is a 9 axis IMU (inertial measurement unit), meaning it has a 3 axis accelerometer, 3 axis gyroscope, and 3 axis magnetometer. Currently the rover only needs to know acceleration in each axis and bearing (rotation around the Z axis). 

We are using an Arduino to read this data from the sensor and send it over serial to the Jetson (or a test laptop). The IMU driver then reads this data from serial and publishes it to LCM, where it can be accessed by the software that needs to use it. 

BNO055 Sensor -> Arduino -> IMU driver -> LCM -> rover software

[Adafruit BNO055 Guide](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf)

[BNO055 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf6)

## Setting up the Arduino
Since our Arduino program doesn't use much memory or processing power, just about any standard Arduino board should work. We're using an Arduino Nano Every, since it's really small.

### Wiring
to connect the BNO055 to the Arduino, connect the following pins:
| Arduino pin | BNO055 pin |
| ----------- | ---------- |
| 5V          | Vin        |
| GND         | GND        |
| A4          | SDA        |
| A5          | SCL        |

5V and GND power the BNO055, A4 and A5 allow us to connect to it over I2C.

### Uploading the code
In the Arduino IDE (download [here](https://www.arduino.cc/en/software)), open `BNO055_serial.ino` and Plug in the Arduino over USB.

We are using Adafruit's [BNO055 Arduino Library](https://github.com/adafruit/Adafruit_BNO055) to read data from the IMU, so we need to install it in order to compile the code. Go to Tools -> Manage Libraries to open the library manager, search for "Adafruit BNO055", and click install on the library with that name. It will probably ask you if you want to install dependencies for the library, which you should do. once installed, close the library manager.

Because the Arduino Nano Every uses a newer chip than older Arduino boards, we need to install some drivers for it. Go to Tools -> Board -> Boards Manager, search for "Arduino megaAVR Boards" in the board manager window, and click install on the board package with that name. Close the board manager.

Now go to Tools -> Board -> Arduino megaAVR Boards, and select Arduino Nano Every. Select whatever USB port your Arduino is plugged into, and click the upload button. 

You only need to upload once (unless you change the code), since the code will stay on the arduino perpetually and will start automatically whenever you power it on.

## Running the IMU Driver
Open up your `mrover-workspace` in a terminal. All you have to do is build and run the IMU driver with the following commands:

`./jarvis build jetson/imu_channel` \
`./jarivs exec jetson/imu_channel`

If you get an error that looks something like `No such device or address: '/dev/ttyACM0'`, the Arduino is probably connected to a different port than the driver is looking for. To solve this, open up `jetson/imu_channel/src/__main__.py` and change `'/dev/ttyACM0'` on line 9 to whatever port your Arduino is connected to.

If the arduino is connected correctly, IMU data should be printing to the terminal.

 The IMU driver should be sending IMU data over LCM with the `IMUData` struct on the `/imu_data` channel. Check to make sure the data is being sent over LCM by running the following commands:

`./jarvis build lcm_tools/echo` \
`./jarivs exec lcm_tools/echo IMUData /imu_data`

You should see a stream of `IMUData` structs being printed to the terminal, try wiggling the BNO055 and make sure you see some of the IMU data values changing on the terminal.


## Calibrating the BNO055

The BNO055 doesn't have any flash memory, so it cannot save a calibration state while powered off. This means we have to recalibrate the sensor every time it is turned on. While the BNO055 is on, it's internal chip is constantly trying to recalibrate itself. All we have to do to get it calibrated correctly is to move the sensor around in certain patterns until it says it's calibrated.

You can see the calibration status of each onboard sensor via the LCM data. `gyro_x_dps` refers to the gyro calibration status, `gyro_y_dps` is for the accelerometer calibration status, and `gyro_z_dps` is for the magnetometer calibration status. Each of these variables should be an integer from 0 to 3, with 0 meaning the sensor isn't calibrated at all, and 3 meaning it's fully calibrated.

_(This is a sketchy temporary solution that works bc nothing uses the gyro data but it's built into the IMU driver and the LCM struct. In the future I'll add calibration fields to the LCM struct and do it properly)._

Now just detach the BNO055 from its mount and wave it around in the air. Watch [this video](https://www.youtube.com/watch?v=Bw0WuAyGsnY) on what patterns to move it in for optimal results. You'll know you're done when all three calibration status variables read 3.

_(There's also a 4th calibration status called sys, but that one usually sorts itself out. I'll add it to the LCM struct in the future)_

The BNO055 should now be ready to go and outputting good data, so you can run whatever other programs need to use it.

## TODO
- update `IMUData` LCM struct to include all 4 calibration status values and maybe get rid of values we don't need?
- add feature to allow axes to be rearranged easily (to account for different sensor orientation relative to rover)
- try it out on Arduino IDE 2.0 and change tutorial to work with that, or try it on Arduino Pro CMD and write a script that does everything using that