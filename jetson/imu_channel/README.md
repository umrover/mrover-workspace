# Using the BNO055 IMU

## Background Info
The BNO055 is a 9 axis IMU (inertial measurement unit), meaning it has a 3 axis accelerometer, 3 axis gyroscope, and 3 axis magnetometer. Currently the rover only needs to know acceleration in each axis and bearing (rotation around the Z axis). 

We are using an Arduino to read this data from the sensor and send it over serial to the Jetson (or a test laptop). The IMU driver then reads this data from serial and publishes it to LCM, where it can be accessed by the software that needs to use it. 

BNO055 Sensor -> Arduino -> IMU driver -> LCM -> rover software

[Adafruit BNO055 Guide](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf)

[BNO055 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)

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

You can see the calibration status of each onboard sensor via the LCM data. `calibration_gyro` refers to the gyro calibration status, `calibration_accel` is for the accelerometer calibration status, and `calibration_mag` is for the magnetometer calibration status. Additionally, there is `calibration_sys`, which represents the calibration of the whole system (read about it on page 67 of the datasheet). Each of these variables should be an integer from 0 to 3, with 0 meaning the sensor isn't calibrated at all, and 3 meaning it's fully calibrated.

Now just detach the BNO055 from its mount and wave it around in the air. Watch [this video](https://www.youtube.com/watch?v=Bw0WuAyGsnY) on what patterns to move it in for optimal results. You'll know you're done when all four calibration status variables read 3. 

Be aware that the accelerometer can be very tricky to calibrate, sometimes taking upwards of 10 minutes to get the calibration status to 3. It's also worth noting that the magnetometer calibration status often drops to a 2 or a 1 after a while.

The BNO055 should now be ready to go and outputting good data, so you can run whatever other programs need to use it.

## Arduino Pro CLI

Instead of using the Arduino IDE to upload to the Arduino, you can use the Arduino Pro CLI tool. This is better since it can be done over SSH purely from the command line.

### Installation
Download the `arduino-cli` binaries from [here](https://arduino.github.io/arduino-cli/0.21/installation/), then put them in `/bin/`

Install the BNO055 library: \
`arduino-cli lib install "Adafruit BNO055"`

Install `screen` with `sudo apt install screen`

### Compile/Upload
First install the necessary core for your board: \
`arduino-cli core install arduino:megaavr` 

Then find your board: \
`arduino-cli board list`

Then compile, specifying the board type and the sketch path: \
`arduino-cli compile -b arduino:megaavr:nona4809 Desktop/my_sketch.ino`

Then upload, specifying the port, board type, and sketch: \
`arduino-cli upload -p /dev/ttyACM0 -b arduino:megaavr:nona4809 \Desktop/my_sketch.ino`

(the Arduino Nano Every uses board type `arduino:megaavr:nona4809`)

### Viewing Serial Output
Use GNU screen tool, specifying port and baud rate: \
`screen /dev/ttyACM0 9600`

Exit screen by pressing _ctrl+a_ then _k_


## TODO
- add feature to allow axes to be rearranged easily (to account for different sensor orientation relative to rover)
- try it out on Arduino IDE 2.0 and change tutorial to work with that
- write a script that uses arduino cli to do everything automatically