Code for the science bridge program that will interpret UART messages from a Nucleo
======================================================================================
### About
Writes, reads and parses NMEA like messages from the onboard 
science nucleo to operate the science boxes and get relevant data.

### Hardware
- NVIDIA JetsonNX
- STM32G050C8 Nucleo board
- RX/TX connection cables 

### Spectral
Reads the 6 channel data from 3 different spectral sensors in one NMEA style sentence from an STM32 Nucleo Board over UART. 
#### LCM Channels Publishing/Subscribed To
**Spectral Data [publisher]** \
Messages: [SpectralData.lcm](https://github.com/tabiosg/mrover-workspace/blob/new_science_bridge/rover_msgs/SpectralData.lcm) "/spectral_data" \
Publishers: jetson/science_bridge \
Subscribers: base_station/gui
### UART Message
`$SPECTRAL, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,`

### Triad
Reads the 18 channel data from a spectral triad sensor in one NMEA style sentence from an STM32 Nucleo Board over UART. 
#### LCM Channels Publishing/Subscribed To
**Spectral Triad Data [publisher]** \
Messages: [SpectralData.lcm](https://github.com/tabiosg/mrover-workspace/blob/new_science_bridge/rover_msgs/SpectralData.lcm) "/spectral_triad_data" \
Publishers: jetson/science_bridge \
Subscribers: base_station/gui
### UART Message
`$TRIAD, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,`

### Thermistor
Reads three seperate temperatures from one NMEA-style sentence from the nucleo over UART.
#### LCM Channels Publishing/Subscribed To
**Thermistor Data [Publisher]** \
Messages: [ThermistorData.lcm](https://github.com/tabiosg/mrover-workspace/blob/new_science_bridge/rover_msgs/ThermistorData.lcm) "/thermistor_data" \
Publishers: jetson/science_bridge\
Subscribers: jetson/teleop
#### UART Message
Format of the data string
- `$THERMISTOR,<temp0>,<temp1>,<temp2>`
- String is 50 characters long

### Mosfet
Writes an NMEA like message over UART to the Nucleo in order to turn a specified mosfet device on or off. \
Controls the auton LEDs, raman laser, UV LEDs, UV Bulb, nichrome wires, and raman lasers.
#### LCM Channels Publishing/Subscribed To
**Mosfet Command [subscriber]** \
Messages: [MosfetCmd.lcm](https://github.com/tabiosg/mrover-workspace/blob/new_science_bridge/rover_msgs/MosfetCmd.lcm) "/mosfet_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge

**Nav Status [subscriber]** \
Messages: [NavStatus.lcm](https://github.com/tabiosg/mrover-workspace/blob/new_science_bridge/rover_msgs/NavStatus.lcm) "/nav_status" \
Publishers: jetson/nav \
Subscribers: jetson/science_bridge

#### UART Message
Format of the UART NMEA command
- `$Mosfet,<device>,<enable>,<extra padding>`
- String is 30 characters long
- The angles are in degrees

### Servo
Writes NMEA like messages over UART to the Nucleo in order to move servo to specific angles (in degrees). 
#### LCM Channels Publishing/Subscribed To 
**Servo Command [subscriber]** \
Messsages: [ServoCmd.lcm](https://github.com/tabiosg/mrover-workspace/blob/new_science_bridge/rover_msgs/ServoCmd.lcm) "/servo_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge
#### UART Message
Format of the UART NMEA command
- `$SERVO,<angle0>,<angle1>,<angle2>,<extra padding>`
- String is 30 characters long
- The angles are in degrees

### HBridge Errors (OUTDATED)
There are two pins on the hbridge, LO1 and LO2, that designate error status. \
LO1    |   LO2  \
High   |   High - Normal \
High   |   Low - Motor load open (OPD) \
Low    |   High - Over current (ISD) \
Low    |   Low - Over thermal (TSD)

The nucleo will send a NMEA style message describing the error to the bridge program. \
Using pins PA7 and PC4 on the nucleo - seem to cause no errors/issues.
#### UART Message (OUTDATED)
Format of the error message.
- `$HBRIDGE,<error>`

## TODO
- [ ] Finish this readme
- [ ] Add HBridge code
- [ ] Add limit switch code
- [ ] Pass the linter


