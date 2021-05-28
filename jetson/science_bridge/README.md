Code for the science bridge program that will interpret UART messages from a Nucleo
======================================================================================
### About
Writes, reads and parses NMEA like messages from the onboard 
science nucleo to operate the science boxes and get relevant data.

### Hardware
- NVIDIA JetsonTX2
- STM32F303RE Nucleo board
- RX/TX connection cables 

### Spectral
Reads the 6 channel data from 3 different spectral sensors in one NMEA style sentence from an STM32 Nucleo Board over UART. 
#### LCM Channels Publishing/Subscribed To
**Spectral Data [publisher]** \
Messages: [SpectralData.lcm](https://github.com/cgiger00/mrover-workspace/blob/spectral/rover_msgs/SpectralData.lcm) "/spectral_data" \
Publishers: jetson/science_bridge \
Subscribers: base_station/gui
### UART Message
`$SPECTRAL, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,`

### Thermistor
Reads three seperate temperatures from one NMEA-style sentence from the nucleo over UART.
#### LCM Channels Publishing/Subscribed To
**Thermistor Data [Publisher]** \
Messages: [ThermistorData.lcm](https://github.com/cgiger00/mrover-workspace/blob/science-nucleo/rover_msgs/ThermistorData.lcm) "/thermistor_data" \
Publishers: jetson/science_bridge\
Subscribers: jetson/teleop
#### UART Message
Format of the data string
- `$THERMISTOR,<temp0>,<temp1>,<temp2>`
- String is 50 characters long

### Mosfet
Writes an NMEA like message over UART to the Nucleo in order to turn a specified mosfet device on or off. \
Controls the UV leds, white leds, peristaltic pumps, heaters, raman laser, repeater drop, and auton leds.
#### LCM Channels Publishing/Subscribed To
**Mosfet Command [subscriber]** \
Messages: [MosfetCmd.lcm](https://github.com/cgiger00/mrover-workspace/blob/science-nucleo/rover_msgs/MosfetCmd.lcm) "/mosfet_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge

**Nav Status [subscriber]** \
Messages: [NavStatus.lcm](https://github.com/cgiger00/mrover-workspace/blob/science-nucleo/rover_msgs/NavStatus.lcm) "/nav_status" \
Publishers: jetson/nav \
Subscribers: jetson/science_bridge

**Repeater Drop Init [subscriber]** \
Messages: [RepeaterDrop.lcm](https://github.com/cgiger00/mrover-workspace/blob/science-nucleo/rover_msgs/RepeaterDrop.lcm) "/rr_drop_init" \
Publishers: jetson/nav \
Subscribers: jetson/science_bridge

**Repeater Drop Complete [publisher]** \
Messages: [RepeaterDrop.lcm](https://github.com/cgiger00/mrover-workspace/blob/science-nucleo/rover_msgs/RepeaterDrop.lcm) "/rr_drop_complete" \
Publishers: jetson/science_bridge\
Subscribers: jetson/nav

#### UART Message
Format of the UART NMEA command
- `$Mosfet,<device>,<enable>,<extra padding>`
- String is 13 characters long

### Ammonia
Writes NMEA like messages over UART to the Nucleo in order to run the ammonia test linear actuator forward/backwards at a given speed. 
#### LCM Channels Publishing/Subscribed To 
Messsages: [AmmoniaCmd.lcm](https://github.com/jnnanni/mrover-workspace/blob/sagui/rover_msgs/AmmoniaCmd.lcm) "/ammonia_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge
#### UART Message
Format of the UART NMEA command
- `$AMMONIA,<speed>,<padding commas>`
- String is 13 characters long
- Speed can range from -1 to 1 & is capped at those two points 

### HBridge Errors
There are two pins on the hbridge, LO1 and LO2, that designate error status. \
LO1    |   LO2  \
High   |   High - Normal \
High   |   Low - Motor load open (OPD) \
Low    |   High - Over current (ISD) \
Low    |   Low - Over thermal (TSD)

The nucleo will send a NMEA style message describing the error to the bridge program.
#### UART Message
Format of the error message.
- `$HBRIDGE,<error>`
## TODO
- [ ] Finish this readme
- [x] Increase mosfet buffer to probably around 20 (whatever the buffer size is on the science nucleo) (this is to handle more precise ammonia motor speeds (i.e -0.25) / more importantly handle double digit mosfet devices
- [x] Fix bug where the mosfet handler is a 13 byte buffer but if it's a double digit device it will send 14 bytes 
- [x] Revert mosfet struct back to what is in the ICD once this ^ bug has been fixed 
- [ ] Pass the linter
- [ ] Receive handler for hbridge errors - WIP

