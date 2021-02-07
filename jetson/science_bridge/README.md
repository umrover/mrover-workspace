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

**Spectral Command [subscriber]** \
Messages: [SpectralCmd.lcm](https://github.com/cgiger00/mrover-workspace/blob/spectral/rover_msgs/SpectralCmd.lcm) "/spectral_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge

### Thermistor
Reads three seperate temperatures from one NMEA-style sentence from the nucleo over UART.
#### LCM Channels Publishing/Subscribed To
**Thermistor Data [Publisher]** \
Messages: [ThermistorData.lcm](https://github.com/cgiger00/mrover-workspace/blob/science-nucleo/rover_msgs/ThermistorData.lcm) "/thermistor_data" \
Publishers: jetson/science_bridge\
Subscribers: jetson/teleop

### Text
TODO

### Mosfet
Writes an NMEA like message over UART to the Nucleo in order to turn a specified mosfet device on or off.
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

### Ammonia
TODO

### Pump
TODO
