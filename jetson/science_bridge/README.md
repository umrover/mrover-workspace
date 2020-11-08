Code for the science bridge program that will interpret UART messages from a Nucleo
======================================================================================
### About
Writes, reads and parses NMEA like messages from the onboard 
science nucleo to operate the science boxes and get relevant data.

### Hardware
We will be using a Nucleo STM32 F303RE board.

### Spectral
TODO

### Thermistor
Reads three seperate temperatures from one NMEA-style sentence from the nucleo over UART.

#### LCM Channels Publishing/Subscribed To
**Thermistor Data [Publisher]** \
Messages: [ThermistorData.lcm](https://github.com/cgiger00/mrover-workspace/blob/science-nucleo/rover_msgs/ThermistorData.lcm) "/thermistor_data" \
Publishers: onboard/science_bridge\
Subscribers: onboard/teleop

### Text
TODO

### Mosfet
TODO

### Ammonia
TODO

### Pump
TODO
