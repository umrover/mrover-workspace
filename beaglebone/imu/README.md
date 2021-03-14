Code for the UM-7 IMU
----

### About
The UM-7 is an integrated IMU and GPS unit. It supports direct register access as well as NMEA protocol through UART. It will run on a _________ (unclear). \
[Datasheet](https://www.pololu.com/file/0J1556/UM7%20Datasheet_v1-8_30.07.2018.pdf)

#### LCM Channels Publishing/Subscribed To
**something auton** \
Messages: TODO \
Publishers: TODO \
Subscribers: TODO

### Usage
The UM-7 is hooked up using UART (RX and TX) wires. Sending an enable command (Via changing transmission rate from 0 to not 0) to the CREG_COM_RATES7 register
 will enable NMEA style messages to be sent over at the specified rate for the specified NMEA string.
 
#### LCM Commands
TODO


