Code for the RL0503-5820-97-MS Thermistor
---

### About
This code assumes the use of a voltage divider connected in series with the thermistor

#### LCM Channels
Messages: [ThermistorData.lcm](https://github.com/nkr101/mrover-workspace/blob/thermistor/rover_msgs/ThermistorData.lcm) "/thermistor_temp" \
Publishers: beaglebone/thermistor \
Subscribers: base_station/gui \

### Usage
At the top of the code there are alot of global variables that can dictate how the code works

They all have comments that describe what they do and where they were gotten from

This code assumes the usage of a simple voltage divider to figure out what the resistance of \ the thermistor is.  The R1 value in the code is the resistor that is in series with the thermistor (this resistor \ should be BEFORE the thermistor).  The V1 value is the source voltage that the whole circuit is hooked \ up to, this will be either 3.3V or 5V depending on what pin on the beaglebone the circuit is connected to.

The code also supports changing the ADC pin that is used very simply.  There is a variable called \ "adcPin" at the top which can be changed to the desired pin.

The constantsArray is given on page 69 of [this](https://www.amphenol-sensors.com/en/component/edocman/292-thermometrics-temperature-resistance-curves-reference-guide/download) pdf.  This pdf contains all the resistance curves for \ all the different thermistors this company offers.  We are using the second table since we want temp \ from resistance.

#### LCM Commands
Todo

### Off Nominal Behavior Handling
If a temp out of range is detected, it will print out a message and exit

### ToDo
- [x] Make sure using correct A,B,C,D values
- [ ] Correct error handling
- [ ] Make sure using correct read function from BBIO library
- [ ] Test on beaglebone with thermistor

### Notes
Todo
