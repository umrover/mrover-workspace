Code for the TH10K Thermistor
---

### About
This code assumes the use of a voltage divider connected in series with the thermistor.  At the top of the code there are alot of global variables that can dictate how the code works.  They all have comments that describe what they do and where they were gotten from.

This code assumes the usage of a simple voltage divider to figure out what the resistance of the thermistor is.  The R1 value in the code is the resistor that is in series with the thermistor (this resistor should be AFTER the thermistor).  The V1 value is the source voltage that the whole circuit is hooked up to, this should be 3.3V as the beaglebone's adc pin can only read as high as 1.8V.

The code also supports changing the ADC pin that is used very simply.  There is a variable called "adcPin" at the top which can be changed to the desired pin.  This is a list of three different adc pins, as the program supports 3 different thermistors.

[this](https://www.thorlabs.com/thorproduct.cfm?partnumber=TH10K) is the link to the product page where the link to the data sheet can be found.  Linking directly to the pdf doesn't work for some reason, so you will have to find the data sheet and then all the data about the thermistor and where all the constants come from can be found within there.

#### LCM Channels
Thermistor Data \[publisher\] \
Messages: [ThermistorData.lcm](https://github.com/nkr101/mrover-workspace/blob/thermistor/rover_msgs/ThermistorData.lcm) "/thermistor_data" \
Publishers: beaglebone/thermistor \
Subscribers: base_station/gui

Thermistor Request \[subscriber\] \
Messages: [ThermistorData.lcm](https://github.com/nkr101/mrover-workspace/blob/thermistor/rover_msgs/ThermistorRequest.lcm) "/thermistor_request" \
Publishers: base_station/gui \
Subscribers: beaglebone/thermistor

### Usage

Required Components
- BeagleBone
- Resistor
- Thermistor

Wire up a series circuit using one of the either 5V or 3.3V connections of the BeagleBone with the constant resistor coming direction after the voltage source and the thermistor after that.  Make sure to change variables in the code to match your configuration (voltage and resistance).

Connect the ADC pin of your choice on the beagle bone between the resistor and thermistor.

To execute:
- Navigate to your mrover-workspace
- `$ ./jarvis build beaglebone/thermistor` to build the program
- `$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec beaglebone/thermistor` to run the program (unsure about this one)


#### LCM Commands
Todo

#### Off Nominal Behavior Handling
If a temp out of range is detected, it will print out a message and exit

This is not how it will be working on the rover 

### ToDo
- [x] Make sure using correct A,B,C,D values
- [ ] Correct error handling
- [x] Make sure using correct read function from BBIO library
- [x] Determine what value of resistor to use
- [ ] Update correct Resistor and Thermistor values for yellow thermistor
- [ ] Check if need to get R25 of each therm
- [ ] Test on beaglebone with thermistor
- [ ] Weird divide by zero error when circuit gets unplugged

### Notes
Todo
