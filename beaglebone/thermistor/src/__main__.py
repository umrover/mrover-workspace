# This code assumes the use of a voltage divider
# The R1 value needs to be set to whatever resistor is being used
# change the ADC pin to whatever one is being used, I just chose a random ADC pin

import Adafruit_BBIO.ADC as adc
from math import log as ln
from time import sleep
import lcm as lcm_
from rover_msgs import ThermistorData

# time between temperature sends
global LCM_INTERVAL, publishChannel
LCM_INTERVAL = .05
publishChannel = "/thermistor_data"

# setting ADC pin
# FIXME change to desired pin
global adcPin
adcPin = "P9_33"

# voltage divider constants
global R1, V1
R1 = 9760
V1 = 3.3

# constants depending on the thermistor
# see readme for explanation of constants
global constantArray, R25
constantArray = [[3.3570420E-03,2.5214848E-04, 3.3743283E-06, -6.4957311E-08 ],\
                [3.3540170E-03,2.5617244E-04, 2.1400943E-06, -7.2405219E-08 ],\
                [3.3530481E-03, 2.5420230E-04, 1.1431163E-06, -6.9383563E-08],\
                [3.3536166E-03,2.5377200E-04,8.5433271E-07, -8.7912262E-08]]
# Given by the spec of the specific thermistor
R25 = 10000

def main():
    # Start the LCM
    global lcm
    lcm = lcm_.LCM

    # init the ADC
    adc.setup()

    # main while loop
    while True:
        # reading voltage from ADC
        # Documentation says need to read twice due to bug in the library
        # FIXME I believe we want to use read_raw, since read is between 0 and 1
        V2 = adc.read_raw(adcPin)
        V2 = adc.read_raw(adcPin)

        # calc the current flowing through circuit
        I =  (V1 - V2) / R1

        # calculating R2 (resistance of thermistor)
        Rt = V2 / I

        # Determine which set of constants should be used
        if(Rt < 692600 and Rt >= 32770):
            constantSet = 0
        elif(Rt < 32770 and Rt >= 3599):
            constantSet = 1
        elif(Rt < 3599 and Rt >= 681.6):
            constantSet = 2
        elif(Rt < 681.6 and Rt >= 187):
            constantSet = 3
        else:
            # TODO proper error handling (out of bounds temp)
            print("OOB temp")
            exit()


        # Using the Rt/R25 range to determine actual temp
        lnRtoverR25 = ln(Rt/R25)
        oneOverT = constantArray[constantSet][0] + (constantArray[constantSet][1] * lnRtoverR25) \
            + (constantArray[constantSet][2] *  lnRtoverR25 * lnRtoverR25) \
            + (constantArray[constantSet][3] * lnRtoverR25 * lnRtoverR25 * lnRtoverR25)

        # calc actual temp from the IoverT value the formula gives
        currTemp = (1 / oneOverT) + 272.15 # + 273 to convert to C

        # publishing message and waiting desired interval
        publishMessage(currTemp)
        sleep(LCM_INTERVAL)   
        
def publishMessage(currTemp):
    global msg
    msg = ThermistorData()
    msg.temperature = currTemp
    lcm.publish(publishChannel, msg.encode())


main()

