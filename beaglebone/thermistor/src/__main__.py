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
R1 = 10000
V1 = 3.3

# constants depending on the thermistor
# see readme for explanation of constants
global constantArray, R25
constantArray = [[0.003357042, 0.000252143, 3.37742e-06, -6.54336e-08],\
                [00.003354016, 0.000256173, 2.13941e-06, -7.25325e-08],\
                [0.003353045, 0.0002542, 1.14261e-06, -6.93803e-08],\
                [0.0033533609, 0.000253768, 8.53411e-07, -8.7962e-08]]
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
        if(Rt/R25 < 69.26 and Rt/R25 >= 3.277):
            constantSet = 0
        elif(Rt/R25 < 3.277 and Rt/R25 >= 0.3599):
            constantSet = 1
        elif(Rt/R25 < 0.3599 and Rt/R25 >= 0.06816):
            constantSet = 2
        elif(Rt/R25 < 0.06816 and Rt/R25 >= 0.0187):
            constantSet = 3
        else:
            # TODO proper error handling (out of bounds temp)
            print("OOB temp")
            exit()


        # Using the Rt/R25 range to determine actual temp
        lnRtoverR25 = ln(Rt/R25)
        IoverT = constantArray[constantSet][0] + (constantArray[constantSet][1] * lnRtoverR25) \
            + (constantArray[constantSet][2] *  lnRtoverR25 * lnRtoverR25) \
            + (constantArray[constantSet][3] * lnRtoverR25 * lnRtoverR25 * lnRtoverR25)

        # calc actual temp from the IoverT value the formula gives
        currTemp = 1 / (IoverT / I) + 272.15 # + 273 to convert to C

        # publishing message and waiting desired interval
        publishMessage(currTemp)
        sleep(LCM_INTERVAL)   
        
def publishMessage(currTemp):
    global msg
    msg = ThermistorData()
    msg.temperature = currTemp
    lcm.publish(publishChannel, msg.encode())


main()

