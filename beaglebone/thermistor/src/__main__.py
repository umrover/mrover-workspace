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
# FIXME channel name may be wrong
publishChannel = "/thermistor_temp"

# setting ADC pin
# FIXME change to desired pin
global adcPin
adcPin = "P9_33"

# voltage divider constants
global R1, V1
R1 = 10000
V1 = 3.3

# constants depending on the thermistor
# FIXME unsure if these are correct, 
# given by a random 90 page pdf unsure if picked right ones
global A,B,C,D,R25
A = 0.003357042
B = 0.00025143
C = 3.37742e-06
D = -6.54336e-08
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

        # Using the Rt/R25 range to determine actual temp
        lnRtoverR25 = ln(Rt/R25)
        IoverT = A + (B * lnRtoverR25) + (C *  lnRtoverR25 * lnRtoverR25) \
            + (D * lnRtoverR25 * lnRtoverR25 * lnRtoverR25)

        # calc actual temp from the IoverT value the formula gives
        currTemp = 1 / (IoverT / I)

        # publishing message and waiting desired interval
        publishMessage(currTemp)
        sleep(LCM_INTERVAL)   
        
def publishMessage(currTemp):
    global msg
    msg = ThermistorData()
    msg.temperature = currTemp
    lcm.publish(publishChannel, msg.encode())


main()

