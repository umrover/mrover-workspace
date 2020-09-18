# This code assumes the use of a voltage divider
# The R1 value needs to be set to whatever resistor is being used
# change the ADC pin to whatever one is being used, I just chose a random ADC pin

import Adafruit_BBIO.ADC as adc
from math import log as ln
from time import sleep
import lcm as lcm_
import threading
from rover_msgs import ThermistorData, ThermistorRequest

# time between temperature sends
global LCM_INTERVAL, publishChannel
LCM_INTERVAL = .05
publishChannel = "/thermistor_data"

# setting ADC pin
# FIXME set pin to the corresponding thermistor
global adcPin
adcPin = ["P9_33", "P9_34", "P9_35"]


# voltage divider constants
global R1, V1
# Input the three resistor values here
R1 = [9760, 9760, 9760]
V1 = 3.3

# constants depending on the thermistor
# see readme for explanation of constants
global constantArray, R25
constantArray = [[3.3570420E-03, 2.5214848E-04, 3.3743283E-06, -6.4957311E-08 ],\
                [3.3540170E-03, 2.5617244E-04, 2.1400943E-06, -7.2405219E-08 ],\
                [3.3530481E-03, 2.5420230E-04, 1.1431163E-06, -6.9383563E-08 ],\
                [3.3536166E-03, 2.5377200E-04, 8.5433271E-07, -8.7912262E-08 ] ]
# Given by the spec of the specific thermistor
R25 = 10000

def publishMessage(currTemp, num):
    msgSend = ThermistorData()
    msgSend.temperature_C = currTemp
    msgSend.thermistorNum = num
    lcm.publish(publishChannel, msgSend.encode())


def request_callback(channel, msg):
    whichTherm = ThermistorRequest.decode(msg)    
    outputting = whichTherm


def readVolt():
    V2 = [0,0,0]
    for i in range(3):
        # Documentation says need to read twice due to bug in the library
        V2[i] = adc.read(adcPin[i])
        V2[i] = adc.read(adcPin[i])
        # Adjusting V2 to actual Voltage (Multiply by 1.8V since that is max ADC can handle)
        V2[i] *= 1.8
    return V2


def calcRTherm(V2):
    Rt = [0,0,0]
    for i in range(3):
        Rt[i] = ((R1[i]*V1)/(V2[i]))-(R1[i])


def readTherms():
    # reading voltage from ADC
    V2 = readVolt()

    # calculating R2 (resistance of thermistor)
    Rt = calcRTherm(V2)

    currTemp = [0,0,0]

    for i in range(3):
        # Determine which set of constants should be used
        if(Rt[i] < 692600 and Rt[i] >= 32770):
            constantSet = 0
        elif(Rt[i] < 32770 and Rt[i] >= 3599):
            constantSet = 1
        elif(Rt[i] < 3599 and Rt[i] >= 681.6):
            constantSet = 2
        elif(Rt[i] < 681.6 and Rt[i] >= 187):
            constantSet = 3
        else:
            # TODO proper error handling (out of bounds temp)
            print("OOB temp")

        # Using the Rt/R25 range to determine actual temp
        lnRtoverR25 = ln(Rt/R25)
        oneOverT = constantArray[constantSet][0] + (constantArray[constantSet][1] * lnRtoverR25) \
            + (constantArray[constantSet][2] *  lnRtoverR25 * lnRtoverR25) \
            + (constantArray[constantSet][3] * lnRtoverR25 * lnRtoverR25 * lnRtoverR25)

        # calc actual temp from the IoverT value the formula gives
        currTemp[i] = (1 / oneOverT)

    return currTemp


def runTherms():

    global outputting
    outputting = 0
    # main while loop
    while True:
        lcm.handle()
        
        currTemp = readTherms()

        # FIXME DELETE FOR DEBUGGNG PURPOSE
        print(currTemp[0])
        print("\n")

        # publishing message and waiting desired interval
        if outputting != 0:
            publishMessage(currTemp[outputting], outputting)
        sleep(LCM_INTERVAL) 


def main():
    # Start the LCM
    global lcm
    lcm = lcm_.LCM

    #receiveMsg = ThermistorRequest()

    # init the ADC
    adc.setup()

    lcm.subscribe("/thermistor_request", request_callback)

    runTherms()

if __name__ == "__main__":
    main()