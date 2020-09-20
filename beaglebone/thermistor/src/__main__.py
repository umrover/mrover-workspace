# This code assumes the use of a voltage divider
# The R1 value needs to be set to whatever resistors is being used

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
adcPin = ["P9_38", "P9_39", "P9_40"]

# Global outputting var
# Defaulting to -1 so that the program starts non ouputting
outputting = -1

# voltage divider constants
global R1, V1
# Input the three resistor values here
# 0 = White, 1 = Blue, 2 = Yellow
R1 = [9820, 10020, 9830]
V1 = 3.3

# constants depending on the thermistor
# see readme for explanation of constants
global constantArray, R25
constantArray = [[3.3570420E-03, 2.5214848E-04, 3.3743283E-06, -6.4957311E-08],
                 [3.3540170E-03, 2.5617244E-04, 2.1400943E-06, -7.2405219E-08],
                 [3.3530481E-03, 2.5420230E-04, 1.1431163E-06, -6.9383563E-08],
                 [3.3536166E-03, 2.5377200E-04, 8.5433271E-07, -8.7912262E-08]]
# Given by the spec of the specific thermistors
# Measured Room Temp valus in comment, they might not be exactly 25C
# R25 = [11100, 11400, 11120]
R25 = 10000


def publishMessage(currTemp, num):
    msgSend = ThermistorData()
    msgSend.temperature_C = currTemp
    if num == 0:
        msgSend.thermistor = "white"
    elif num == 1:
        msgSend.thermistor = "blue"
    elif num == 2:
        msgSend.thermistor = "yellow"
    global publishChannel
    lcm.publish(publishChannel, msgSend.encode())


def request_callback(channel, msg):
    global outLock
    with outLock:
        whichTherm = ThermistorRequest.decode(msg)
        global outputting
        if whichTherm.thermistor == "white":
            outputting = 0
        elif whichTherm.thermistor == "blue":
            outputting = 1
        elif whichTherm.thermistor == "yellow":
            outputting = 2


def readVolt():
    V2 = []
    for i in range(3):
        # Documentation says need to read twice due to bug in the library
        temp = adc.read(adcPin[i])
        temp = adc.read(adcPin[i])
        # Adjusting V2 to actual Voltage (Multiply by 1.8V since that is max ADC can handle)
        temp *= 1.8
        V2.append(temp)
    return V2


def calcRTherm(V2):
    Rt = []
    for i in range(3):
        Rt.append(((R1[i]*V1)/(V2[i]))-(R1[i]))
    return Rt


def readTherms():
    # reading voltage from ADC
    V2 = readVolt()

    # calculating R2 (resistance of thermistor)
    Rt = calcRTherm(V2)

    currTemp = []

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
        lnRtoverR25 = ln(Rt[i]/R25)
        oneOverT = constantArray[constantSet][0] + (constantArray[constantSet][1] * lnRtoverR25) \
            + (constantArray[constantSet][2] * lnRtoverR25 * lnRtoverR25) \
            + (constantArray[constantSet][3] * lnRtoverR25 * lnRtoverR25 * lnRtoverR25)

        # calc actual temp from the IoverT value the formula gives
        currTemp.append((1 / oneOverT) - 272.15)  # -272.15 for K to C

    return currTemp


# All this function does is LCM handle on its own thread
def handleLCM():
    while True:
        lcm.handle()


def runTherms():
    # main while loop
    while True:
        currTemp = readTherms()

        # publishing message and waiting desired interval
        global outputting
        if outputting != -1:
            publishMessage(currTemp[outputting], outputting)
        sleep(LCM_INTERVAL)


def main():
    # Start the LCM
    global lcm
    lcm = lcm_.LCM()

    # init the ADC
    adc.setup()

    lcm.subscribe("/thermistor_request", request_callback)

    # Setting up lock for outputting
    global outLock
    outLock = threading.Lock()

    # Creating Thread and running it
    listenThread = threading.Thread(target=handleLCM)
    listenThread.start()

    # Running main data reading loop
    runTherms()


if __name__ == "__main__":
    main()
