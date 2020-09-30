import smbus
import time as t
import numpy as np
from math import ceil
bus = smbus.SMBus(2)

# For beaglebone black
I2C_ADDRESS = 0x53

def enable():
    # sets justified bit to 0, 10-bit resolution, +- 16g, etc.
    bus.write_byte_data(I2C_ADDRESS, 0x31, 0x3)
    # Set everything in register 0x2E to 0
    # Prevents functions from generating interrupts
    bus.write_byte_data(I2C_ADDRESS, 0x2E, 0x0)
    # Set everything in register 0x1D to 10100
    # THRESH_TAP - Scale factor us 62.5 mg/LSB (0xFF = 16g)
    bus.write_byte_data(I2C_ADDRESS, 0x1D, 0x14)

    # Important -  Set register to 100, Turns on measure,
    # turns off sleep, and turns of waking up during sleep.
    bus.write_byte_data(I2C_ADDRESS, 0x2D, 0x08)

def get_decimal(ls, ms):
    # Gets most significant bit (MSG) and shifts by 8
    high = read_data(ms) << 8

    # Combines MSB and LSB and scales based off of values in the
    # Arduino ADXL343 digital accelerometer library
    # multiplied by 9.8 again otherwise get output in terms of g
    return np.int16((high | read_data(ls))) * 0.004 * -9.80665 * 9.80665

# Gets raw byte data from address I2C_ADDRESS
def read_data(num):
    return bus.read_byte_data(I2C_ADDRESS, num)

# Rounds the number to a number of decimal places
def float_round(num, places = 0, direction = ceil):
    return direction(num * (10**places)) / float(10**places)


def main():
    data_file = open("output.txt", "w")

    # Data output loop

    enable() 

    while 1:
        time = t.process_time()

        # Gets x y z data of the accelerometer and prints it
        if (float_round(time, 3) % 0.01 >= 0.0009 and float_round(time, 3) % 0.01 <= 0.001 ):
            x = get_decimal(0x32, 0x33)
            y = get_decimal(0x34, 0x35)
            z = get_decimal(0x36, 0x37)
            output = str(time) + '\t' + str(x) + '\t' + str(y) + '\t' + str(z) + '\n'
            data_file.write(output)

    data_file.close()

if __name__ == "__main__":
    main()