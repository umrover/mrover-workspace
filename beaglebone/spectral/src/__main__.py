from math import ceil
import smbus
import time as t
import numpy as np
import spectral

bus = smbus.SMBus(2)

I2C_MUX_ADDRESS = 0x70
MUX_READ_WRITE_REG = 0xCC  # filler address
DEVICE_SLAVE_ADDRESS = 0x49

def read(channel):
    bus.read_byte_data(I2C_MUX_ADDRESS, MUX_READ_WRITE_REG, channel)

def write(channel):
    bus.write_byte_data(I2C_MUX_ADDRESS, MUX_READ_WRITE_REG, channel)    

ADDRESSES = [ 't': 0x1, '0': 0x2, '1': 0x4, '2': 0x8 ]
def enable():
    for i in addresses:
        write(i[0])
        spectral.enable_spectral()

def main():
    while 1:
        mode = input("select mode: t (triad), 0 (normal 0), 1 (normal 1), 2 (normal 2)")
        write(ADDRESSES[mode][0])
        spectral.get_spectral_data(mode)



if __name__ == "__main__":
    main()

