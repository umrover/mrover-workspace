import smbus
import numpy as np
import time as t
from rover_msgs import SpectralData
#will add another soon 
import lcm

# 0x00: Select master data -- AS72651
# 0x01: Select first slave data -- AS72652
# 0x02: Select second slave data -- AS72653
DEV_SEL  = 0x4F

bus = smbus.SMBus(2)
lcm_ = lcm.LCM()

I2C_AS72XX_SLAVE_STATUS_REG = 0x00
I2C_AS72XX_SLAVE_WRITE_REG = 0x01
I2C_AS72XX_SLAVE_READ_REG = 0x02
I2C_AS72XX_SLAVE_TX_VALID = 0x02
I2C_AS72XX_SLAVE_RX_VALID = 0x01

DEVICE_SLAVE_ADDRESS_READ = 0x93
DEVICE_SLAVE_ADDRESS_WRITE = 0x92
DEVICE_SLAVE_ADDRESS = 0x49

# registers for triad 
RAW_VALUE_RGA_HIGH = 0x08
RAW_VALUE_RGA_LOW = 0x09

RAW_VALUE_SHB_HIGH = 0x0A
RAW_VALUE_SHB_LOW = 0x0B

RAW_VALUE_TIC_HIGH = 0x0C
RAW_VALUE_TIC_LOW = 0x0D

RAW_VALUE_UJD_HIGH = 0x0E
RAW_VALUE_UJD_LOW = 0x0F

RAW_VALUE_VKE_HIGH = 0x10
RAW_VALUE_VKE_LOW = 0x11

RAW_VALUE_WLF_HIGH = 0x12
RAW_VALUE_WLF_LOW = 0x13

# non-triad registers
R_HIGH = 0x08  # Channel R High Data Byte
R_LOW = 0x09  # Channel R Low Data Byte
S_HIGH = 0x0A  # Channel S High Data Byte
S_LOW = 0x0B  # Channel S Low Data Byte
T_HIGH = 0x0C  # Channel T High Data Byte
T_LOW = 0x0D  # Channel T Low Data Byte
U_HIGH = 0x0E  # Channel U High Data Byte
U_LOW = 0x0F  # Channel U Low Data Byte
V_HIGH = 0x10  # Channel V High Data Byte
V_LOW = 0x11  # Channel V Low Data Byte
W_HIGH = 0x12  # Channel W High Data Byte
W_Low = 0x13  # Channel W Low Data Byte

#combines MSB and LSB data
def get_decimal(virtual_reg_L, virtual_reg_H):  
    high = virtual_read(virtual_reg_H) << 8
    return np.int16((high | virtual_read(virtual_reg_L))) 

def virtual_read(virtual_reg):
    status = None 
    d = None

    status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, 
                I2C_AS72XX_SLAVE_STATUS_REG)

    if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0):
        d = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG)

    while(1):
        status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, 
                I2C_AS72XX_SLAVE_STATUS_REG)

        if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0):
            #no inbound TX pending at slave 
            break
        t.sleep(0.005)
    
    bus.write_byte_data(DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, virtual_reg)

    while(1):
        status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, 
                I2C_AS72XX_SLAVE_STATUS_REG)
        
        if((status & I2C_AS72XX_SLAVE_RX_VALID) != 0):
            #read data is ready
            break
        t.sleep(0.005)
    
    d = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, 
                    I2C_AS72XX_SLAVE_READ_REG)
    return d


def virtual_write(virtual_reg, data):
    print ("data sent:", data)
    status = None
    while(1):
        status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, 
                    I2C_AS72XX_SLAVE_STATUS_REG)
        if((status & I2C_AS72XX_SLAVE_TX_VALID) == 0):
            break
        t.sleep(0.005)

    bus.write_byte_data(DEVICE_SLAVE_ADDRESS, 
                    I2C_AS72XX_SLAVE_WRITE_REG, (virtual_reg | 1<<7))
    while(1):
        status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, 
                    I2C_AS72XX_SLAVE_STATUS_REG)
        if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0):
            break
        t.sleep(0.005) #amount of ms to wait between checking virtual register changes

    bus.write_byte_data(DEVICE_SLAVE_ADDRESS, 
                I2C_AS72XX_SLAVE_WRITE_REG, data)
    

def get_triad_data(msg):
    colors = SpectralData.decode(msg)
    dev_channels = {
                    0x00: {"R": colors.r, "S": colors.s, "T": colors.t, "U": colors.u, "V": colors.v, "W": colors.w },
                    0x01: {"G": colors.g, "H": colors.h, "I": colors.i, "J": colors.j, "K": colors.k, "L": colors.l },
                    0x02: {"A": colors.a, "B": colors.b, "C": colors.c, "D": colors.d, "E": colors.e, "F": colors.f }
                    }
    channel_registers = [[RAW_VALUE_RGA_LOW, RAW_VALUE_RGA_HIGH], [RAW_VALUE_SHB_LOW, RAW_VALUE_SHB_HIGH],
                     [RAW_VALUE_TIC_LOW, RAW_VALUE_TIC_HIGH], [RAW_VALUE_UJD_LOW, RAW_VALUE_UJD_HIGH],
                     [RAW_VALUE_VKE_LOW, RAW_VALUE_VKE_HIGH], [RAW_VALUE_WLF_LOW, RAW_VALUE_WLF_HIGH]]

    for i in dev_channels:
        virtual_write(DEV_SEL, i)
        print("device number:", virtual_read(DEV_SEL), i)
        channel = 0
        for j in dev_channels[i]:
            dev_channels[i][j] = get_decimal(channel_registers[channel][0], channel_registers[channel][1])
            channel += 1
    return dev_channels

def get_data():
    # colors = SpectralData.decode(msg)
    # dev_channels = {"R": colors.r, "S": colors.s, "T": colors.t, "U": colors.u, "V": colors.v, "W": colors.w }
    dev_channels = {"R": 0, "S": 0, "T": 0, "U": 0, "V": 0, "W": 0 }

    channel_registers = [ 
                    [ R_LOW, R_HIGH ], [ S_LOW, S_HIGH ],
                    [ T_LOW, T_HIGH ], [ U_LOW, U_HIGH ],
                    [ V_LOW, V_HIGH ], [ W_LOW, W_HIGH ] 
                    ]

    for i in dev_channels[i]:
        dev_channels[i] = get_decimal(channel_registers[channel][0], channel_registers[channel][1])
        channel += 1
    return dev_channels


def spectral_callback(msg):
    if ((virtual_read(0x04) & 0x02) == 2):
        get_data(msg)

def enableLED():
    virtual_write(0x07, virtual_read(0x07) | ~0xF7)

def disableLED():
    virtual_write(0x07, virtual_read(0x07) & 0xF7)

def enable_spectral():
    virtual_write(0x04, 0x28) #runs twice to account for status miss
    virtual_write(0x04, 0x28) #converts data bank to 2
    virtual_write(0x05, 0xFF) #increases integration time

def get_spectral_data(mode):
    if ((virtual_read(0x04) & 0x02) == 2):
        print("DATA READY TO READ")
        if (mode == 't'):
            data = get_spectral_data()
            for i in range(0, 3):
                [print(key, value) for key, value in data[i].items()] 
        else:
            data = get_data()
            [print(key, value) for key, value in data[i].items()]    
    

def main():
    mode = input("select mode: s (spectral) or n (normal)")

    while True:
        lcm_.handle() 
        if ((virtual_read(0x04) & 0x02) == 2):
            print("DATA READY TO READ")
            if (mode == 't'):
                data = get_spectral_data()
                for i in range(0, 3):
                    [print(key, value) for key, value in data[i].items()] 
            else:
                data = get_data()
                [print(key, value) for key, value in data[i].items()]

if __name__ == "__main__":
    main()

        