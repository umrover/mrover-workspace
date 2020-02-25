import smbus
import numpy as np
import time as t
from rover_msgs import SpectralData
from rover_msgs import SpectralCmd
import lcm

# 0x00: Select master data -- AS72651
# 0x01: Select first slave data -- AS72652
# 0x02: Select second slave data -- AS72653
DEV_SEL = 0x4F

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

global SENSOR
SENSOR = 'none'  # triad = 0, single = 1 - 3


# combines MSB and LSB data
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
        status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG)

        if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0):
            # no inbound TX pending at slave
            break
        t.sleep(0.005)

    bus.write_byte_data(DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, virtual_reg)

    while(1):
        status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG)
        if((status & I2C_AS72XX_SLAVE_RX_VALID) != 0):
            # read data is ready
            break
        t.sleep(0.005)

    d = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG)
    return d


def virtual_write(virtual_reg, data):
    status = None
    while(1):
        status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG)
        if((status & I2C_AS72XX_SLAVE_TX_VALID) == 0):
            break
        t.sleep(0.005)

    bus.write_byte_data(DEVICE_SLAVE_ADDRESS,
                        I2C_AS72XX_SLAVE_WRITE_REG, (virtual_reg | 1 << 7))
    while(1):
        status = bus.read_byte_data(DEVICE_SLAVE_ADDRESS,
                                    I2C_AS72XX_SLAVE_STATUS_REG)
        if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0):
            break
        t.sleep(0.005)  # amount of ms to wait between checking virtual register changes
    bus.write_byte_data(DEVICE_SLAVE_ADDRESS,
                        I2C_AS72XX_SLAVE_WRITE_REG, data)


def get_data():
    colors = SpectralData()
    dev_channels = {
                   0x00: {"r": colors.r, "s": colors.s, "t": colors.t, "u": colors.u, "v": colors.v, "w": colors.w},
                   0x01: {"g": colors.g, "h": colors.h, "i": colors.i, "j": colors.j, "k": colors.k, "l": colors.l},
                   0x02: {"a": colors.a, "b": colors.b, "c": colors.c, "d": colors.d, "e": colors.e, "f": colors.f}
                   }

    channel_registers = [
                        [RAW_VALUE_RGA_LOW, RAW_VALUE_RGA_HIGH],
                        [RAW_VALUE_SHB_LOW, RAW_VALUE_SHB_HIGH],
                        [RAW_VALUE_TIC_LOW, RAW_VALUE_TIC_HIGH],
                        [RAW_VALUE_UJD_LOW, RAW_VALUE_UJD_HIGH],
                        [RAW_VALUE_VKE_LOW, RAW_VALUE_VKE_HIGH],
                        [RAW_VALUE_WLF_LOW, RAW_VALUE_WLF_HIGH]
                        ]
    global SENSOR
    for i in dev_channels:
        if (SENSOR == 'traid'):
            virtual_write(DEV_SEL, i)
            print("device number:", virtual_read(DEV_SEL), i)
        channel = 0
        for j in dev_channels[i]:
            setattr(colors, j, get_decimal(channel_registers[channel][0], channel_registers[channel][1]))
            channel += 1

        if (SENSOR != 'triad'):
            break
    return colors


def spectral_cmd_callback(channel, msg):
    sensors = ['triad', 'single_0', 'single_1', 'single_2']
    cmd = SpectralCmd.decode(msg)

    global SENSOR
    SENSOR = sensors[cmd.sensor]


def publish_spectral_data():
    if ((virtual_read(0x04) & 0x02) == 2):
        lcm_.publish('/spectral_data_publish', get_data().encode())


def enableLED():
    virtual_write(0x07, virtual_read(0x07) | ~0xF7)


def disableLED():
    virtual_write(0x07, virtual_read(0x07) & 0xF7)


def enable_spectral():
    virtual_write(0x04, 0x28)  # runs twice to account for status miss
    virtual_write(0x04, 0x28)  # converts data bank to 2
    virtual_write(0x05, 0xFF)  # increases integration time

