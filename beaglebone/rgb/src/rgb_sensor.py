# RGB sensor
import time
import smbus
bus = smbus.SMBus(2)

# Relevant Addresses for the RGB Sensor
TCS34725_ADDRESS = 0x29  # Address of RGB Sensor
TCS34725_ID = 0x44  # 0x44 0x12 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727
TCS34725_COMMAND_BIT = 0x80

TCS34725_ENABLE = 0x00
TCS34725_ENABLE_AIEN = 0x10  # RGBC Interrupt Enable
TCS34725_ENABLE_WEN = 0x08  # Wait enable-Writing 1 activates the wait timer
# RGBC Enable - Writing 1 actives the ADC, 0 disables it
TCS34725_ENABLE_AEN = 0x02
# Power on - Writing 1 activates the internal oscillator, 0 disables it
TCS34725_ENABLE_PON = 0x01
TCS34725_ATIME = 0x01  # Integration time
TCS34725_WTIME = 0x03  # Wait time (if TCS34725_ENABLE_WEN is asserted)
TCS34725_WTIME_2_4MS = 0xFF  # WLONG0 = 2.4ms   WLONG1 = 0.029s
TCS34725_WTIME_204MS = 0xAB  # WLONG0 = 204ms   WLONG1 = 2.45s
TCS34725_WTIME_614MS = 0x00  # WLONG0 = 614ms   WLONG1 = 7.4s
TCS34725_AILTL = 0x04  # Clear channel lower interrupt threshold
TCS34725_AILTH = 0x05
TCS34725_AIHTL = 0x06  # Clear channel upper interrupt threshold
TCS34725_AIHTH = 0x07
TCS34725_PERS = 0x0C
TCS34725_PERS_NONE = 0b0000  # Every RGBC cycle generates an interrupt
TCS34725_PERS_1_CYCLE = 0b0001
TCS34725_PERS_2_CYCLE = 0b0010
TCS34725_PERS_3_CYCLE = 0b0011
TCS34725_PERS_5_CYCLE = 0b0100
TCS34725_PERS_10_CYCLE = 0b0101
TCS34725_PERS_15_CYCLE = 0b0110
TCS34725_PERS_20_CYCLE = 0b0111
TCS34725_PERS_25_CYCLE = 0b1000
TCS34725_PERS_30_CYCLE = 0b1001
TCS34725_PERS_35_CYCLE = 0b1010
TCS34725_PERS_40_CYCLE = 0b1011
TCS34725_PERS_45_CYCLE = 0b1100
TCS34725_PERS_50_CYCLE = 0b1101
TCS34725_PERS_55_CYCLE = 0b1110
TCS34725_PERS_60_CYCLE = 0b1111
TCS34725_CONFIG = 0x0D
TCS34725_CONFIG_WLONG = 0x02
TCS34725_CONTROL = 0x0F  # Set the gain level for the sensor
TCS34725_STATUS = 0x13
TCS34725_STATUS_AINT = 0x10  # RGBC Clean channel interrupt
TCS34725_STATUS_AVALID = 0x01

TCS34725_CDATAL = 0x14  # Clear channel data
TCS34725_CDATAH = 0x15
TCS34725_RDATAL = 0x16  # Red channel data
TCS34725_RDATAH = 0x17
TCS34725_GDATAL = 0x18  # Green channel data
TCS34725_GDATAH = 0x19
TCS34725_BDATAL = 0x1A  # Blue channel data
TCS34725_BDATAH = 0x1B

TCS34725_INTEGRATIONTIME_2_4MS = 0xFF  # 2.4ms - 1 cycle    - Max Count: 1024
TCS34725_INTEGRATIONTIME_24MS = 0xF6  # 24ms  - 10 cycles  - Max Count: 10240
TCS34725_INTEGRATIONTIME_50MS = 0xEB  # 50ms  - 20 cycles  - Max Count: 20480
TCS34725_INTEGRATIONTIME_101MS = 0xD5  # 101ms - 42 cycles  - Max Count: 43008
TCS34725_INTEGRATIONTIME_154MS = 0xC0  # 154ms - 64 cycles  - Max Count: 65535
TCS34725_INTEGRATIONTIME_700MS = 0x00  # 700ms - 256 cycles - Max Count: 65535

TCS34725_GAIN_1X = 0x00  # No gain
TCS34725_GAIN_4X = 0x01  # 2x gain
TCS34725_GAIN_16X = 0x02  # 16x gain
TCS34725_GAIN_60X = 0x03  # 60x gain


def readU8(reg):
    return bus.read_byte_data(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg)


def readU16Rev(reg):
    return bus.read_word_data(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg)


def write8(reg, value):
    return bus.write_byte_data(TCS34725_ADDRESS, TCS34725_COMMAND_BIT
                               | reg, value & 0xff)


def enable():
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON)
    time.sleep(0.01)
    write8(TCS34725_ENABLE, (TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN))

# returns red, blue, and green values divided by clear


def getData():
    red = readU16Rev(TCS34725_RDATAL) * 1.00
    blue = readU16Rev(TCS34725_BDATAL) * 1.00
    green = readU16Rev(TCS34725_GDATAL) * 1.00
    clear = readU16Rev(TCS34725_CDATAL) * 1.00

    while(clear == 0):
        clear = readU16Rev(TCS34725_CDATAL) * 1.00

    red = red/clear
    blue = blue/clear
    green = green/clear
    return red, green, blue

# Requires : i is a boolean value (t/f)
# Modifies : Light of current light you are connected to.
#           If i is true, we turn on the light
#           If i is false, the light turns off


def light(i):
    r = readU8(TCS34725_ENABLE)
    if(i):
        r &= ~TCS34725_ENABLE_AIEN
        write8(TCS34725_ENABLE, r)
    else:
        write8(TCS34725_ENABLE, r | TCS34725_ENABLE_AIEN)
