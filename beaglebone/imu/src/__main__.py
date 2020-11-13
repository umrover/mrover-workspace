import Adafruit_BBIO.UART as UART
import serial as se

#https://learn.adafruit.com/setting-up-io-python-library-on-beaglebone-black/uart
#UART1 RX:P9_26 TX:P9_24 CTS:P9_20 RTS:P9_19
#UART2 RX:P9_22 TX:P9_21
UART.setup("UART2")


#Binary packets Pg. 31
#https://www.pololu.com/file/0J1556/UM7%20Datasheet_v1-8_30.07.2018.pdf



# Command flag
COMMAND = (1 << 7)

#LightSensor I2C_Address
I2C_ADDRESS = 0x29

#Relevant Registers (From .h sheet)
CD_LS = 0x14  # Clear channel data low byte 
CD_MS = 0x15  # Clear channel data high byte
R_LS = 0x16  # Red channel data low byte 
R_MS = 0x17  # Red channel data high byte 
G_LS = 0x18  # Green channel data low byte 
G_MS = 0x19  # Green channel data high byte 
B_LS = 0x1A  # Blue channel data low byte 
B_MS = 0x1B  # Blue channel data high byte

# https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.cpp
# https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h
# http://wiki.erazor-zone.de/wiki:linux:python:smbus:doc

# todo:
# 1. fix read byte data to write to command address
# 2. Make it read two bytes 

#Need to make this read word data take in the lowest byte, and out put the high byte and low byte together.
#Page 19

def convert_raw_to_rgb(red, green, blue, clear):
    """
    Pass the raw values from the TCS to this function to convert to rgb values.
    Returns tuple of red, green, blue values
    """
    # avoid division by zero when reading black color


    if np.int32(clear) == 0:
        return (0, 0, 0)

    return (
        red / np.int32(clear) * 255.0,
        green / np.int32(clear) * 255.0,
        blue / np.int32(clear) * 255.0,
    )

# https://github.com/adafruit/Adafruit_TCS34725/blob/6dc42834bd071aeb94bdddff7e17cb662de20ad2/Adafruit_TCS34725.h#L53
#Power on the Sensor (take out of sleep mode) (page 8,15)
PON = 0x00
bus.write_byte_data(I2C_ADDRESS, PON | COMMAND, 0x01)

# Activates the GAIN (gain for sensor is set to x1) (10)
AGAIN = 0x0F
bus.write_byte_data(I2C_ADDRESS, AGAIN | COMMAND, 0x00)

#Activates the RGBC (turn on sensor to read color) (15)
AEN = 0x00
bus.write_byte_data(I2C_ADDRESS, AEN | COMMAND, 0x03)

# Set the integration cycle to highest for most accurate reading (page 16)
ATIME = 0x01
bus.write_byte_data(I2C_ADDRESS, ATIME | COMMAND, 0x00)

#Make sure to use AutoIncrement from the Command Register
#From Data Sheet:"With this operation,
#when the lower byte register is read, the upper eight bits are stored into a shadow register, which is read by a
#subsequent read to the upper byte."
AUTOINCREMENT = 0x20 | COMMAND
while True:
    # Read the raw color data
    r = bus.read_word_data(I2C_ADDRESS, R_LS | AUTOINCREMENT)
    g = bus.read_word_data(I2C_ADDRESS, G_LS | AUTOINCREMENT)
    b = bus.read_word_data(I2C_ADDRESS, B_LS | AUTOINCREMENT)
    clear = bus.read_word_data(I2C_ADDRESS, CD_LS | AUTOINCREMENT)

    # Convert the raw data to actual rgb
    r, g, b = convert_raw_to_rgb(r, g, b, clear)

    # Print
    print("Red: {} Green: {} Blue: {}".format(r, g, b))
