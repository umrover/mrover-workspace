# TCA9548A Multiplexer
import smbus

# Note, use SMBus 2, not 0 or 1. The default i2c for the beagle bone is 2.
bus = smbus.SMBus(2)

i2c = None

# Relevant Addresses for the Mux
TCA9548A_ADDRESS = 0x70
TCA9548A_SWITCH_CHANNEL = 0x04


class I2C_Multiplexer():
    # Requires : reg is an actual register ie 0x01 (0x00 is not one of these)
    # IF you want to read data, talk to only one sensor at a time -
    # ie 0x01 talks to 0 only, 0x02 talks to 1 only,
    # 0x04 talks to 2 only... 0x40 talks to 8 only
    # IF you want to write to multiple rgb sensors you can
    # "combine" the above ie 0x02 + 0x01 = 0x03 will write to 0 AND 1.
    # 0xff will write to ALL rgb sensors
    # Modifies which channel you are talking to
    def tca_select(self, reg):
        bus.write_byte_data(TCA9548A_ADDRESS, TCA9548A_SWITCH_CHANNEL, reg)
