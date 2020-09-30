import smbus

bus = smbus.SMBus(2)

I2C_MUX_ADDRESS = 0x70
MUX_READ_WRITE_REG = 0xCC  # filler address


def read(channel):
    bus.read_byte_data(I2C_MUX_ADDRESS, MUX_READ_WRITE_REG, channel)


def write(channel):
    bus.write_byte_data(I2C_MUX_ADDRESS, MUX_READ_WRITE_REG, channel)


def enable(pin):
        write(pin)  # address on mux
