import smbus
from . import spectral

bus = smbus.SMBus(2)

I2C_MUX_ADDRESS = 0x70
MUX_READ_WRITE_REG = 0xCC  # filler address
DEVICE_SLAVE_ADDRESS = 0x49
ADDRESSES = {'t': 0x1, 'b': 0x2, 'y': 0x4, 'w': 0x8}
# ADDRESSES = {'triad': 0x1, 'single_0': 0x2}


def read(channel):
    bus.read_byte_data(I2C_MUX_ADDRESS, MUX_READ_WRITE_REG, channel)


def write(channel):
    bus.write_byte_data(I2C_MUX_ADDRESS, MUX_READ_WRITE_REG, channel)


def enable():
    for (key, value) in ADDRESSES.items():
        write(value)
        spectral.enable_spectral()


def main():
    enable()
    spectral.lcm_.subscribe("/spectral_cmd", spectral.spectral_cmd_callback)
    while 1:
        spectral.lcm_.handle()
        write(ADDRESSES[spectral.SENSOR])
        spectral.publish_spectral_data()


if __name__ == "__main__":
    main()
