# main function for rgb sensor and i2c multiplexer
from . import i2c_multiplexer
from . import rgb_sensor
import time
import lcm
from rover_msgs import RGB

lcm_ = lcm.LCM()


def main():
    mux = i2c_multiplexer.I2C_Multiplexer()
    # Write to all channels to enable
    mux.tca_select(0xff)
    rgb_sensor.enable()

    rgb = RGB()

    while(True):
        # Read data from channel 1 (Ammonia, Site 1)
        mux.tca_select(0x02)
        r, g, b = rgb_sensor.getData()

        rgb.id = "rgb_ammonia_1"
        rgb.r = r
        rgb.g = g
        rgb.b = b
        lcm_.publish('/rgb', rgb.encode())

        # Read data from channel 2 (Buret, Site 1)
        mux.tca_select(0x04)
        r, g, b = rgb_sensor.getData()

        rgb.id = "rgb_buret_1"
        rgb.r = r
        rgb.g = g
        rgb.b = b
        lcm_.publish('/rgb', rgb.encode())

        # Read data from channel 3 (Ammonia, Site 2)
        mux.tca_select(0x08)
        r, g, b = rgb_sensor.getData()

        rgb.id = "rgb_ammonia_2"
        rgb.r = r
        rgb.g = g
        rgb.b = b
        lcm_.publish('/rgb', rgb.encode())

        # Read data from channel 4 (Buret, Site 2)
        mux.tca_select(0x10)
        r, g, b = rgb_sensor.getData()

        rgb.id = "rgb_buret_2"
        rgb.r = r
        rgb.g = g
        rgb.b = b
        lcm_.publish('/rgb', rgb.encode())

        time.sleep(0.5)


if(__name__ == "__main__"):
    main()
