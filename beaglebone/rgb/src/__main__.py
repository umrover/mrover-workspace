# main function for rgb sensor and i2c multiplexer
from . import i2c_multiplexer
from . import rgb_sensor
import Adafruit_BBIO.GPIO as GPIO
import asyncio
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import RGB, RGBLED

lcm_ = aiolcm.AsyncLCM()
leds = "P8_11"


async def publish_rgb_readings():
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

        # Read data from channel 0 (Buret, Site 1)
        mux.tca_select(0x01)
        r, g, b = rgb_sensor.getData()

        rgb.id = "rgb_buret_1"
        rgb.r = r
        rgb.g = g
        rgb.b = b
        lcm_.publish('/rgb', rgb.encode())

        # Read data from channel 2 (Ammonia, Site 2)
        mux.tca_select(0x04)
        r, g, b = rgb_sensor.getData()

        rgb.id = "rgb_ammonia_2"
        rgb.r = r
        rgb.g = g
        rgb.b = b
        lcm_.publish('/rgb', rgb.encode())

        # Read data from channel 3 (Buret, Site 2)
        mux.tca_select(0x08)
        r, g, b = rgb_sensor.getData()

        rgb.id = "rgb_buret_2"
        rgb.r = r
        rgb.g = g
        rgb.b = b
        lcm_.publish('/rgb', rgb.encode())

        await asyncio.sleep(0.5)


def rgb_led_callback(channel, msg):
    cmd = RGBLED.decode(msg)

    if cmd.on:
        GPIO.output(leds, GPIO.HIGH)
    else:
        GPIO.output(leds, GPIO.LOW)


def main():
    GPIO.setup(leds, GPIO.OUT)
    GPIO.output(leds, GPIO.HIGH)

    lcm_.subscribe("/rgb_leds", rgb_led_callback)
    run_coroutines(lcm_.loop(), publish_rgb_readings())


if(__name__ == "__main__"):
    main()
