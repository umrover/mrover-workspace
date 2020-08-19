import asyncio
import time
import Adafruit_BBIO.PWM as PWM
import lcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import ServoCMD


# Don't know if this is the same for the servo we're using
SERVO_MAX_DC = 10.0
SERVO_MIN_DC = 4.0



lcm_ = lcm.LCM()


def angle_to_dc(degrees):
    percent = degrees / 120.0
    dc = SERVO_MIN_DC + (percent * (SERVO_MAX_DC - SERVO_MIN_DC))
    return dc


def run_servo(pin, degrees):
    dc = angle_to_dc(degrees)
    PWM.set_duty_cycle(pin, dc)


def servo_init(pin, degrees):
    dc = angle_to_dc(degrees)
    PWM.start(pin, dc, 50)


def servocmd_callback(channel, msg):
    servo = ServoCMD.decode(msg)
    if (servo.id == "left"):
        run_servo(leftpin, servo.position)
        time.sleep(4)
    else if (servo.id == "right"):
        run_servo(rightpin, servo.position)
        time.sleep(4)


def main():

    # Change pins to actual pin on beaglebone once we know what pin
    servo_init(leftpin, 0)
    servo_init(rightpin, 0)
    # Might need a manual set to 0, not sure if init sets to angle 0 initially

    lcm_.subscribe("/servocmd", servocmd_callback)








