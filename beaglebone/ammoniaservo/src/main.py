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


def ammoniaservo_callback(channel, msg):
    servo = Servo.decode(msg)
    
    run_servo(pin, 0)
    time.sleep(2)
    run_servo(pin, 90)
    time.sleep(4)


def main():

    # Change pin to actual pin on beaglebone once we know what pin
    servo_init(pin, 0)
    

    lcm_.subscribe("/ammoniaservo", ammoniaservo_callback)








