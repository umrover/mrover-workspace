import asyncio
import time
import Adafruit_BBIO.PWM as PWM
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import Servo, ESCToggle, ESCThrottle

SERVO_MAX_DC = 10.0
SERVO_MIN_DC = 4.0

ESC_MAX_DC = 10.0
ESC_MIN_DC = 5.0

ESC_ON_PERCENT = 100.0
ESC_OFF_PERCENT = 0.0

ESC_RAMP_RATE = 10
ESC_EXECUTE_DELAY = 0.1
ESC_RAMP_PERCENT = ESC_RAMP_RATE * ESC_EXECUTE_DELAY

SERVO_AMMONIA_1 = "P9_14"
SERVO_AMMONIA_2 = "P9_16"
servos = [SERVO_AMMONIA_1, SERVO_AMMONIA_2]

VACUUM_1 = "P8_13"
VACUUM_2 = "P8_19"
escs = [VACUUM_1, VACUUM_2]
escs_on = [False, False]
escs_percent = [0, 0]

lcm_ = aiolcm.AsyncLCM()


def angle_to_dc(degrees):
    percent = degrees / 120.0
    dc = SERVO_MIN_DC + (percent * (SERVO_MAX_DC - SERVO_MIN_DC))
    return dc


def run_servo(pin, degrees):
    dc = angle_to_dc(degrees)
    PWM.set_duty_cycle(pin, dc)


def percent_to_dc(percent):
    percent = percent / 100.0
    return ESC_MIN_DC + (percent * (ESC_MAX_DC - ESC_MIN_DC))


def run_esc(pin, percent):
    dc = percent_to_dc(percent)
    PWM.set_duty_cycle(pin, dc)


def esc_arm(pin):
    # Start PWM signal
    PWM.start(pin, 0, 50)
    # Apply positive throttle
    run_esc(pin, 5.0)
    time.sleep(0.5)
    # Apply 0 throttle
    run_esc(pin, 0.0)


async def escs_execute():
    while True:
        for index, pin in enumerate(escs):
            percent = escs_percent[index]
            target_percent = ESC_OFF_PERCENT
            if escs_on[index]:
                target_percent = min(percent + ESC_RAMP_PERCENT,
                                     ESC_ON_PERCENT)

            if percent != target_percent:
                run_esc(pin, target_percent)
                escs_percent[index] = target_percent

        await asyncio.sleep(ESC_EXECUTE_DELAY)


def servo_init(pin, degrees):
    dc = angle_to_dc(degrees)
    PWM.start(pin, dc, 50)


def servo_callback(channel, msg):
    servo = Servo.decode(msg)
    if servo.id == "servo_1":
        run_servo(servos[0], servo.degrees)
    elif servo.id == "servo_2":
        run_servo(servos[1], servo.degrees)
    else:
        print("Invalid servo ID.")


def esc_toggle_callback(channel, msg):
    esc = ESCToggle.decode(msg)

    vacuum = 0 if esc.id == "vacuum_1" else 1
    escs_on[vacuum] = esc.enable


def esc_throttle_callback(channel, msg):
    esc = ESCThrottle.decode(msg)
    if esc.id == "vacuum_1":
        run_esc(escs[0], esc.percent)
    elif esc.id == "vacuum_2":
        run_esc(escs[1], esc.percent)
    else:
        print("Invalid ESC ID.")


def main():
    # Perform arming squence for ESCs
    for pin in escs:
        esc_arm(pin)

    # Initialize servos at 0 position
    for pin in servos:
        servo_init(pin, 0)

    lcm_.subscribe("/servo", servo_callback)
    lcm_.subscribe("/esc_toggle", esc_toggle_callback)
    lcm_.subscribe("/esc_throttle", esc_throttle_callback)

    run_coroutines(escs_execute(), lcm_.loop())
