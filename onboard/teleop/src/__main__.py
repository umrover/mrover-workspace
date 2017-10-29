import asyncio
import random
import math
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import Odometry, Joystick, Motors


lcm_ = aiolcm.AsyncLCM()
kill_switch = False
lock = asyncio.Lock()


def connection_state_changed(c):
    if c:
        print("Connection established.")
    else:
        print("Disconnected.")


def deadzone(magnitude, threshold):
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold)/(1 - threshold)

    return math.copysign(temp_mag, magnitude)


def joystick_callback(channel, msg):
    global kill_switch

    if kill_switch:
        return

    input_data = Joystick.decode(msg)

    new_motor = Motors()

    if input_data.kill:
        new_motor.left = 0
        new_motor.right = 0
        kill_switch = True
    else:
        magnitude = deadzone(input_data.forward_back, 0.1)
        theta = deadzone(input_data.left_right, 0.1)

        new_motor.left = abs(magnitude)
        new_motor.right = new_motor.left

        if theta > 0:
            new_motor.right *= 1 - theta/2
        elif theta < 0:
            new_motor.left *= 1 + theta/2

        if magnitude < 0:
            new_motor.left *= -1
            new_motor.right *= -1
        elif magnitude == 0:
            new_motor.left += theta/2
            new_motor.right -= theta/2

    lcm_.publish('/motor', new_motor.encode())


async def transmit_fake_odometry():
    while True:
        new_odom = Odometry()
        new_odom.latitude_deg = random.uniform(42, 43)
        new_odom.longitude_deg = random.uniform(-84, -82)
        new_odom.bearing_deg = random.uniform(0, 359)

        with await lock:
            lcm_.publish('/odom', new_odom.encode())

        # print("Published new odometry")
        await asyncio.sleep(0.5)


async def transmit_fake_joystick():
    while True:
        new_joystick = Joystick()
        new_joystick.forward_back = random.uniform(-1, 1)
        new_joystick.left_right = random.uniform(-1, 1)
        new_joystick.kill = False

        with await lock:
            lcm_.publish('/joystick', new_joystick.encode())

        print("Published new joystick\nfb: {}   lr: {}"
              .format(new_joystick.forward_back, new_joystick.left_right))
        await asyncio.sleep(0.5)


def motor_callback(channel, msg):
    input_data = Motors.decode(msg)
    # This function is for testing the joystick-motor algorithm
    print("Left: {}  Right: {}\n".format(input_data.left, input_data.right))


def main():
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed)
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/joystick", joystick_callback)
    # lcm_.subscribe('/motor', motor_callback)
    run_coroutines(hb.loop(), transmit_fake_odometry(),
                   transmit_fake_joystick(), lcm_.loop())
