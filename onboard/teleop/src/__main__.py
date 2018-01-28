import asyncio
import random
import math
import time
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (Odometry, Joystick, DriveMotors, Sensors,
                        Kill_switches, Temperature)

lcm_ = aiolcm.AsyncLCM()
kill_motor = False
lock = asyncio.Lock()


def connection_state_changed(c, _):
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


def joystick_math(new_motor, magnitude, theta):
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
        new_motor.left += theta
        new_motor.right -= theta


def joystick_callback(channel, msg):
    global kill_motor

    input_data = Joystick.decode(msg)

    new_kill_msg = Kill_switches()

    if kill_motor:
        if input_data.restart:
            kill_motor = False
            new_kill_msg.killed = False
            lcm_.publish('/kill_switch', new_kill_msg.encode())
        else:
            return

    damp = (input_data.dampen - 1)/(-2)

    new_motor = DriveMotors()

    if input_data.kill:
        new_motor.left = 0
        new_motor.right = 0
        kill_motor = True
        new_kill_msg.killed = True
        lcm_.publish('/kill_switch', new_kill_msg.encode())

    else:
        magnitude = deadzone(input_data.forward_back, 0.2)
        theta = deadzone(input_data.left_right, 0.1)

        joystick_math(new_motor, magnitude, theta)

        new_motor.left *= new_motor.left*damp
        new_motor.right *= new_motor.right*damp

    lcm_.publish('/motor', new_motor.encode())


def autonomous_callback(channel, msg):
    input_data = Joystick.decode(msg)
    new_motor = DriveMotors()

    joystick_math(new_motor, input_data.forward_back, input_data.left_right)

    lcm_.publish('/motor', new_motor.encode())


async def transmit_fake_odometry():
    while True:
        new_odom = Odometry()
        new_odom.latitude_deg = random.randint(42, 43)
        new_odom.longitude_deg = random.randint(-84, -82)
        new_odom.latitude_min = random.uniform(0, 60)
        new_odom.longitude_min = random.uniform(0, 60)
        new_odom.bearing_deg = random.uniform(0, 359)

        with await lock:
            lcm_.publish('/odom', new_odom.encode())

        print("Published new odometry")
        await asyncio.sleep(0.5)


async def transmit_fake_joystick():
    while True:
        new_joystick = Joystick()
        new_joystick.forward_back = random.uniform(-1, 1)
        new_joystick.left_right = random.uniform(-1, 1)
        new_joystick.dampen = random.uniform(-1, 1)
        new_joystick.kill = False
        new_joystick.restart = False

        with await lock:
            lcm_.publish('/joystick', new_joystick.encode())

        print("Published new joystick\nfb: {}   lr: {}"
              .format(new_joystick.forward_back, new_joystick.left_right))
        await asyncio.sleep(0.5)


async def transmit_fake_sensors():
    while True:
        new_sensors = Sensors()
        new_sensors.timestamp = int(time.time())
        new_sensors.temperature = random.uniform(0, 100)
        new_sensors.moisture = random.uniform(0, 100)
        new_sensors.pH = random.uniform(0, 14)
        new_sensors.soil_conductivity = random.uniform(0, 100)
        new_sensors.uv = random.uniform(0, 100)

        with await lock:
            lcm_.publish('/sensors', new_sensors.encode())

        print("Published new fake sensor data")
        print("temp: {} moisture: {} ph: {} soil: {} uv: "
              .format(new_sensors.temperature, new_sensors.moisture,
                      new_sensors.pH, new_sensors.soil_conductivity,
                      new_sensors.uv))
        await asyncio.sleep(1)


async def transmit_temperature():
    while True:
        new_temps = Temperature()
        with open("/sys/class/hwmon/hwmon0/temp1_input", "r") as bcpu_file:
            bcpu_temp = int(bcpu_file.read())
        with open("/sys/class/hwmon/hwmon2/temp1_input", "r") as gpu_file:
            gpu_temp = int(gpu_file.read())
        with open("/sys/class/hwmon/hwmon4/temp1_input", "r") as tboard_file:
            tboard_temp = int(tboard_file.read())

        with await lock:
            lcm_.publish('/temperature', new_temps.encode())

        print("Published new tempertues")
        print("bcpu temp: {} gpu temp: {} tboard temp: {} ".format(
            bcpu_temp/1000, gpu_temp/1000, tboard_temp/1000))
        await asyncio.sleep(1)


def motor_callback(channel, msg):
    input_data = DriveMotors.decode(msg)
    # This function is for testing the joystick-motor algorithm
    print("Left: {}  Right: {}\n".format(input_data.left, input_data.right))


def main():
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed, 0)
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/joystick", joystick_callback)
    lcm_.subscribe("/autonomous", autonomous_callback)
    lcm_.subscribe('/motor', motor_callback)

    new_kill_msg = Kill_switches()
    new_kill_msg.killed = False
    lcm_.publish('/kill_switch', new_kill_msg.encode())

    run_coroutines(hb.loop(), transmit_fake_odometry(),
                   transmit_fake_sensors(), lcm_.loop(),
                   transmit_temperature())
