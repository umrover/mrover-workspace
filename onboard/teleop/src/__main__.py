import asyncio
import math
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (Joystick, DriveMotors, KillSwitch,
                        Xbox, Temperature, SAMotors, OpenLoopRAMotor)


class Toggle:

    def __init__(self, toggle):
        self.toggle = toggle
        self.previous = False
        self.input = False
        self.last_input = False

    def new_reading(self, reading):
        self.input = reading
        if self.input and not self.last_input:
            # just pushed
            self.last_input = True
            self.toggle = not self.toggle
        elif not self.input and self.last_input:
            # just released
            self.last_input = False

        self.previous = reading
        return self.toggle


lcm_ = aiolcm.AsyncLCM()
kill_motor = False
lock = asyncio.Lock()
drill_on = Toggle(False)


def connection_state_changed(c, _):
    global kill_motor
    if c:
        print("Connection established.")
    else:
        print("Disconnected.")

        # Kill drive motors
        kill_motor = True
        drive_motor = DriveMotors()
        drive_motor.left = 0.0
        drive_motor.right = 0.0

        lcm_.publish('/motor', drive_motor.encode())

        # Kill arm motors
        for i in range(7):
            arm_motor = OpenLoopRAMotor()
            arm_motor.joint_id = i
            arm_motor.speed = 0.0
            lcm_.publish('/arm_motors', arm_motor.encode())

        # Kill SA motors
        sa_motor = SAMotors()
        sa_motor.drill = 0.0
        sa_motor.lead_screw = 0.0
        sa_motor.door_actuator = 0.0
        sa_motor.cache = 0.0

        lcm_.publish('/sa_motors', sa_motor.encode())


def quadratic(val):
    return math.copysign(val**2, val)


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
        new_motor.right *= 1 - (theta * 0.75)
    elif theta < 0:
        new_motor.left *= 1 + (theta * 0.75)

    if magnitude < 0:
        new_motor.left *= -1
        new_motor.right *= -1
    elif magnitude == 0:
        new_motor.left += theta
        new_motor.right -= theta


def drive_control_callback(channel, msg):
    global kill_motor

    input_data = Joystick.decode(msg)
    input_data.forward_back = -quadratic(input_data.forward_back)

    if input_data.kill:
        kill_motor = True
    elif input_data.restart:
        kill_motor = False

    new_motor = DriveMotors()

    if kill_motor:
        new_motor.left = 0
        new_motor.right = 0

    else:
        magnitude = deadzone(input_data.forward_back, 0.04)
        theta = deadzone(input_data.left_right, 0.1)

        joystick_math(new_motor, magnitude, theta)

        damp = (input_data.dampen - 1)/(-2)
        new_motor.left *= damp
        new_motor.right *= damp

    lcm_.publish('/motor', new_motor.encode())


def arm_control_callback(channel, msg):
    xbox = Xbox.decode(msg)
    motorSpeeds = [-deadzone(quadratic(xbox.left_js_x), 0.09)*.5,
                   -deadzone(quadratic(xbox.left_js_y), 0.09)*.5,
                   quadratic(xbox.left_trigger - xbox.right_trigger)*.60,
                   deadzone(quadratic(xbox.right_js_y), 0.09)*.75,
                   deadzone(quadratic(xbox.right_js_x), 0.09)*.75,
                   (xbox.d_pad_right - xbox.d_pad_left)*0.60,
                   (xbox.right_bumper - xbox.left_bumper)]

    for i in range(7):
        openLoopMsg = OpenLoopRAMotor()
        openLoopMsg.joint_id = i
        openLoopMsg.speed = motorSpeeds[i]
        lcm_.publish('/arm_motors', openLoopMsg.encode())


def sa_control_callback(channel, msg):
    global drill_on, door_open
    xbox = Xbox.decode(msg)
    new_sa_motors = SAMotors()

    val = drill_on.new_reading(xbox.right_bumper > 0.5)
    new_sa_motors.drill = -0.9 if val else 0.0
    new_sa_motors.lead_screw = deadzone(xbox.left_js_y, 0.1)
    new_sa_motors.lead_screw = math.copysign(
        new_sa_motors.lead_screw ** 2,
        new_sa_motors.lead_screw)
    if xbox.d_pad_up:
        new_sa_motors.door_actuator = 0.5
    elif xbox.d_pad_down:
        new_sa_motors.door_actuator = -0.5
    else:
        new_sa_motors.door_actuator = 0
    new_sa_motors.cache = deadzone(xbox.right_js_x, 0.2)

    lcm_.publish('/sa_motors', new_sa_motors.encode())


def autonomous_callback(channel, msg):
    input_data = Joystick.decode(msg)
    new_motor = DriveMotors()

    joystick_math(new_motor, input_data.forward_back, input_data.left_right)

    lcm_.publish('/motor', new_motor.encode())


async def transmit_temperature():
    while True:
        new_temps = Temperature()

        try:
            with open("/sys/class/hwmon/hwmon0/temp1_input", "r") as bcpu_file:
                new_temps.bcpu_temp = int(bcpu_file.read())
            with open("/sys/class/hwmon/hwmon2/temp1_input", "r") as gpu_file:
                new_temps.gpu_temp = int(gpu_file.read())
            with open("/sys/class/hwmon/hwmon4/temp1_input", "r") \
                    as tboard_file:
                new_temps.tboard_temp = int(tboard_file.read())
        except FileNotFoundError:
            print("Temperature files not found")
            return

        with await lock:
            lcm_.publish('/temperature', new_temps.encode())

        # print("Published new tempertues")
        # print("bcpu temp: {} gpu temp: {} tboard temp: {} ".format(
        #     new_temps.bcpu_temp/1000, new_temps.gpu_temp/1000,
        #     new_temps.tboard_temp/1000))
        await asyncio.sleep(1)


async def transmit_drive_status():
    global kill_motor
    while True:
        new_kill = KillSwitch()
        new_kill.killed = kill_motor
        with await lock:
            lcm_.publish('/kill_switch', new_kill.encode())
        # print("Published new kill message: {}".format(kill_motor))
        await asyncio.sleep(1)


def main():
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed, 0)
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/drive_control", drive_control_callback)
    lcm_.subscribe("/autonomous", autonomous_callback)
    lcm_.subscribe('/arm_control', arm_control_callback)
    lcm_.subscribe('/sa_control', sa_control_callback)

    run_coroutines(hb.loop(), lcm_.loop(),
                   transmit_temperature(), transmit_drive_status())
