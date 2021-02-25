import asyncio
import math
import time
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (Joystick, DriveVelCmd, KillSwitch,
                        Xbox, Temperature, RAOpenLoopCmd,
                        SAOpenLoopCmd, GimbalCmd, HandCmd,
                        Keyboard, FootCmd)


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
prev_killed = False
kill_motor = False
lock = asyncio.Lock()
front_drill_on = Toggle(False)
back_drill_on = Toggle(False)
connection = None

# duration to wait for auton commands before killing drive (in seconds)
AUTON_TIMEOUT = 1.0

# last time auton updated motors
auton_last = 0.0
auton_running = False


def send_drive_kill():
    drive_motor = DriveVelCmd()
    drive_motor.left = 0.0
    drive_motor.right = 0.0

    lcm_.publish('/drive_vel_cmd', drive_motor.encode())


def send_arm_kill():
    arm_motor = RAOpenLoopCmd()
    arm_motor.throttle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    lcm_.publish('/ra_openloop_cmd', arm_motor.encode())


def send_sa_kill():
    sa_motor = SAOpenLoopCmd()
    sa_motor.throttle = [0.0, 0.0, 0.0]

    lcm_.publish('/sa_openloop_cmd', sa_motor.encode())


def connection_state_changed(c, _):
    global kill_motor, prev_killed, connection
    if c:
        print("Connection established.")
        kill_motor = prev_killed
        connection = True
    else:
        connection = False
        print("Disconnected.")
        prev_killed = kill_motor
        send_drive_kill()
        send_arm_kill()
        send_sa_kill()


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
    global kill_motor, connection

    if not connection:
        return

    input_data = Joystick.decode(msg)

    if input_data.kill:
        kill_motor = True
    elif input_data.restart:
        kill_motor = False

    if kill_motor:
        send_drive_kill()
    else:
        new_motor = DriveVelCmd()
        input_data.forward_back = -quadratic(input_data.forward_back)
        magnitude = deadzone(input_data.forward_back, 0.04)
        theta = deadzone(input_data.left_right, 0.1)

        joystick_math(new_motor, magnitude, theta)

        damp = (input_data.dampen - 1)/(-2)
        new_motor.left *= damp
        new_motor.right *= damp

        lcm_.publish('/drive_vel_cmd', new_motor.encode())


def ra_control_callback(channel, msg):
    xboxData = Xbox.decode(msg)

    motor_speeds = [-deadzone(quadratic(xboxData.left_js_x), 0.09),
                    -deadzone(quadratic(xboxData.left_js_y), 0.09),
                    deadzone(quadratic(xboxData.right_js_y), 0.09),
                    deadzone(quadratic(xboxData.right_js_x), 0.09),
                    quadratic(xboxData.right_trigger -
                              xboxData.left_trigger),
                    (xboxData.right_bumper - xboxData.left_bumper)]

    openloop_msg = RAOpenLoopCmd()
    openloop_msg.throttle = motor_speeds

    lcm_.publish('/ra_openloop_cmd', openloop_msg.encode())

    hand_msg = HandCmd()
    hand_msg.finger = xboxData.y - xboxData.a
    hand_msg.grip = xboxData.b - xboxData.x

    lcm_.publish('/hand_openloop_cmd', hand_msg.encode())


def autonomous_callback(channel, msg):
    global auton_last, auton_running

    # TODO look into communicating with GUI to get mode
    if not auton_running:
        auton_running = True

    auton_last = time.time()

    input_data = Joystick.decode(msg)
    new_motor = DriveVelCmd()

    joystick_math(new_motor, input_data.forward_back, input_data.left_right)

    lcm_.publish('/drive_vel_cmd', new_motor.encode())


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


async def check_auton_status():
    global AUTON_TIMEOUT, auton_last, auton_running

    while True:
        print('Checking status')

        # if auton stopped sending messages, kill drive
        if auton_running and time.time() - auton_last > AUTON_TIMEOUT:
            print('Attempting to kill')
            send_drive_kill()
            auton_running = False

        # sleep briefly before checking again
        await asyncio.sleep(0.2 * AUTON_TIMEOUT)


def sa_control_callback(channel, msg):
    xboxData = Xbox.decode(msg)

    saMotorsData = [deadzone(quadratic(xboxData.left_js_x), 0.09),
                    -deadzone(quadratic(xboxData.left_js_y), 0.09),
                    -deadzone(quadratic(xboxData.right_js_y), 0.09)]

    openloop_msg = SAOpenLoopCmd()
    openloop_msg.throttle = saMotorsData

    lcm_.publish('/sa_openloop_cmd', openloop_msg.encode())

    foot_msg = FootCmd()
    foot_msg.claw = xboxData.a - xboxData.y
    foot_msg.sensor = 0.5 * (xboxData.left_bumper - xboxData.right_bumper)
    lcm_.publish('/foot_openloop_cmd', foot_msg.encode())


def gimbal_control_callback(channel, msg):
    keyboardData = Keyboard.decode(msg)

    pitchData = [keyboardData.s - keyboardData.w,
                 keyboardData.i - keyboardData.k]

    yawData = [keyboardData.a - keyboardData.d,
               keyboardData.j - keyboardData.l]

    gimbal_msg = GimbalCmd()
    gimbal_msg.pitch = pitchData
    gimbal_msg.yaw = yawData

    lcm_.publish('/gimbal_openloop_cmd', gimbal_msg.encode())


def main():
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed, 0)
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/drive_control", drive_control_callback)
    lcm_.subscribe("/autonomous", autonomous_callback)
    lcm_.subscribe('/ra_control', ra_control_callback)
    lcm_.subscribe('/sa_control', sa_control_callback)
    lcm_.subscribe('/gimbal_control', gimbal_control_callback)
    # lcm_.subscribe('/arm_toggles_button_data', arm_toggles_button_callback)

    run_coroutines(hb.loop(), lcm_.loop(),
                   transmit_temperature(), transmit_drive_status(), check_auton_status())