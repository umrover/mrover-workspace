import asyncio
import math
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (Joystick, DriveVelCmd, KillSwitch,
                        Xbox, Temperature, RAOpenLoopCmd,
                        SAOpenLoopCmd, MastGimbalCmd, HandCmd,
                        Keyboard, FootCmd, ArmControlState, WristTurnCount)


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
        send_ra_kill()
        send_sa_kill()


def send_drive_kill():
    drive_motor = DriveVelCmd()
    drive_motor.left = 0.0
    drive_motor.right = 0.0

    lcm_.publish('/drive_vel_cmd', drive_motor.encode())


def send_ra_kill():
    arm_motor = RAOpenLoopCmd()
    arm_motor.throttle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    lcm_.publish('/ra_openloop_cmd', arm_motor.encode())

    hand_msg = HandCmd()
    hand_msg.finger = 0
    hand_msg.grip = 0
    lcm_.publish('/hand_openloop_cmd', hand_msg.encode())


def send_sa_kill():
    sa_motor = SAOpenLoopCmd()
    sa_motor.throttle = [0.0, 0.0, 0.0, 0.0]

    lcm_.publish('/sa_openloop_cmd', sa_motor.encode())

    foot_msg = FootCmd()
    foot_msg.scoop = 0
    foot_msg.microscope_triad = 0
    lcm_.publish('/foot_openloop_cmd', foot_msg.encode())


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

        damp = (input_data.dampen - 1)/(2)
        new_motor.left *= damp
        new_motor.right *= damp

        lcm_.publish('/drive_vel_cmd', new_motor.encode())


def gimbal_control_callback(channel, msg):
    keyboardData = Keyboard.decode(msg)
    print("mast gimbal call back")

    pitchData = [float(keyboardData.w - keyboardData.s),
                 float(keyboardData.i - keyboardData.k)]

    yawData = [float(keyboardData.d - keyboardData.a),
               float(keyboardData.l - keyboardData.j)]

    gimbal_msg = MastGimbalCmd()
    gimbal_msg.pitch = pitchData
    gimbal_msg.yaw = yawData

    print("mast gimbal call back end")

    lcm_.publish('/mast_gimbal_cmd', gimbal_msg.encode())


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


def autonomous_callback(channel, msg):
    input_data = Joystick.decode(msg)
    new_motor = DriveVelCmd()

    joystick_math(new_motor, input_data.forward_back, input_data.left_right)

    temp = new_motor.left
    new_motor.left = new_motor.right
    new_motor.right = temp

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


class ArmControl:

    def __init__(self):
        self.arm_control_state = "off"
        self.wrist_turn_count = 0

    def arm_control_state_callback(self, channel, msg):
        self.arm_control_state = ArmControlState.decode(msg)
        if (self.arm_control_state != "open-loop"):
            send_sa_kill()
            send_ra_kill()

    def ra_control_callback(self, channel, msg):

        xboxData = Xbox.decode(msg)

        motor_speeds = [deadzone(quadratic(xboxData.left_js_x), 0.09),
                        -deadzone(quadratic(xboxData.left_js_y), 0.09),
                        -deadzone(quadratic(xboxData.right_js_y), 0.09),
                        deadzone(quadratic(xboxData.right_js_x), 0.09),
                        quadratic(xboxData.right_trigger - xboxData.left_trigger),
                        (xboxData.right_bumper - xboxData.left_bumper)]

        # TODO: test open loop, might have to switch these
        if (self.wrist_turn_count == -2 and motor_speeds[5] < 0):
            motor_speeds[5] = 0
        if (self.wrist_turn_count == 2 and motor_speeds[5] > 0):
            motor_speeds[5] = 0

        openloop_msg = RAOpenLoopCmd()
        openloop_msg.throttle = motor_speeds

        lcm_.publish('/ra_openloop_cmd', openloop_msg.encode())

        hand_msg = HandCmd()
        hand_msg.finger = xboxData.y - xboxData.a
        hand_msg.grip = xboxData.b - xboxData.x

        lcm_.publish('/hand_openloop_cmd', hand_msg.encode())

    def wrist_turn_count_callback(self, channel, msg):
        self.wrist_turn_count = WristTurnCount.decode(msg) - 2

    def sa_control_callback(channel, msg):
        xboxData = Xbox.decode(msg)

        saMotorsData = [deadzone(quadratic(xboxData.left_js_x), 0.09),
                        -deadzone(quadratic(xboxData.left_js_y), 0.09),
                        -deadzone(quadratic(xboxData.right_js_y), 0.09),
                        quadratic(xboxData.right_trigger - xboxData.left_trigger)]

        openloop_msg = SAOpenLoopCmd()
        openloop_msg.throttle = saMotorsData

        lcm_.publish('/sa_openloop_cmd', openloop_msg.encode())

        foot_msg = FootCmd()
        foot_msg.scoop = xboxData.a - xboxData.y
        foot_msg.microscope_triad = -(xboxData.left_bumper - xboxData.right_bumper)
        lcm_.publish('/foot_openloop_cmd', foot_msg.encode())


def main():
    arm_control = ArmControl()
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed, 0)
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/drive_control", drive_control_callback)
    lcm_.subscribe("/autonomous", autonomous_callback)
    lcm_.subscribe('/ra_control', arm_control.ra_control_callback)
    lcm_.subscribe('/sa_control', arm_control.sa_control_callback)
    lcm_.subscribe('/arm_control_state', arm_control.arm_control_state_callback)
    lcm_.subscribe('/wrist_turn_count', arm_control.wrist_turn_count_callback)
    lcm_.subscribe('/gimbal_control', gimbal_control_callback)
    # lcm_.subscribe('/arm_toggles_button_data', arm_toggles_button_callback)

    run_coroutines(hb.loop(), lcm_.loop(),
                   transmit_temperature(), transmit_drive_status())
