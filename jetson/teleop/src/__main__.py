import asyncio
from math import copysign
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (Joystick, Xbox, Keyboard,
                        DriveVelCmd, GimbalCmd,
                        AutonState, AutonDriveControl,
                        KillSwitch, Temperature,
                        RAOpenLoopCmd, HandCmd,
                        SAOpenLoopCmd, FootCmd,
                        ArmControlState,
                        ReverseDrive)


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
    global kill_motor, prev_killed
    if c:
        print("Connection established.")
        kill_motor = prev_killed
    else:
        print("Disconnected.")
        prev_killed = kill_motor
        send_drive_kill()
        send_arm_kill()
        send_sa_kill()


def quadratic(val):
    return copysign(val**2, val)


def deadzone(magnitude, threshold):
    temp_mag = abs(magnitude)
    if temp_mag <= threshold:
        temp_mag = 0
    else:
        temp_mag = (temp_mag - threshold)/(1 - threshold)

    return copysign(temp_mag, magnitude)


class Drive:
    def __init__(self, reverse: bool):
        self.auton_enabled = False
        self.reverse = reverse

    def reverse_callback(self, channel, msg):
        self.reverse = ReverseDrive.decode(msg).reverse

    def auton_enabled_callback(self, channel, msg):
        self.auton_enabled = AutonState.decode(msg).is_auton

    def teleop_drive_callback(self, channel, msg):
        if self.auton_enabled:
            return

        input = Joystick.decode(msg)

        # TODO: possibly add quadratic control
        linear = deadzone(input.forward_back, 0.05) * input.dampen
        angular = deadzone(input.left_right, 0.1) * input.dampen

        # Convert arcade drive to tank drive
        # TODO: possibly flip signs of rotation if necessary
        angular_op = (angular / 2) / (abs(linear) + 0.5)
        vel_left = linear + angular_op
        vel_right = linear - angular_op

        # Account for reverse
        if self.reverse:
            tmp = vel_left
            vel_left = -1 * vel_right
            vel_right = -1 * tmp

        # Scale to be within [-1, 1], if necessary
        if abs(vel_left) > 1 or abs(vel_right) > 1:
            if abs(vel_left) > abs(vel_right):
                vel_right /= abs(vel_left)
                vel_left /= abs(vel_left)
            else:
                vel_left /= abs(vel_right)
                vel_right /= abs(vel_right)

        command = DriveVelCmd()
        command.left = vel_left
        command.right = vel_right

        lcm_.publish('/drive_vel_cmd', command.encode())

    def auton_drive_callback(self, channel, msg):
        if not self.auton_enabled:
            return

        input = AutonDriveControl.decode(msg)

        command = DriveVelCmd()
        command.left = input.right_percent_velocity
        command.right = input.left_percent_velocity

        lcm_.publish('/drive_vel_cmd', command.encode())


def send_zero_arm_command():
    openloop_msg = RAOpenLoopCmd()
    openloop_msg.throttle = [0, 0, 0, 0, 0, 0]

    lcm_.publish('/ra_openloop_cmd', openloop_msg.encode())

    hand_msg = HandCmd()
    hand_msg.finger = 0
    hand_msg.grip = 0

    lcm_.publish('/hand_openloop_cmd', hand_msg.encode())


def arm_control_state_callback(channel, msg):
    arm_control_state = ArmControlState.decode(msg)

    if arm_control_state != 'open-loop':
        send_zero_arm_command()


def ra_control_callback(channel, msg):

    xboxData = Xbox.decode(msg)

    motor_speeds = [-deadzone(quadratic(xboxData.left_js_x), 0.09)/4.0,
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

    drive = Drive(reverse=False)

    lcm_.subscribe("/auton", drive.auton_enabled_callback)

    lcm_.subscribe('/teleop_reverse_drive', drive.reverse_callback)
    lcm_.subscribe("/drive_control", drive.teleop_drive_callback)
    lcm_.subscribe("/auton_drive_control", drive.auton_drive_callback)

    lcm_.subscribe('/ra_control', ra_control_callback)
    lcm_.subscribe('/sa_control', sa_control_callback)
    lcm_.subscribe('/arm_control_state', arm_control_state_callback)
    lcm_.subscribe('/gimbal_control', gimbal_control_callback)

    run_coroutines(hb.loop(), lcm_.loop(),
                   transmit_temperature(), transmit_drive_status())
