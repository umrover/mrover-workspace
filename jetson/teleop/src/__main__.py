import asyncio
from math import copysign
from enum import Enum
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (Joystick, Xbox, Keyboard,
                        DriveVelCmd, MastGimbalCmd,
                        AutonState, AutonDriveControl,
                        GUICameras, Cameras, Mission,
                        RAOpenLoopCmd, HandCmd,
                        SAOpenLoopCmd, FootCmd,
                        ArmControlState, ReverseDrive,
                        Calibrate)


lcm_ = aiolcm.AsyncLCM()
lock = asyncio.Lock()
connection = None


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
        angular_op = (angular / 2) / (abs(linear) + 0.5)
        vel_left = linear - angular_op
        vel_right = linear + angular_op

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

    def send_drive_kill(self):
        drive_motor = DriveVelCmd()
        drive_motor.left = 0.0
        drive_motor.right = 0.0

        lcm_.publish('/drive_vel_cmd', drive_motor.encode())


def gimbal_control_callback(channel, msg):
    keyboardData = Keyboard.decode(msg)

    pitchData = [0.4 * float(keyboardData.w - keyboardData.s),
                 float(keyboardData.i - keyboardData.k)]

    yawData = [float(keyboardData.d - keyboardData.a),
               float(keyboardData.l - keyboardData.j)]

    gimbal_msg = MastGimbalCmd()
    gimbal_msg.pitch = pitchData
    gimbal_msg.yaw = yawData

    lcm_.publish('/mast_gimbal_cmd', gimbal_msg.encode())


class Camera:
    def __init__(self):
        self.cameras = [-1, -1, -1, -1]

        self.mission = ""

    def update_cameras(self, channel, msg):
        message = GUICameras.decode(msg)
        incoming = message.port

        if self.mission == 'Science':
            num_to_update = 2
            incoming = incoming[0:2]
        else:
            num_to_update = 4

        for i in range(num_to_update):
            if self.cameras[i] not in incoming:
                self.cameras[i] = -1
            else:
                incoming.pop(incoming.index(self.cameras[i]))
        
        for i in range(num_to_update):
            if self.cameras[i] == -1:
                self.cameras[i] = incoming[0]
                incoming.pop(0)

        self.send_camera_cmd()

    def update_ish_cameras(self, channel, msg):
        if self.mission != 'Science':
            return

        message = GUICameras.decode(msg)

        # If a camera is streamed to opposite port already
        if message.port[0] == self.cameras[3]:
            self.cameras[2] = message.port[1]
        elif message.port[1] == self.cameras[2]:
            self.cameras[3] = message.port[0]

        # Otherwise, route normally
        else:
            self.cameras[2] = message.port[0]
            self.cameras[3] = message.port[1]

        self.send_camera_cmd()

    def send_camera_cmd(self):
        camera_msg = Cameras()
        camera_msg.port = self.cameras

        lcm_.publish('/cameras_cmd', camera_msg.encode())

    def update_mission(self, channel, msg):
        message = Mission.decode(msg)
        self.mission = message.name


class ArmControl:
    class ArmType(Enum):
        UNKNOWN = 0
        RA = 1
        SA = 2

    def __init__(self):
        self.arm_control_state = "off"
        self.arm_type = self.ArmType.UNKNOWN

    def arm_control_state_callback(self, channel, msg):
        self.arm_control_state = ArmControlState.decode(msg).state
        if (self.arm_control_state != "open-loop"):
            self.send_ra_kill()
            self.send_sa_kill()

    def ra_control_callback(self, channel, msg):
        if (self.arm_control_state != "open-loop"):
            return

        self.arm_type = self.ArmType.RA

        xboxData = Xbox.decode(msg)

        motor_speeds = [quadratic(deadzone(xboxData.left_js_x, 0.15)),
                        quadratic(-deadzone(xboxData.left_js_y, 0.15)),
                        quadratic(-deadzone(xboxData.right_js_y, 0.15)),
                        quadratic(deadzone(xboxData.right_js_x, 0.15)),
                        quadratic(xboxData.right_trigger - xboxData.left_trigger),
                        (xboxData.right_bumper - xboxData.left_bumper)]

        openloop_msg = RAOpenLoopCmd()
        openloop_msg.throttle = motor_speeds

        lcm_.publish('/ra_open_loop_cmd', openloop_msg.encode())

        hand_msg = HandCmd()
        hand_msg.finger = xboxData.y - xboxData.a
        hand_msg.grip = xboxData.b - xboxData.x

        lcm_.publish('/hand_open_loop_cmd', hand_msg.encode())

    def sa_control_callback(self, channel, msg):
        if (self.arm_control_state != "open-loop"):
            return

        self.arm_type = self.ArmType.SA

        xboxData = Xbox.decode(msg)

        saMotorsData = [quadratic(deadzone(xboxData.left_js_x, 0.15)),
                        quadratic(-deadzone(xboxData.left_js_y, 0.15)),
                        quadratic(-deadzone(xboxData.right_js_y, 0.15)),
                        quadratic(xboxData.right_trigger - xboxData.left_trigger)]

        openloop_msg = SAOpenLoopCmd()
        openloop_msg.throttle = saMotorsData

        lcm_.publish('/sa_open_loop_cmd', openloop_msg.encode())

        foot_msg = FootCmd()
        foot_msg.scoop = xboxData.a - xboxData.y
        foot_msg.microscope_triad = -(xboxData.left_bumper - xboxData.right_bumper)
        lcm_.publish('/foot_open_loop_cmd', foot_msg.encode())

    def send_ra_kill(self):
        if self.arm_type != self.ArmType.RA:
            return

        arm_motor = RAOpenLoopCmd()
        arm_motor.throttle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lcm_.publish('/ra_open_loop_cmd', arm_motor.encode())

        hand_msg = HandCmd()
        hand_msg.finger = 0
        hand_msg.grip = 0
        lcm_.publish('/hand_open_loop_cmd', hand_msg.encode())

    def send_sa_kill(self):
        if self.arm_type != self.ArmType.SA:
            return

        sa_motor = SAOpenLoopCmd()
        sa_motor.throttle = [0.0, 0.0, 0.0, 0.0]

        lcm_.publish('/sa_open_loop_cmd', sa_motor.encode())

        foot_msg = FootCmd()
        foot_msg.scoop = 0
        foot_msg.microscope_triad = 0
        lcm_.publish('/foot_open_loop_cmd', foot_msg.encode())

    def ra_calibration_callback(self, channel, msg):
        if Calibrate.decode(msg).calibrated or self.arm_control_state != 'calibrating':
            return

        ra_openloop_msg = RAOpenLoopCmd()
        ra_openloop_msg.throttle = [0, 1, 0, 0, 0, 0]

        lcm_.publish('/ra_open_loop_cmd', ra_openloop_msg.encode())

    def sa_calibration_callback(self, channel, msg):
        if Calibrate.decode(msg).calibrated or self.arm_control_state != 'calibrating':
            return

        sa_openloop_msg = SAOpenLoopCmd()
        sa_openloop_msg.throttle = [0, 1, 0, 0]

        lcm_.publish('/sa_open_loop_cmd', sa_openloop_msg.encode())


def main():
    arm = ArmControl()
    drive = Drive(reverse=False)
    camera = Camera()

    def connection_state_changed(c, _):
        global connection
        if c:
            print("Connection established.")
            connection = True
        else:
            connection = False
            print("Disconnected.")
            drive.send_drive_kill()
            arm.send_ra_kill()
            arm.send_sa_kill()

    hb = heartbeatlib.JetsonHeartbeater(connection_state_changed, 0)
    # look LCMSubscription.queue_capacity if messages are discarded

    lcm_.subscribe("/auton", drive.auton_enabled_callback)
    lcm_.subscribe('/teleop_reverse_drive', drive.reverse_callback)
    lcm_.subscribe("/drive_control", drive.teleop_drive_callback)
    lcm_.subscribe("/auton_drive_control", drive.auton_drive_callback)

    lcm_.subscribe('/ra_control', arm.ra_control_callback)
    lcm_.subscribe('/sa_control', arm.sa_control_callback)
    lcm_.subscribe('/arm_control_state', arm.arm_control_state_callback)
    lcm_.subscribe('/ra_b_calib_data', arm.ra_calibration_callback)
    lcm_.subscribe('/sa_b_calib_data', arm.sa_calibration_callback)

    lcm_.subscribe('/gimbal_control', gimbal_control_callback)

    # Subscribe to updated GUI camera channels
    lcm_.subscribe('/cameras_control_ish', camera.update_ish_cameras)
    lcm_.subscribe('/cameras_control', camera.update_cameras)
    lcm_.subscribe('/cameras_mission', camera.update_mission)

    run_coroutines(hb.loop(), lcm_.loop())
