from rover_common import aiolcm
import asyncio
from rover_common.aiohelper import run_coroutines
from rover_msgs import DriveMotors
from rover_msgs import SetParam
from rover_msgs import Encoder
from rover_msgs import SetDemand
from .lowlevel import LowLevel
from enum import Enum
import can
from rover_common import talon_srx

lcm_ = aiolcm.AsyncLCM()
rover = None
NUM_TALONS = 4
can.rc['interface'] = 'socketcan_ctypes'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 500000


class RoverTalons(Enum):
    left_front = 0
    left_back = 1
    right_front = 2
    right_back = 3


class Rover:
    def __init__(self):
        self.talons = []
        for i in range(NUM_TALONS):
            self.talons.append(LowLevel(can.interface.Bus(), i))
        # make left_back talon follow left_front
        self.talons[RoverTalons.left_back.value].set_demand(
            RoverTalons.left_front.value,
            talon_srx.TalonControlMode.kFollowerMode.value)
        # make right_back talon follow right_front
        self.talons[RoverTalons.right_back.value].set_demand(
            RoverTalons.right_front.value,
            talon_srx.TalonControlMode.kFollowerMode.value)

    def left_drive(self, speed):
        converted_speed = 1023 * speed
        self.talons[RoverTalons.left_front.value].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)

    def right_drive(self, speed):
        converted_speed = 1023 * speed
        self.talons[RoverTalons.right_front.value].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)


def motor_callback(channel, msg):
    print("Recieving {} bytes from {}".format(len(msg), channel))
    # Message has left and right
    # Calculate talonValue for all left and all right Motors
    m = DriveMotors.decode(msg)
    rover.left_drive(m.left)
    rover.right_drive(m.right)


def set_param_callback(channel, msg):
    print("Recieving {} bytes from {}".format(len(msg), channel))
    m = SetParam.decode(msg)
    rover.talons[m.deviceID].set_param(m.paramID, m.value)


def set_demand_callback(channel, msg):
    print("Recieving {} bytes from {}".format(len(msg), channel))
    m = SetDemand.decode(msg)
    rover.talons[m.deviceID].set_demand(m.value, m.control_mode)


async def publish_response():
    while True:
        ec = Encoder()
        ec.joint_a = int(
            rover.talons[RoverTalons.left_front.value].read_enc_value() or 0)
        ec.joint_b = int(
            rover.talons[RoverTalons.left_back.value].read_enc_value() or 0)
        ec.joint_c = int(
            rover.talons[RoverTalons.right_front.value].read_enc_value() or 0)
        ec.joint_d = int(
            rover.talons[RoverTalons.right_back.value].read_enc_value() or 0)

        lcm_.publish('/encoder', ec.encode())

        print("Published response")
        await asyncio.sleep(0.1)


def main():
    global rover
    rover = Rover()
    lcm_.subscribe("/motor", motor_callback)
    lcm_.subscribe("/setparam", set_param_callback)
    lcm_.subscribe("/setdemand", set_demand_callback)
    run_coroutines(lcm_.loop(), publish_response())
