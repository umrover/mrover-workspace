from rover_common import aiolcm
import asyncio
import os
from rover_common.aiohelper import run_coroutines, exec_later
from rover_msgs import DriveMotors
from rover_msgs import SetParam
from rover_msgs import Encoder
from rover_msgs import SetDemand
from .lowlevel import LowLevel
from enum import Enum
from rover_common import talon_srx

lcm_ = aiolcm.AsyncLCM()
rover = None
NUM_TALONS = 10
CHANNEL = os.environ.get('MROVER_TALON_CAN_IFACE', 'vcan0')


class RoverTalons(Enum):
    left_front = 2
    left_back = 7
    right_front = 4
    right_back = 8
    arm_joint_a = 5


class Rover:
    def __init__(self):
        self.talons = []
        for i in range(NUM_TALONS):
            self.talons.append(LowLevel(CHANNEL, i))

    async def configure_talons(self):
        # make left_back talon follow left_front
        # make right_back talon follow right_front
        await self.talons[RoverTalons.left_back.value].set_demand(
            RoverTalons.left_front.value,
            talon_srx.TalonControlMode.kFollowerMode.value)
        await self.talons[RoverTalons.right_back.value].set_demand(
            RoverTalons.right_front.value,
            talon_srx.TalonControlMode.kFollowerMode.value)
        print('configured talons')

    async def left_drive(self, speed):
        converted_speed = 1023 * speed
        await self.talons[RoverTalons.left_front.value].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)

    async def right_drive(self, speed):
        converted_speed = 1023 * speed
        await self.talons[RoverTalons.right_front.value].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)

    async def run_all(self):
        asyncio.ensure_future(
                asyncio.gather(*(t.loop() for t in self.talons)))
        await self.configure_talons()


def motor_callback(channel, msg):
    # Message has left and right
    # Calculate talonValue for all left and all right Motors
    m = DriveMotors.decode(msg)
    exec_later(rover.left_drive(m.left))
    exec_later(rover.right_drive(m.right))


def set_param_callback(channel, msg):
    m = SetParam.decode(msg)
    exec_later(rover.talons[m.deviceID].set_param(m.paramID, m.value))


def set_demand_callback(channel, msg):
    m = SetDemand.decode(msg)
    exec_later(rover.talons[m.deviceID].set_demand(m.value, m.control_mode))


async def publish_response():
    while True:
        ec = Encoder()
        ec.joint_a = int(
            await rover.talons[RoverTalons.left_front.value].read_enc_value()
            or 0)
        ec.joint_b = int(
            await rover.talons[RoverTalons.left_back.value].read_enc_value()
            or 0)
        ec.joint_c = int(
            await rover.talons[RoverTalons.right_front.value].read_enc_value()
            or 0)
        ec.joint_d = int(
            await rover.talons[RoverTalons.right_back.value].read_enc_value()
            or 0)

        lcm_.publish('/encoder', ec.encode())

        await asyncio.sleep(0.1)


def main():
    global rover
    rover = Rover()
    lcm_.subscribe("/motor", motor_callback)
    lcm_.subscribe("/setparam", set_param_callback)
    lcm_.subscribe("/setdemand", set_demand_callback)
    run_coroutines(lcm_.loop(), publish_response(), rover.run_all())
