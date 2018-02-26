from rover_common import aiolcm
import asyncio
import os
from rover_common.aiohelper import run_coroutines, exec_later
from rover_msgs import DriveMotors
from rover_msgs import SetParam
from rover_msgs import Encoder
from rover_msgs import SetDemand
from rover_msgs import SAMotors
from .lowlevel import LowLevel
from enum import Enum
from rover_common import talon_srx

lcm_ = aiolcm.AsyncLCM()
rover = None
NUM_TALONS = 10
CHANNEL = os.environ.get('MROVER_TALON_CAN_IFACE', 'vcan0')


class Talons(Enum):
    left_front = 2
    left_back = 7
    right_front = 4
    right_back = 8
    arm_joint_a = 5
    arm_joint_b = 6
    arm_joint_c = 9
    arm_joint_d = 3
    arm_joint_e = 1
    arm_joint_f = 0
    drill = 0
    lead_screw = 1
    door_actuator = 9
    cache = 3


class Rover:
    def __init__(self):
        self.talons = []
        for i in range(NUM_TALONS):
            self.talons.append(LowLevel(CHANNEL, i))

    async def configure_talons(self):
        # make left_back talon follow left_front
        # make right_back talon follow right_front
        await self.talons[Talons.left_back.value].set_demand(
            Talons.left_front.value,
            talon_srx.TalonControlMode.kFollowerMode.value)
        await self.talons[Talons.right_back.value].set_demand(
            Talons.right_front.value,
            talon_srx.TalonControlMode.kFollowerMode.value)
        # set up joint_d to use PID profile 0
        await self.talons[
            Talons.arm_joint_d.value].set_profile_slot_select(0)
        await self.talons[
            Talons.arm_joint_d.value].set_param(
                talon_srx.Param.ClearPositionOnLimitF.value, 1)
        await self.talons[
            Talons.arm_joint_d.value].set_param(
                talon_srx.Param.ClearPositionOnLimitR.value, 1)
        print('configured talons')

    async def percent_vbus_drive(self, talon, speed):
        converted_speed = 1023 * speed
        await self.talons[talon].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)

    async def position_pid_drive(self, talon, pos):
        await self.talons[talon].set_demand(
            pos, talon_srx.TalonControlMode.kPositionMode.value)

    async def run_all(self):
        asyncio.ensure_future(
                asyncio.gather(*(t.loop() for t in self.talons)))
        await self.configure_talons()


def drive_motor_callback(channel, msg):
    m = DriveMotors.decode(msg)
    exec_later(
        rover.percent_vbus_drive(Talons.left_front.value), m.left)
    exec_later(
        rover.percent_vbus_drive(Talons.right_front.value), m.right)


def sa_motor_callback(channel, msg):
    m = SAMotors.decode(msg)
    exec_later(
        rover.percent_vbus_drive(Talons.drill.value, m.drill))
    exec_later(
        rover.percent_vbus_drive(Talons.lead_screw.value, m.lead_screw))
    exec_later(
        rover.percent_vbus_drive(Talons.door_actuator.value, m.door_actuator))
    exec_later(
        rover.percent_vbus_drive(Talons.cache.value, m.cache))


def arm_demand_callback(channel, msg):
    m = Encoder.decode(msg)
    exec_later(
        rover.position_pid_drive(Talons.arm_joint_a.value, m.joint_a))
    exec_later(
        rover.position_pid_drive(Talons.arm_joint_b.value, m.joint_b))
    exec_later(
        rover.position_pid_drive(Talons.arm_joint_c.value, m.joint_c))
    exec_later(
        rover.position_pid_drive(Talons.arm_joint_d.value, m.joint_d))
    exec_later(
        rover.position_pid_drive(Talons.arm_joint_e.value, m.joint_e))
    exec_later(
        rover.position_pid_drive(Talons.arm_joint_f.value, m.joint_f))


def set_param_callback(channel, msg):
    m = SetParam.decode(msg)
    exec_later(rover.talons[m.deviceID].set_param(m.paramID, m.value))


def set_demand_callback(channel, msg):
    m = SetDemand.decode(msg)
    exec_later(rover.talons[m.deviceID].set_demand(m.value, m.control_mode))


async def publish_arm_encoders():
    while True:
        ec = Encoder()
        '''
        ec.joint_a = int(
            await rover.talons[Talons.arm_joint_a.value].read_enc_value()
            or 0)
        ec.joint_b = int(
            await rover.talons[Talons.arm_joint_b.value].read_enc_value()
            or 0)
        ec.joint_c = int(
            await rover.talons[Talons.arm_joint_c.value].read_enc_value()
            or 0)
        '''
        val = await rover.talons[
            Talons.arm_joint_d.value].read_enc_value() or 0
        val = max(min(val, 32767), -32768)
        ec.joint_d = int(val)
        '''
        ec.joint_e = int(
            await rover.talons[Talons.arm_joint_e.value].read_enc_value()
            or 0)
        '''

        lcm_.publish('/encoder', ec.encode())

        await asyncio.sleep(0.1)


def main():
    global rover
    rover = Rover()
    # lcm_.subscribe("/motor", drive_motor_callback)
    lcm_.subscribe("/setparam", set_param_callback)
    lcm_.subscribe("/setdemand", set_demand_callback)
    lcm_.subscribe("/sa_motors", sa_motor_callback)
    lcm_.subscribe("/arm_demand", arm_demand_callback)
    run_coroutines(lcm_.loop(), rover.run_all())
