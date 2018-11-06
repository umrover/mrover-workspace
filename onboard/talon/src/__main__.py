from rover_common import aiolcm
import asyncio
import os
from rover_common.aiohelper import run_coroutines, exec_later
from rover_msgs import DriveMotors
from rover_msgs import SetParam
from rover_msgs import Encoder
from rover_msgs import SetDemand
from rover_msgs import SAMotors
from rover_msgs import OpenLoopRAMotors
from .lowlevel import LowLevel
from enum import Enum
from rover_common import talon_srx

lcm_ = aiolcm.AsyncLCM()
rover = None
NUM_TALONS = 10
CHANNEL = os.environ.get('MROVER_TALON_CAN_IFACE', 'vcan0')


class Talons(Enum):
    left_front = 0
    left_back = 1
    right_front = 2
    right_back = 3
    arm_joint_a = 4
    arm_joint_b = 5
    arm_joint_c = 6
    arm_joint_d = 7
    arm_joint_e = 8
    arm_joint_f = 9
    drill = 4
    lead_screw = 5
    door_actuator = 9
    cache = 6


class Rover:
    def __init__(self):
        self.talons = []
        for i in range(NUM_TALONS):
            self.talons.append(LowLevel(CHANNEL, i))

    async def configure_talons(self):
        # make left_back talon follow left_front (Follower Mode)
        await self.talons[Talons.left_back.value].set_demand(
            Talons.left_front.value,
            talon_srx.TalonControlMode.kFollowerMode.value)
        # make right_back talon follow right_front (Follower Mode)
        await self.talons[Talons.right_back.value].set_demand(
            Talons.right_front.value,
            talon_srx.TalonControlMode.kFollowerMode.value)
        print('configured talons')

    # This function drives a talon at percent speed
    #
    # talon:  Talon SRX to drive
    # speed:  [-1.0, 1.0]
    async def percent_vbus_drive(self, talon, speed):
        converted_speed = 1023 * speed
        await self.talons[talon].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)

    # This function drives a talon to a target position.
    #
    # talon:  Talon SRX to drive
    # pos:    target encoder position
    async def position_pid_drive(self, talon, pos):
        await self.talons[talon].set_demand(
            pos, talon_srx.TalonControlMode.kPositionMode.value)

    # This function sets the current limit for a talon.
    #
    # talon:  Talon SRX to set current limit on
    # amps:   current limit in amps
    async def set_current_lim(self, talon, amps):
        await self.talons[talon].set_param(
            talon_srx.Param.CurrentLimThreshold.value, amps)

    # Configures the encoder counts per revolution for a talon.
    #
    # talon:          Talon SRX to configure
    # codes_per_rev:  encoder ticks per revolution
    async def config_encoder_codes_per_rev(self, talon, codes_per_rev):
        await self.talons[talon].set_param(
            talon_srx.Param.NumberEncoderCPR.value, codes_per_rev)

    async def run_all(self):
        asyncio.ensure_future(
                asyncio.gather(*(t.loop() for t in self.talons)))
        await self.configure_talons()


# Callback for DriveMotors LCM message
def drive_motor_callback(channel, msg):
    m = DriveMotors.decode(msg)
    exec_later(
        rover.percent_vbus_drive(Talons.left_front.value, m.left))
    exec_later(
        rover.percent_vbus_drive(Talons.right_front.value, m.right))


# Callback for SAMotors LCM Message
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


# Callback for Encoder LCM message (driving arm to position)
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


# Callback for OpenLoopRA LCM Message (drive arm with percent drive)
def open_loop_arm_callback(channel, msg):
    m = OpenLoopRAMotors.decode(msg)
    exec_later(
        rover.percent_vbus_drive(Talons.arm_joint_a.value, m.joint_a))
    exec_later(
        rover.percent_vbus_drive(Talons.arm_joint_b.value, m.joint_b))
    exec_later(
        rover.percent_vbus_drive(Talons.arm_joint_c.value, m.joint_c))
    exec_later(
        rover.percent_vbus_drive(Talons.arm_joint_d.value, m.joint_d))
    exec_later(
        rover.percent_vbus_drive(Talons.arm_joint_e.value, m.joint_e))
    exec_later(
        rover.percent_vbus_drive(Talons.arm_joint_f.value, m.joint_f))


# Callback for SetParam LCM Message
def set_param_callback(channel, msg):
    m = SetParam.decode(msg)
    exec_later(rover.talons[m.deviceID].set_param(m.paramID, m.value))


# Callback for SetDemand LCM Message
def set_demand_callback(channel, msg):
    m = SetDemand.decode(msg)
    exec_later(rover.talons[m.deviceID].set_demand(m.value, m.control_mode))


# Publish encoder positions for each RA motor.
async def publish_arm_encoders():
    while True:
        ec = Encoder()
        ec.joint_a = int(
            await rover.talons[Talons.arm_joint_a.value].get_enc_pos() or 0)
        ec.joint_b = int(
            await rover.talons[Talons.arm_joint_b.value].get_enc_pos() or 0)
        ec.joint_c = int(
            await rover.talons[Talons.arm_joint_c.value].get_enc_pos() or 0)
        ec.joint_d = int(
            await rover.talons[Talons.arm_joint_d.value].get_enc_pos() or 0)
        ec.joint_e = int(
            await rover.talons[Talons.arm_joint_e.value].get_enc_pos() or 0)
        ec.joint_f = int(
            await rover.talons[Talons.arm_joint_f.value].get_enc_pos() or 0)

        lcm_.publish('/encoder', ec.encode())

        await asyncio.sleep(0.1)


def main():
    global rover
    rover = Rover()
    lcm_.subscribe("/motor", drive_motor_callback)
    lcm_.subscribe("/set_param", set_param_callback)
    lcm_.subscribe("/set_demand", set_demand_callback)
    lcm_.subscribe("/sa_motors", sa_motor_callback)
    lcm_.subscribe("/arm_demand", arm_demand_callback)
    lcm_.subscribe("/arm_motors", open_loop_arm_callback)
    run_coroutines(publish_arm_encoders(), lcm_.loop(), rover.run_all())
