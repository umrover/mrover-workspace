from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import DriveMotors
from .lowlevel import LowLevel
from enum import Enum
import can
from rover_common import talon_srx

lcm_ = aiolcm.AsyncLCM()
rover = None
NUM_TALONS = 4


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

    def left_drive(self, speed):
        converted_speed = 1023 * speed
        self.talons[RoverTalons.left_front.value].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)
        self.talons[RoverTalons.left_back.value].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)

    def right_drive(self, speed):
        converted_speed = 1023 * speed
        self.talons[RoverTalons.right_front.value].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)
        self.talons[RoverTalons.right_back.value].set_demand(
            converted_speed, talon_srx.TalonControlMode.kThrottle.value)


def motor_callback(channel, msg):
    print("Recieving {} bytes from {}".format(len(msg), channel))
    # Message has left and right
    # Calculate talonValue for all left and all right Motors
    m = DriveMotors.decode(msg)
    rover.left_drive(m.left)
    rover.right_drive(m.right)


def main():
    global rover
    rover = Rover()
    lcm_.subscribe("/motor", motor_callback)
    run_coroutines(lcm_.loop())
