from rover_common import aiolcm
# from rover_common import talon_srx
from rover_common.aiohelper import run_coroutines
from rover_msgs import ArmPosition
lcm_ = aiolcm.AsyncLCM()


def ik_ra_control_callback(channel, msg):
    ircc_constant = 360
    input_data = ArmPosition.decode(msg)
    # Wait until hardware implements
    input_data.joint_a /= ircc_constant
    # set_demand(input_data.joint_a, talon_srx.TalonControlMode.kPositionMode)
    input_data.joint_b /= ircc_constant
    # set_demand(input_data.joint_b, talon_srx.TalonControlMode.kPositionMode)
    input_data.joint_c /= ircc_constant
    # set_demand(input_data.joint_c, talon_srx.TalonControlMode.kPositionMode)
    input_data.joint_d /= ircc_constant
    # set_demand(input_data.joint_d, talon_srx.TalonControlMode.kPositionMode)
    input_data.joint_e /= ircc_constant
    # set_demand(input_data.joint_e, talon_srx.TalonControlMode.kPositionMode)


def main():
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/ik_ra_control", ik_ra_control_callback)
    run_coroutines(lcm_.loop())
