from rover_common import aiolcm
# from rover_common import talon_srx
from rover_common.aiohelper import run_coroutines
from rover_msgs import ArmPosition, Encoder
lcm_ = aiolcm.AsyncLCM()

ircc_constant = 1


def ik_ra_control_callback(channel, msg):
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


def encoder_callback(channel, msg):
    encoder = Encoder.decode(msg)
    armPos = ArmPosition()
    armPos.joint_a = encoder.joint_a * ircc_constant
    armPos.joint_b = encoder.joint_b * ircc_constant
    armPos.joint_c = encoder.joint_c * ircc_constant
    armPos.joint_d = encoder.joint_d * ircc_constant
    armPos.joint_e = encoder.joint_e * ircc_constant
    lcm_.publish("/fk_arm_control", armPos.encode())


def main():
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/ik_ra_control", ik_ra_control_callback)
    lcm_.subscribe("/encoder", encoder_callback)
    run_coroutines(lcm_.loop())
