import os
from .mrover_arm import MRoverArm
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm


def main():
    config_path = os.environ['MROVER_CONFIG']
    geom_file = config_path + '/config_kinematics/mrover_arm_geom.json'
    args = {
        'geom_file': geom_file,
    }
    lcm_ = aiolcm.AsyncLCM()

    arm = MRoverArm(args, lcm_)

    lcm_.subscribe("/arm_position", arm.arm_position_callback)
    lcm_.subscribe("/target_orientation", arm.target_orientation_callback)
    lcm_.subscribe("/target_angles", arm.target_angles_callback)
    lcm_.subscribe("/confirmation", arm.arm_position_callback)
    lcm_.subscribe("/motion_execute", arm.motion_execute_callback)
    lcm_.subscribe("/simulation_mode", arm.simulation_mode_callback)
    lcm_.subscribe("/ik_arm_control", arm.cartesian_control_callback)
    lcm_.subscribe("/lock_joint_e", arm.lock_e_callback)
    lcm_.subscribe("/ik_enabled", arm.ik_enabled_callback)

    run_coroutines(lcm_.loop(), arm.execute_spline())


if __name__ == "__main__":
    main()
