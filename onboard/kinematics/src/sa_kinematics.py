import numpy as np
from scipy.interpolate import CubicSpline
from rover_msgs import SAClosedLoopCmd
import asyncio
from .logger import logger


class SAKinematics:
    def __init__(self, lcm, arm):
        self.lcm_ = lcm
        self.arm_ = arm
        self.spline_t = 0
        self.current_spline = []
        self.enable_execute = False

    def plan_return_to_origin(self, cur_pos):
        translator = cur_pos[0]
        joint_a = cur_pos[1]
        joint_b = cur_pos[2]
        path = [[translator, joint_a, joint_b]]
        path.append([0, joint_a, joint_b])
        path.append([0, 0, 0])

        cs = self.spline_fitting(path)
        print(path)
        print(cs)

        self.current_spline = cs

        return cs

    def spline_fitting(self, path):
        x_ = np.linspace(0, 1, len(path))
        return CubicSpline(x_, path)

    def execute_callback(self, channel, msg):
        print("is this being called?")
        self.enable_execute = True

    def publish_config(self, angles, torques, channel):
        sa_clsd_loop = SAClosedLoopCmd()
        sa_clsd_loop.angle = angles
        sa_clsd_loop.torque = torques

        self.lcm_.publish(channel, sa_clsd_loop.encode())

    async def execute_spline(self):
        logger.info("Executing path on SA Arm")
        self.spline_t = 0
        while True:
            if self.enable_execute:

                logger.info('spline time: {}'.format(self.spline_t))

                target_angs = self.current_spline(self.spline_t)
                torques = [0, 0, 0]

                self.publish_config(target_angs, torques, '/sa_closedloop_cmd')
                print(target_angs)
                self.spline_t += 0.01

                if self.spline_t >= 1:
                    self.enable_execute = False
            await asyncio.sleep(0.001)
        return
