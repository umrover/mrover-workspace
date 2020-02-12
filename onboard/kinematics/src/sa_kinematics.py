import numpy as np
from scipy.interpolate import CubicSpline


class SAKinematics:
    def __init__(self, lcm, arm):
        self.lcm_ = lcm
        self.arm_ = arm

    def FK(self, )

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

        return cs

    def spline_fitting(self, path):
        x_ = np.linspace(0, 1, len(path))
        return CubicSpline(x_, path)
