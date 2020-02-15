import numpy as np
from scipy.interpolate import interp1d
from rover_msgs import (SAClosedLoopCmd, ArmPosition)
import asyncio
from .logger import logger

from numpy import linalg as LA
from.utils import calculate_torque

class SAKinematics:
    def __init__(self, lcm, arm):
        self.lcm_ = lcm
        self.arm_ = arm
        self.spline_t = 0
        self.current_spline = []
        self.enable_execute = False

    def FK(self, cur_robot):
        joints = cur_robot.all_joints
        global_transform = np.eye(4)
        # set the base to be at the origin:
        cur_robot.set_joint_transform(cur_robot.geom['joint']['joint_a'], np.eye(4))
        parent_mat = np.eye(4)

        # Iterate through each joint updating position:
        for joint in joints:
            xyz = copy.deepcopy(np.array(cur_robot.get_joint_rel_xyz(joint)))
            rot_axis_child = np.array(cur_robot.get_joint_axis(joint))

            theta = cur_robot.angles[joint]
            rot_theta = np.eye(4)
            trans = np.eye(4)
            ctheta = np.cos(theta)
            stheta = np.sin(theta)
            trans[0:3][:, 3] = xyz

            # Account for translation in the first joint:
            if (joint == "joint_a"):
                # Translation along the x-axis:
                rot_theta[0, 3] = 1
                continue

            elif (np.array_equal(rot_axis_child, [1, 0, 0])):
                rot_theta[1:3, 1:3] = [[ctheta, -stheta],
                                       [stheta, ctheta]]
            elif (np.array_equal(rot_axis_child, [0, 1, 0])):
                rot_theta[0, 0] = ctheta
                rot_theta[2, 0] = -1 * stheta
                rot_theta[0, 2] = stheta
                rot_theta[2, 2] = ctheta
            elif (np.array_equal(rot_axis_child, [0, 0, 1])):
                rot_theta[0:2, 0:2] = [[ctheta, -stheta],
                                       [stheta, ctheta]]
            elif (np.array_equal(rot_axis_child, [0, 0, -1])):
                rot_theta[0:2, 0:2] = [[ctheta, stheta],
                                       [-stheta, ctheta]]

            T = np.matmul(trans, rot_theta)
            global_transform = np.matmul(parent_mat, T)
            cur_robot.set_joint_transform(joint, global_transform)
            parent_mat = copy.deepcopy(global_transform)

        ef_xyz = copy.deepcopy(np.array(cur_robot.get_joint_rel_xyz('end_effector')))
        T = np.eye(4)

        T[0:3][:, 3] = ef_xyz
        global_transform = np.matmul(parent_mat, T)
        cur_robot.set_ef_xform(global_transform)

        ef_pos_world = np.matmul(global_transform, np.array([0, 0, 0, 1]))
        cur_robot.ef_pos_world = ef_pos_world[:-1]
        prev_com = np.array([0, 0, 0])
        total_mass = 0

        for joint in reversed(joints):
            joint_pos = cur_robot.get_joint_pos_world(joint)
            com_cur_link = cur_robot.get_joint_com(joint)
            cur_link_mass = cur_robot.get_joint_mass(joint)

            # calculate center of mass of current link
            # print("Current COM value for torque calculation: ", com_cur_link)
            curr_com = prev_com * total_mass + com_cur_link * cur_link_mass

            total_mass += cur_link_mass
            curr_com /= total_mass

            # calculate torque for current joint
            r = curr_com - joint_pos
            # print("R value for torque calculation: ", r)
            rot_axis_world = cur_robot.get_joint_axis_world(joint)
            torque = calculate_torque(r, total_mass, rot_axis_world)
            cur_robot.torques[joint] = torque
            prev_com = curr_com

        # print(cur_robot.torques)

        return ef_pos_world[:-1]


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
        return interp1d(x_, np.transpose(path))

    def execute_callback(self, channel, msg):
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
                target_angs = np.append(target_angs, [0, 0, 0])
                torques = [0, 0, 0]

                self.publish_config(target_angs, torques, '/sa_closedloop_cmd')

                targ = ArmPosition()
                targ.joint_a = target_angs[0]
                targ.joint_b = target_angs[1]
                targ.joint_c = target_angs[2]

                self.lcm_.publish('/arm_position', targ.encode())

                print(target_angs)
                self.spline_t += 0.01

                if self.spline_t >= 1:
                    self.enable_execute = False
            await asyncio.sleep(0.001)
        return
