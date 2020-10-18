import numpy as np
import numpy.linalg as LA
# from datetime import datetime
from rover_msgs import (SAClosedLoopCmd, SAPosData, SimulationMode)
import asyncio
from .logger import logger
from scipy.interpolate import interp1d
import copy
from.utils import calculate_torque


class SAKinematics:
    def __init__(self, lcm, arm):
        self.lcm_ = lcm
        self.arm_ = arm
        self.spline_t = 0
        self.current_spline = []
        self.enable_execute = False
        self.sim_mode = True

    def FK(self):
        cur_robot = self.arm_
        joints = cur_robot.all_joints
        global_transform = np.eye(4)
        # set the base to be at the origin:
        cur_robot.set_joint_transform('joint_a', np.eye(4))
        parent_mat = np.eye(4)

        prevasdf = [0, 0, 0]

        # Iterate through each joint updating position:
        for joint in joints:
            if joint == 'end_effector':
                break

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
                rot_theta[0, 3] = theta

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

            print(joint, cur_robot.get_joint_pos_world(joint) - prevasdf)
            prevasdf = cur_robot.get_joint_pos_world(joint)

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
            if joint == 'end_effector':
                break
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

        print(ef_pos_world)

        return ef_pos_world[:-1]

    def plan_return_to_deposit(self, cur_pos):
        translator = cur_pos[0]
        joint_b = cur_pos[1]
        joint_c = cur_pos[2]
        xyz_target = self.arm_.get_deposit_pos()
        path = [[translator, joint_b, joint_c]]
        path.append([xyz_target[0], joint_b, joint_c])
        path.append(xyz_target)

        cs = self.spline_fitting(path)
        print(path)
        # print(cs)

        self.current_spline = cs

        return cs

    def set_angles(self, angles):
        self.arm_.set_angles_list(angles)

    def spline_fitting(self, path):
        step = len(path)
        x_ = np.linspace(0, 1, step)
        return interp1d(x_, np.transpose(path))

    def arm_position_callback(self, channel, msg):
        arm_pos = SAPosData.decode(msg)

        self.arm_.set_angles(arm_pos)
        self.FK()

    def simulation_mode_callback(self, channel, msg):
        simulation_mode_msg = SimulationMode.decode(msg)
        self.sim_mode = simulation_mode_msg.sim_mode

    def execute_callback(self, channel, msg):
        # pos = self.arm_.get_ef_pos_world()
        print("return to deposit callback")
        pos = self.arm_.get_angles()
        self.spline_t = 0
        self.current_spline = self.plan_return_to_deposit(pos)
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
            # start = datetime.now()
            if self.enable_execute:

                logger.info('spline time: {}'.format(self.spline_t))

                target_angs = self.current_spline(self.spline_t)

                cur_angs = self.arm_.get_angles()
                ang_dist = LA.norm(np.array(target_angs - cur_angs))

                torques = [0, 0, 0]  # add in real torques later
                torques = self.arm_.get_torques()

                self.publish_config(target_angs, torques, '/sa_closedloop_cmd')

                print(target_angs)

                if not self.sim_mode:
                    self.spline_t += min(0.01, 0.0005/ang_dist)
                elif self.sim_mode:
                    self.spline_t += 0.01

                self.spline_t = min(self.spline_t, 1)

                if self.spline_t >= 1 and (ang_dist < 0.07 or self.sim_mode):
                    self.enable_execute = False
            await asyncio.sleep(0.001)
            # print(datetime.now() - start)
        return
