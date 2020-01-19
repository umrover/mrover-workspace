import json
import copy
from collections import OrderedDict
from rover_msgs import (FKTransform, ArmPosition, DebugMessage,
                        TargetOrientation, TargetAngles, MotionExecute,
                        SimulationMode, IkArmControl, LockJointE,
                        IkEnabled)
import numpy as np
from numpy import linalg as LA
import asyncio
import time
from .logger import logger
from .arm_state import ArmState
from .kinematics import KinematicsSolver
from .motion_planner import MotionPlanner


class MRoverArm:
    def __init__(self, config, lcm):
        self.config = config

        geom = self.read_geometry_from_JSON()

        # shared global variable between different modules
        self.lcm_ = lcm
        self.state = ArmState(geom)

        self.solver = KinematicsSolver(self.state, lcm)
        self.motion_planner = MotionPlanner(self.state, lcm, self.solver)

        self.current_spline = []
        self.spline_t = 0
        self.done_previewing = False
        self.enable_execute = False
        self.sim_mode = True
        self.ik_enabled = False

    def read_geometry_from_JSON(self):
        geom_file = self.config['geom_file']

        with open(geom_file) as f:
            geom = json.load(f, object_pairs_hook=OrderedDict)

        return geom

    def arm_position_callback(self, channel, msg):
        """
          Handler for angles
          Triggers forward kinematics
        """
        if self.ik_enabled and not self.enable_execute:
            return

        arm_position = ArmPosition.decode(msg)
        if channel == "/arm_position":
            self.state.set_angles(arm_position)
            self.solver.FK(self.state)
            self.publish_transforms(self.state)

    def publish_config(self, config, channel):
        arm_position = ArmPosition()
        arm_position.joint_a = config[0]
        arm_position.joint_b = config[1]
        arm_position.joint_c = config[2]
        arm_position.joint_d = config[3]
        arm_position.joint_e = config[4]

        self.lcm_.publish(channel, arm_position.encode())

    def publish_transforms(self, state):
        tm = FKTransform()
        tm.transform_a = state.get_joint_transform('joint_a')
        tm.transform_b = state.get_joint_transform('joint_b')
        tm.transform_c = state.get_joint_transform('joint_c')
        tm.transform_d = state.get_joint_transform('joint_d')
        tm.transform_e = state.get_joint_transform('joint_e')
        tm.transform_f = state.get_joint_transform('joint_f')
        self.lcm_.publish('/fk_transform', tm.encode())
        return None

    def target_orientation_callback(self, channel, msg):
        point_msg = TargetOrientation.decode(msg)
        self.enable_execute = False
        logger.info('Got a target point.')

        point = np.array([point_msg.x, point_msg.y, point_msg.z])
        print('Point: {}'.format(point))
        print(self.state.angles)
        success = False
        joint_angles, success = self.solver.IK(point, False)

        ik_message = DebugMessage()
        ik_message.isError = False

        for i in range(5):
            if success:
                ik_message.message = "Solved IK"
                break
            print("attempting new IK solution...")
            print(i)
            joint_angles, success = self.solver.IK(point, True)

        if not success:
            ik_message.message = "No IK solution"
            logger.warning('No IK solution found...')
            print("NO IK SOLUTION FOUND, using closest configuration...")

        self.lcm_.publish('/debugMessage', ik_message.encode())
        if not success:
            return

        self.publish_transforms(self.state)
        goal = [joint_angles["joint_a"],
                joint_angles["joint_b"],
                joint_angles["joint_c"],
                joint_angles["joint_d"],
                joint_angles["joint_e"],
                joint_angles["joint_f"]]
        self.plan_path(goal)

    def target_angles_callback(self, channel, msg):
        self.enable_execute = False
        target_angles = TargetAngles.decode(msg)
        goal = [target_angles.a,
                target_angles.b,
                target_angles.c,
                target_angles.d,
                target_angles.e,
                target_angles.f]

        self.plan_path(goal)

    def plan_path(self, goal):
        logger.info('Goal: {}'.format(goal))
        print("goal")
        print(goal)
        print("start")
        print(self.state.angles)

        path_message = DebugMessage()
        path_message.isError = False

        path_spline = self.motion_planner.rrt_connect(goal)
        if path_spline:
            self.current_spline = path_spline
            self.spline_t = 0
            print("planned path")
            path_message.message = 'Planned path'
        else:
            print("No path found.")
            path_message.message = 'No path found'

        self.lcm_.publish('/debugMessage', path_message.encode())

    def motion_execute_callback(self, channel, msg):
        motion_execute_msg = MotionExecute.decode(msg)

        preview = motion_execute_msg.preview

        if preview:
            self.enable_execute = False
            self.preview()
        else:
            self.enable_execute = True

    def simulation_mode_callback(self, channel, msg):
        simulation_mode_msg = SimulationMode.decode(msg)
        self.sim_mode = simulation_mode_msg.sim_mode
        print(self.sim_mode)
        self.publish_transforms(self.state)

    def cartesian_control_callback(self, channel, msg):
        if self.enable_execute:
            return

        cart_msg = IkArmControl.decode(msg)
        delta = [cart_msg.deltaX, cart_msg.deltaY, cart_msg.deltaZ]

        joint_angles, is_safe = self.solver.IK_delta(delta, 3)

        if is_safe:
            arm_position = ArmPosition()
            arm_position.joint_a = joint_angles["joint_a"]
            arm_position.joint_b = joint_angles["joint_b"]
            arm_position.joint_c = joint_angles["joint_c"]
            arm_position.joint_d = joint_angles["joint_d"]
            arm_position.joint_e = -joint_angles["joint_e"]
            self.state.set_angles(arm_position)
            self.solver.FK(self.state)
            self.publish_transforms(self.state)
            if self.sim_mode:
                self.lcm_.publish('/arm_position', arm_position.encode())
            else:
                self.lcm_.publish('/ik_ra_control', arm_position.encode())

    async def execute_spline(self):
        logger.info('Executing path on arm')
        self.spline_t = 0
        while True:
            if self.enable_execute:
                # run spline
                logger.info('spline time: {}'.format(self.spline_t))
                target_c = self.current_spline(self.spline_t)
                # target_v = self.current_spline(self.spline_t, 1)
                # target_a = self.current_spline(self.spline_t, 2)
                cur_c = self.state.get_angles()[:-1]
                c_dist = LA.norm(np.array(target_c - cur_c))
                print(c_dist)

                target_c[-1] *= -1
                if not self.sim_mode:
                    self.publish_config(target_c, '/ik_ra_control')
                    self.spline_t += min(0.0005/c_dist, 0.01)
                # TODO: make sure transition from not self.sim_mode
                #   to self.sim_mode is safe!!
                elif self.sim_mode:
                    time.sleep(0.0003)
                    self.publish_config(target_c, '/arm_position')
                    self.spline_t += 0.01

                self.spline_t = min(self.spline_t, 1)
                if self.spline_t >= 1 and c_dist < 0.07:
                    self.enable_execute = False
            await asyncio.sleep(0.001)
        return

    def preview(self):
        logger.info('Previewing')
        preview_robot = copy.deepcopy(self.state)
        num_steps = 500
        t = 0
        print(self.current_spline(1))
        while t < 1:
            target = self.current_spline(t)
            target_pos = ArmPosition()
            target_pos.joint_a = target[0]
            target_pos.joint_b = target[1]
            target_pos.joint_c = target[2]
            target_pos.joint_d = target[3]
            target_pos.joint_e = -target[4]
            print(target)
            preview_robot.set_angles(target_pos)

            self.solver.FK(preview_robot)
            print(preview_robot.angles)

            self.publish_transforms(preview_robot)
            print(t)
            time.sleep(0.002)
            t += 1/num_steps
        path_message = DebugMessage()
        path_message.isError = False
        path_message.message = 'Preview Done'
        self.lcm_.publish('/debugMessage', path_message.encode())

    def lock_e_callback(self, channel, msg):
        lock_msg = LockJointE.decode(msg)
        self.solver.lock_joint_e(lock_msg.locked)

    def ik_enabled_callback(self, channel, msg):
        enabled_msg = IkEnabled.decode(msg)
        self.ik_enabled = enabled_msg.enabled
