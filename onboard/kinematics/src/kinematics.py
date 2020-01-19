import numpy as np
from numpy import linalg as LA
import copy
from .utils import apply_transformation
import random
# from .logger import logger


class KinematicsSolver:
    def __init__(self, robot_state, lcm):
        super(KinematicsSolver, self).__init__()
        self.robot_state = robot_state
        # robot_ik is only updated in IK!!!!!!!!!
        self.robot_ik = copy.deepcopy(robot_state)
        self.robot_safety = copy.deepcopy(robot_state)
        self.lcm_ = lcm
        self.MAX_ITERATIONS = 500
        self.THRESHOLD = 0.001
        # try robot_ik
        self.FK(self.robot_state)
        self.target_pos_world = np.array([0, 0, 0])
        self.e_locked = True

        self.j_kp = 0.9
        self.j_kd = 0.0

    def FK(self, cur_robot):
        '''
            Forward kinematics using custom convention

            # Returns:
                Transforms for all joints (published on LCM)
        '''
        joints = cur_robot.all_joints
        # links = self.robot_state.all_links

        global_transform = np.eye(4)
        # rot_axis_parent = np.array([0, 0, 1])

        cur_robot.set_link_transform(cur_robot.geom['base'], np.eye(4))
        parent_mat = np.eye(4)

        # Iterate through all joints, starting at joint_a
        for joint in joints:

            xyz = copy.deepcopy(np.array(cur_robot.get_joint_xyz(joint)))
            rot_axis_child = np.array(cur_robot.get_joint_axis(joint))

            # set theta: retrieve joint angle!
            theta = cur_robot.angles[joint]

            rot_theta = np.eye(4)
            trans = np.eye(4)

            ctheta = np.cos(theta)
            stheta = np.sin(theta)

            trans[0:3][:, 3] = xyz

            # make rotation about rot axis by theta
            if (np.array_equal(rot_axis_child, [1, 0, 0])):
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

            link = cur_robot.get_child(joint)

            # set joint and corresponding link global transform
            cur_robot.set_joint_transform(joint, global_transform)
            cur_robot.set_link_transform(link, global_transform)

            # set vector and mat for next iteration
            # rot_axis_parent = copy.deepcopy(rot_axis_child)
            parent_mat = copy.deepcopy(global_transform)

        ef_pos = copy.deepcopy(cur_robot.get_joint_xyz(joints[-1]))
        ef_pos.append(1)
        ef_pos_world = np.matmul(global_transform, np.array([0, 0, 0, 1]))

        cur_robot.ef_pos_world = ef_pos_world[:-1]
        return ef_pos_world[:-1]

    def IK(self, target_orientation, set_random_angles):
        '''
            Inverse kinematics for MRover arm using cyclic
            coordinate descent (CCD)

            Params:
                target_point (np.array([x, y, z])): target point in 3d space
                    for end effector
                set_random_angles: asks solver to set joint angles to random
                    angles. Used to escape local minima

            Returns:
                joint_angles (list)
                bool : whether the robot arm was able to find
                    joint angles that allow it to reach within a threshold
                    of the target location

            Reference:
                http://www.cs.cmu.edu/~15464-s13/assignments/assignment2/jlander_gamedev_nov98.pdf
                http://www.cs.cmu.edu/~15464-s13/lectures/lecture6/IK.pdf

        '''
        num_iterations = 0
        self.target_pos_world = target_orientation[:3]
        self.FK(self.robot_state)
        self.robot_ik = copy.deepcopy(self.robot_state)
        links = self.robot_ik.all_links
        joints = self.robot_ik.all_joints

        if (set_random_angles):
            # set random joint angles (should be used to deal with
            #   local minima)
            print("setting random angles on robot_ik!")
            rand_a = (random.random() - 0.5) * 2 * np.pi
            rand_b = random.random() * .25 * np.pi
            rand_c = (random.random() - 0.5) * np.pi
            rand_d = (random.random() - 0.5) * np.pi
            rand_e = (random.random() - 0.5) * np.pi
            rand_f = (random.random() - 0.5) * 2 * np.pi
            rand_angs = np.array(
                                [rand_a,
                                 rand_b,
                                 rand_c,
                                 rand_d,
                                 rand_e,
                                 rand_f])

            j_idx = 0
            for joint in joints:
                self.robot_ik.angles[joint] = rand_angs[j_idx]
            # update transforms using FK
            self.FK(self.robot_ik)

        # Position of the end effector
        ef_pos_world = self.robot_ik.get_world_point(links[-1])

        dist = LA.norm(ef_pos_world - self.target_pos_world)

        ef_v = 0

        while dist > self.THRESHOLD:
            if (num_iterations > self.MAX_ITERATIONS):
                print("Max ik iterations hit")
                return (self.robot_ik.angles, False)

            ef_to_target_vec_world = self.target_pos_world - ef_pos_world

            # d_ef is the vector we want the ef to move in this step
            # running on a PD controller!
            d_ef = self.j_kp * ef_to_target_vec_world\
                - self.j_kd * (ef_v * (ef_to_target_vec_world /
                                       LA.norm(ef_to_target_vec_world)))

            self.IK_step(d_ef, False)

            # iterate
            ef_pos_world = self.robot_ik.get_world_point(links[-1])
            dist = LA.norm(ef_pos_world - self.target_pos_world)
            num_iterations += 1

        if not self.safe(self.robot_ik.get_angles()):

            print("ik not safe")

            return (self.robot_ik.angles, False)
        print("ik safe about to return")
        return (self.robot_ik.angles, True)

    def IK_delta(self, delta, iterations):
        self.FK(self.robot_state)
        self.robot_ik = copy.deepcopy(self.robot_state)
        links = self.robot_ik.all_links

        start_pos = self.robot_ik.get_world_point(links[-1])
        target_pos = start_pos + delta
        if LA.norm(target_pos) > 0.82:
            return self.robot_ik.angles, True

        for _ in range(iterations):
            original_pos = self.robot_ik.get_world_point(links[-1])
            self.IK_step(delta, True)
            new_pos = self.robot_ik.get_world_point(links[-1])
            true_delta = np.subtract(new_pos, original_pos)
            delta = np.subtract(delta, true_delta)

        is_safe = self.safe(self.robot_ik.get_angles())
        return self.robot_ik.angles, is_safe

    def IK_step(self, d_ef, use_pi):
        links = self.robot_ik.all_links
        joints = self.robot_ik.all_joints
        ef_pos_world = self.robot_ik.get_world_point(links[-1])

        # compute jacobian
        jacobian = np.zeros((3, 6))

        for joint_idx, joint in enumerate(joints):
            if self.e_locked and joint == 'joint_e':
                jacobian[:, joint_idx] = [0, 0, 0]
            else:
                rot_axis_local = self.robot_ik.get_joint_axis(joint)
                joint_xform = self.robot_ik.get_joint_transform(joint)
                joint_pos_world = self.robot_ik.get_world_point(
                                    self.robot_ik.get_child(joint))

                rot_axis_world = apply_transformation(joint_xform,
                                                      rot_axis_local)

                joint_to_ef_vec_world = ef_pos_world - joint_pos_world
                # single column contribution to Jacobian
                joint_col = np.transpose(np.cross(rot_axis_world,
                                                  joint_to_ef_vec_world))

                jacobian[:, joint_idx] = joint_col

        # compute pseudoinverse or transpose
        if use_pi:
            jacobian_inverse = LA.pinv(jacobian)
        else:
            jacobian_inverse = np.transpose(jacobian)

        # compute changes to dofs
        d_theta = np.matmul(jacobian_inverse, d_ef)

        # apply changes to dofs

        for joint_idx, joint in enumerate(joints):
            angle = self.robot_ik.angles[joint] + d_theta[joint_idx]

            limits = self.robot_safety.get_joint_limit(joint)
            angle = np.clip(angle, limits['lower'], limits['upper'])

            self.robot_ik.angles[joint] = angle

        self.FK(self.robot_ik)

    def safe(self, angles):
        limit = self.limit_check(angles)
        joints = self.robot_ik.all_joints
        for joint_idx, joint in enumerate(joints):
            if joint != 'joint_f':
                self.robot_safety.angles[joint] = angles[joint_idx]
        self.FK(self.robot_safety)

        safety = self.robot_safety.obstacle_free()

        return safety and limit

    def limit_check(self, angles):
        joints = self.robot_safety.all_joints[:-1]
        for j_idx, joint in enumerate(joints[1:], 1):
            limits = self.robot_safety.get_joint_limit(joint)
            if (not (limits['lower'] <= angles[j_idx] <= limits['upper'])):
                return False
        return True

    def lock_joint_e(self, lock):
        self.e_locked = lock
