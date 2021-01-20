import numpy as np
from numpy import linalg as LA
import random
import copy
from .utils import apply_transformation
from .utils import compute_euler_angles
from.utils import calculate_torque
# from .utils import degrees_to_radians
# from .logger import logger


class KinematicsSolver:
    def __init__(self, robot_state, lcm):
        super(KinematicsSolver, self).__init__()
        self.robot_state = robot_state
        # robot_ik is only updated in IK!!!!!!!!!
        self.robot_ik = copy.deepcopy(robot_state)
        self.new_state = copy.deepcopy(robot_state)
        self.robot_safety = copy.deepcopy(robot_state)
        self.lcm_ = lcm
        self.MAX_ITERATIONS = 500
        self.THRESHOLD = 0.01
        self.ANGLE_THRESHOLD = 10  # degrees_to_radians(88)
        self.POS_WEIGHT = 1
        # try robot_ik
        self.FK(self.robot_state)
        print("a-b")
        print(robot_state.get_link_transform("a-b"))
        print("b-c")
        print(robot_state.get_link_transform("b-c"))
        print("c-d")
        print(robot_state.get_link_transform("c-d"))
        print("d-e")
        print(robot_state.get_link_transform("d-e"))
        print("e-f")
        print(robot_state.get_link_transform("e-f"))
        print("hand")
        print(robot_state.get_link_transform("hand"))
        self.target_pos_world = np.array([0, 0, 0])
        self.e_locked = False

        self.j_kp = 0.1
        self.j_kd = 0

        self.delta_theta = 0.0001

    def FK(self, cur_robot):
        '''
            Forward kinematics using custom convention
            # Returns:
                Transforms for all joints (published on LCM)
        '''
        joints = cur_robot.all_joints
        # links = self.robot_state.all_links

        global_transform = np.eye(4)
        # print(global_transform)
        # rot_axis_parent = np.array([0, 0, 1])

        cur_robot.set_link_transform(cur_robot.geom['base'], np.eye(4))
        parent_mat = np.eye(4)

        # Iterate through all joints, starting at joint_a
        for joint in joints:

            xyz = copy.deepcopy(np.array(cur_robot.get_joint_xyz(joint)))
            print("xyz: ")
            print(xyz)
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

        ef_xyz = copy.deepcopy(np.array(cur_robot.get_ef_xyz()))
        T = np.eye(4)

        T[0:3][:, 3] = ef_xyz
        global_transform = np.matmul(parent_mat, T)
        cur_robot.set_ef_xform(global_transform)

        # ef_pos = copy.deepcopy(cur_robot.get_ef_pos_world())
        # ef_pos.append(1)
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

    def IK(self, target_point, set_random_angles, use_euler_angles):
        '''
            Inverse kinematics for MRover arm using cyclic
            coordinate descent (CCD)
            Params:
                target_point (np.array([x, y, z, alpha, beta, gamma])):
                target point in 3d space
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
        print("RUNNING IK")

        print("Target Point: ", target_point)

        num_iterations = 0
        self.target_pos_world = target_point[:3]
        self.target_ang_world = target_point[3:]
        # print(self.target_ang_world)
        self.FK(self.robot_state)
        print("FK RAN")
        self.robot_ik = copy.deepcopy(self.robot_state)
        links = self.robot_ik.all_links
        joints = self.robot_ik.all_joints
        # print("joints: ")
        # print(joints)
        # print("links: ")
        # print(links)

        if (set_random_angles):
            # set random joint angles (should be used to deal with
            #   local minima)
            # print("setting random angles on robot_ik!")
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
        ef_vec_world = self.robot_ik.get_world_point_angles(links[-1])
        ef_pos_world = self.robot_ik.get_ef_pos_world()
        ef_ang_world = ef_vec_world[3:]

        dist = LA.norm(ef_pos_world - self.target_pos_world)
        angle_dist = LA.norm(ef_ang_world - self.target_ang_world)

        ef_v = 0
        print("Current EF Position: ", ef_vec_world)
        print("Current Joint Angles: ", self.robot_ik.angles)

        # print(dist)
        # print(angle_dist)

        # print("out of the loop")
        while dist > self.THRESHOLD or angle_dist > self.ANGLE_THRESHOLD:
            # print("in the loop")
            if (num_iterations > self.MAX_ITERATIONS):
                print("Max ik iterations hit")
                print("position reached" + str(ef_vec_world))
                print("target point" + str(target_point))
                return (self.robot_ik.angles, False)

            # print("here")
            # print(target_point)
            ef_to_target_b_weights = target_point - ef_vec_world
            ef_to_tar_xyz = ef_to_target_b_weights[:3] * self.POS_WEIGHT
            ef_to_tar_ang = ef_to_target_b_weights[3:]
            ef_to_target_vec_world = np.append(ef_to_tar_xyz, ef_to_tar_ang)

            # print("made target vec world")

            # d_ef is the vector we want the ef to move in this step
            # running on a PD controller!
            d_ef = self.j_kp * ef_to_target_b_weights\
                - self.j_kd * (ef_v * (ef_to_target_vec_world /
                                       LA.norm(ef_to_target_vec_world)))
            # print("made d_ef")
            # print("About to Run IK Step")
            self.IK_step(d_ef, True, use_euler_angles)
            # print("Ran IK Step")

            # iterate
            ef_vec_world = self.robot_ik.get_world_point_angles(links[-1])
            print("ef_vec_world: ")
            print(ef_vec_world)
            # print(ef_vec_world)
            ef_pos_world = self.robot_ik.get_ef_pos_world()
            ef_ang_world = ef_vec_world[3:]

            dist = LA.norm(ef_pos_world - self.target_pos_world)
            angle_dist = LA.norm(ef_ang_world - self.target_ang_world)

            # print("Euler Angles | Alpha: ")
            # print(ef_pos_world[3])

            num_iterations += 1

            if(num_iterations % 100000 == 0):
                print("euler angles")
                print(ef_ang_world)
                print("target euler angles")
                print(self.target_ang_world)
                print("x y z")
                print(ef_pos_world)
                print("target x y z")
                print(self.target_pos_world)

                print()

                total_dist = LA.norm(ef_vec_world - target_point)
                print(total_dist)

                print()

                print(dist)

                print()

                print(angle_dist)

                print()

        # print("Angle and angle goal: ")
        # print(ef_ang_world)
        # print(self.target_ang_world)

        # print()

        # print("Position and position goal:")

        # print(ef_pos_world)
        # print(self.target_pos_world)

        # print()
        print("AHH, safe checking")
        if not self.safe(self.robot_ik.get_angles()):

            print("ik not safe")
            # print(ef_vec_world)
            return (self.robot_ik.angles, False)
        print("ik safe about to return")
        # num_safe_configs += 1
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

    def IK_step(self, d_ef, use_pi, use_euler_angles):
        # print("IK STEP")
        links = self.robot_ik.all_links
        joints = self.robot_ik.all_joints
        ef_world = self.robot_ik.get_world_point_angles(links[-1])
        ef_pos_world = self.robot_ik.get_ef_pos_world()
        ef_euler_world = ef_world[3:]

        # compute jacobian
        jacobian = np.zeros((6, 6))

        for joint_idx, joint in enumerate(joints):
            # print(joint)
            if self.e_locked and joint == 'joint_e':
                jacobian[:, joint_idx] = [0, 0, 0, 0, 0, 0]
            else:
                rot_axis_local = self.robot_ik.get_joint_axis(joint)
                joint_xform = self.robot_ik.get_joint_transform(joint)
                joint_pos_world = self.robot_ik.get_world_point(
                                    self.robot_ik.get_child(joint))
                # print("child: ")
                # print(self.robot_ik.get_child(joint))

                rot_axis_world = apply_transformation(joint_xform,
                                                      rot_axis_local)

                joint_to_ef_vec_world = ef_pos_world - joint_pos_world

                # single column contribution to Jacobian
                joint_col_xyz = np.transpose(np.cross(rot_axis_world,
                                                      joint_to_ef_vec_world))

                joint_col = []

                if(use_euler_angles):
                    n_xform = self.apply_joint_xform(self.robot_ik,
                                                     joint, self.delta_theta)

                    euler_angles = compute_euler_angles(n_xform)

                    diff_angs = (euler_angles - ef_euler_world)
                    delta_angs = diff_angs / self.delta_theta

                    joint_col_xyz = np.transpose(joint_col_xyz)

                    joint_col_euler = np.transpose(delta_angs)

                    joint_col = np.append(joint_col_xyz, joint_col_euler)
                else:
                    joint_col = np.append(joint_col_xyz, [0, 0, 0])

                jacobian[:, joint_idx] = joint_col

        # compute pseudoinverse or transpose
        if use_pi:
            jacobian_inverse = LA.pinv(jacobian)
            # print(jacobian_inverse)
        else:
            jacobian_inverse = np.transpose(jacobian)

        # compute changes to dofs
        d_theta = np.matmul(jacobian_inverse, d_ef)

        # print(jacobian)

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

    def apply_joint_xform(self, cur_robot, joint, theta):
        xyz = copy.deepcopy(np.array(cur_robot.get_joint_xyz(joint)))
        rot_axis_child = np.array(cur_robot.get_joint_axis(joint))

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

        parent_mat = cur_robot.get_ef_transform()
        T = np.matmul(trans, rot_theta)
        global_transform = np.matmul(parent_mat, T)
        return global_transform
