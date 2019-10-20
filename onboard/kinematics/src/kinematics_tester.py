import numpy as np

import numpy.linalg as LA
import math
import time

# Main imports

# Create a function to test the operation of the theoretical arm


class KinematicsTester:
    def __init__(self, arm_state):
        self.results = []
        self.arm = arm_state
        self.xyz_pts = []
        self.test_pts = []
        self.num_radius_points_lower_limit = 3
        self.num_radius_points = 3
        self.num_phi_points = 3
        self.num_theta_points_equator = 3
        self.num_euler_points = 3
        self.point_attempts = 5
        self.lower_limit = 0.5
        self.run_tests_bool = True
        self.print_points_bool = True
        self.alpha_start = -np.pi
        self.alpha_range = 2 * np.pi
        self.beta_start = 0
        self.beta_range = np.pi
        self.gamma_start = -np.pi
        self.gamma_range = 2 * np.pi
        self.euler_angles = []
        self.successes = 0
        self.trials = 0

    def determine_euler_angles(self, num_euler_points):
        euler_angles = []

        # Assign the calculated points to each array
        alpha_inc_size = self.alpha_range / num_euler_points
        beta_inc_size = self.beta_range / num_euler_points
        gamma_inc_size = self.gamma_range / num_euler_points
        for i in range(num_euler_points):
            current_alpha = i * alpha_inc_size + self.alpha_start
            current_alpha += alpha_inc_size / 2
            for j in range(num_euler_points):
                current_beta = j * beta_inc_size + self.beta_start
                current_beta += beta_inc_size / 2
                for k in range(num_euler_points):
                    gamma = k * gamma_inc_size + self.gamma_start
                    gamma += gamma_inc_size / 2

                    euler_angles.append([current_alpha, current_beta, gamma])

        self.euler_angles = euler_angles
        return euler_angles

    def determine_xyz(self):

        # Instantiate an mrover arm
        geom = self.arm.read_geometry_from_JSON()

        joints = self.arm.state.all_joints

        joint_sum = [0, 0, 0]

        for jointidx, joint in enumerate(joints):
            joint_sum += self.arm.state.get_joint_xyz(joint)

        ef_xyz = geom['endeffector']['origin']['xyz']

        joint_sum += ef_xyz

        radius = LA.norm(joint_sum)
        radius = 0.81

        # Look at line 94 in mroverarm.py
        # Look at line 28 in mroverarm.py

        # Evenly distribute points:

        # num_radius_points_lower_limit = self.num_radius_points_lower_limit
        num_radius_points = self.num_radius_points
        num_phi_points = self.num_phi_points
        num_theta_points_equator = self.num_theta_points_equator
        num_euler_points = self.num_euler_points

        # srm = True

        euler_angles = self.determine_euler_angles(num_euler_points)

        upper_limit = radius
        print()
        print(radius)
        print("radius")
        # Set lower limit to be about 3 in.
        # LOWER LIMIT CANNOT BE ZERO -DIVIDE BY ZERO ERROR
        lower_limit = self.lower_limit
        range_radius = upper_limit - lower_limit

        radius_trials = 0

        current_radius = lower_limit
        rad_inc_size = range_radius / num_radius_points

        while(radius_trials < num_radius_points):
            current_radius = radius_trials * range_radius / num_radius_points
            current_radius += lower_limit + rad_inc_size/2
            radius_trials += 1

            current_phi = 0
            phi_trials = 0
            # un_pts = num_radius_points_lower_lim*(current_radius/lower_limit)
            num_phi_points = self.num_phi_points
            print()
            print(num_phi_points)
            print("num phi points")
            print(current_radius)
            print("current radius")
            print()

            phi_inc_size = np.pi / 2 / num_phi_points

            while(phi_trials < num_phi_points):
                current_phi = phi_trials * phi_inc_size
                current_phi += phi_inc_size/2
                phi_trials += 1

                current_theta = 0
                theta_trials = 0
                unth_pts = np.sin(current_phi) * num_theta_points_equator
                num_theta_points = round(np.abs(unth_pts))
                num_theta_points += (num_theta_points == 0)
                print()
                print(num_theta_points)
                print("num theta points")
                print(current_phi)
                print("current phi")
                print()
                theta_inc_size = 2 * np.pi / num_theta_points

                while(theta_trials < num_theta_points):
                    current_theta = theta_trials * theta_inc_size
                    current_theta += theta_inc_size/2
                    if(current_theta > 2 * np.pi):
                        current_theta = 2 * np.pi
                    theta_trials += 1

                    x = current_radius * np.sin(current_phi)
                    x *= np.cos(current_theta)
                    y = current_radius * np.sin(current_phi)
                    y *= np.sin(current_theta)
                    z = current_radius * np.cos(current_phi)

                    self.xyz_pts.append([x, y, z])

                    print()
                    print(current_theta)
                    print("current theta")
                    print()

                    for i in range(len(euler_angles)):
                        alpha = euler_angles[i][0]
                        beta = euler_angles[i][1]
                        gamma = euler_angles[i][2]

                        # Add the alpha, beta, and
                        # gamma values to the target point:
                        self.test_pts.append([x, y, z, alpha, beta, gamma])

                    #     # Run kinematics algorithm with given variables:
                    #     # def IK(self, target_point, set_random_angles):

                    #     success = False

                    #     joint_angles, success = self.arm.solver.IK(xyz, srm)

                    #     results.append(success)

                    #     print("ran_iteration")
                    #     print("current radius")
                    #     print(current_radius)
                    #     # print(joint_angles)
                    #     print("success" if success else "failure")
                    #     print(xyz)

    def run_tests(self):
        self.successes = 0
        # print(self.test_pts)
        for i in range(len(self.xyz_pts)):
            this_time = time.process_time()
            total_time = (this_time/i)*len(self.xyz_points)
            print("ETA: " + str(math.floor(total_time/3600)) + " hours, " +
                  str(math.floor((total_time % 3600)/60)) + " minutes, " +
                  str(math.floor((total_time % 3600) % 60)) + " seconds")
            print("Test number " + str(i) + " of " + str(len(self.xyz_pts)))
            self.test_point(self.xyz_pts[i])

    def test_point(self, point):
        success = False
        # Assuming we are using euler angles and not randomizing angles
        point[3:6] = [0, 0, 0]
        joint_angles, success = self.arm.solver.IK(point, False, False)

        for i in range(self.point_attempts):
            if success:
                break
            print("attempting new IK solution...")
            print(i)
            joint_angles, success = self.arm.solver.IK(point, False, False)

        if not success:
            return success

        print("position test passed")
        euler_angles = self.euler_angles
        for i in range(len(euler_angles)):
            point[3:6] = euler_angles[i]
            success = self.test_point_with_angles(point)

            self.results.append(success)
            print("ran_iteration")
            print("success" if success else "failure")
            print(point)
            print("radius: " + str(LA.norm(point[:3])))
            print()

            self.trials += 1
            if(success):
                self.successes += 1
            print("IK ran successfully "
                  + str(100 * self.successes / self.trials)
                  + "%% of the time.")
            print()

        return success

    def test_point_with_angles(self, point):
        success = False

        for i in range(self.point_attempts):
            if success:
                break
            print("attempting new IK solution...")
            print(i)
            joint_angles, success = self.arm.solver.IK(point, False, True)

        return success

    def print_points(self, isEuler):
        if(isEuler):
            for ptid, pt in enumerate(self.test_pts):
                print(pt)
        else:
            for ptid, pt in enumerate(self.xyz_pts):
                print(pt)

    def statistics(self, results):
        successes = 0
        fails = 0

        for i in range(len(results)):
            if(results[i]):
                successes += 1
            else:
                fails += 1

        if(len(results) > 0):
            print("IK ran successfully "
                  + str(100 * successes / len(results)) + "%% of the time.")

    def kinematics_test(self):
        print("running kinematics test")
        self.determine_xyz()
        if(self.print_points_bool):
            self.print_points(True)
        if(self.run_tests_bool):
            self.run_tests()
        self.statistics(self.results)
