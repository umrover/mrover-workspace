import numpy as np
import csv
from .utils import degrees_to_radians


class ConfigurationSpaceTest:
    def __init__(self, arm_state):
        self.arm = arm_state
        self.angle_precision = 30
        self.angle_limit = 270
        self.num_valid_points = 0
        self.num_config_attempts = 0
        self.point_attempts = 5
        self.pos_successes = 0
        self.orient_successes = 0
        self.trials = 0
        self.iterations = int(self.angle_limit / self.angle_precision) + 1
        self.filename = "valid_configurations_2.csv"
        self.vws = np.array([0, 0, 0, 0, 0, 0])

    def write_file(self):
        print("Determining safe configurations stored at: " + self.filename)
        file = open(self.filename, 'w')
        csvwriter = csv.writer(file)
        lim_a = self.arm.state.get_joint_limit('joint_a')
        lim_b = self.arm.state.get_joint_limit('joint_b')
        lim_c = self.arm.state.get_joint_limit('joint_c')
        lim_d = self.arm.state.get_joint_limit('joint_d')
        lim_e = self.arm.state.get_joint_limit('joint_e')
        lim_f = self.arm.state.get_joint_limit('joint_f')
        a = lim_a['lower']
        b = lim_b['lower']
        c = lim_c['lower']
        d = lim_d['lower']
        e = lim_e['lower']
        f = lim_f['lower']
        print()
        print(b)
        print(lim_b['upper'])
        print()
        while a <= lim_a['upper']:
            a += self.angle_precision

            while b <= lim_b['upper']:
                b += self.angle_precision

                while c <= lim_c['upper']:
                    c += self.angle_precision

                    while d <= lim_d['upper']:
                        d += self.angle_precision

                        while e <= lim_e['upper']:
                            e += self.angle_precision
                            angles = [a, b, c, d, e, f]
                            self.write_angles(angles, csvwriter)
                        e = lim_e['lower']
                    d = lim_d['lower']
                c = lim_c['lower']
            b = lim_b['lower']
        print("Arm covers " +
              str(100 * self.num_valid_points / self.num_config_attempts)
              + "% of global space")

    def write_angles(self, angles, writer):
        # print(angles)
        for joint_idx, joint in enumerate(self.arm.state.all_joints):
            angle = angles[joint_idx]

            limits = self.arm.state.get_joint_limit(joint)
            angle = np.clip(angle, limits['lower'], limits['upper'])

            angle = degrees_to_radians(angle)
            self.arm.state.angles[joint] = angle

        self.arm.solver.FK(self.arm.state)
        self.num_config_attempts += 1
        if self.arm.state.obstacle_free():
            world_point = self.arm.state.get_world_point_angles("sid the kid")
            writer.writerow(world_point)
            self.num_valid_points += 1

        # if(self.arm.solver.safe(self.arm.state.get_angles())):

    def read_file(self):
        file = open(self.filename, 'r')
        csvFile = csv.reader(file)
        for row in csvFile:
            self.read_line(row)
        self.run_tests()

    def read_line(self, target_point):
        # print(target_point)
        x = float(target_point[0])
        y = float(target_point[1])
        z = float(target_point[2])
        alpha = float(target_point[3])
        beta = float(target_point[4])
        gamma = float(target_point[5])
        input_point = np.array([x, y, z, alpha, beta, gamma])
        # print("line 101")
        # print(input_point)
        self.vws = np.vstack((self.vws, input_point))
        # print(self.vws)

    def run_tests(self):
        print(self.vws[:20])
        arr = np.array([1, 2, 3, 4])
        np.random.shuffle(arr)
        print(arr)
        np.random.shuffle(self.vws)
        for world_point in self.vws[:500]:
            print(world_point)
            self.test_point(world_point)
            self.trials += 1
            print("Trial " + str(self.trials) +
                  " out of " + str(self.num_valid_points))
            print("IK position ran successfully "
                  + str(100 * self.pos_successes / self.trials)
                  + "%% of the time.")
            if(self.pos_successes != 0):
                print("Out of these points, IK orientation ran successfully "
                      + str(100 * self.orient_successes / self.pos_successes)
                      + "%% of the time.")

    def test_point(self, point):
        pos_success = False
        print()
        print("attempting new IK solution...")
        for i in range(self.point_attempts):
            print("attempts for this point: " + str(i))
            print()
            j_angles, pos_success = self.arm.solver.IK(point, True, False)
            if pos_success:
                break

        if not pos_success:
            print("could not pass position test")
            print()
            return
        else:
            self.pos_successes += 1
        print()

        orient_success = False

        j_angles, orient_success = self.arm.solver.IK(point, False, True)
        print("attempting orientation test")
        for i in range(self.point_attempts):
            print()
            if orient_success:
                break
            j_angles, orient_success = self.arm.solver.IK(point, True, True)

        if not orient_success:
            print("could not pass orientation test")
            print()
        else:
            self.orient_successes += 1
        print()

    def straight_up_torque_test(self):
        for joint in self.arm.state.all_joints:
            self.arm.state.angles[joint] = 0
            if (joint == 'joint_c'):
                self.arm.state.angles[joint] = np.pi / 2
            if (joint == 'joint_b'):
                self.arm.state.angles[joint] = np.pi / 2

        self.arm.solver.FK(self.arm.state)
        for joint in self.arm.state.all_joints:
            print(joint, " torque: ", self.arm.state.torques[joint])

        self.arm.publish_transforms(self.arm.state)
