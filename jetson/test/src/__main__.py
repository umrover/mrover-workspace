import lcm
from rover_msgs import ArmPosition


lcm_ = lcm.LCM()
start = [0, 0, 0, 0, 0, 0]
angles = [0, 0, 0, 0, 0, 0]

global state  # forward


def print_list(lis):
    print("angles: ", lis[0], lis[1], lis[2], lis[3], lis[4], lis[5])


def record_angle_callback(channel, msg):
    joints = ArmPosition.decode(msg)
    angles[0] = joints.joint_a
    angles[1] = joints.joint_b
    angles[2] = joints.joint_c
    angles[3] = joints.joint_d
    angles[4] = joints.joint_e
    angles[5] = joints.joint_f

    print("initial angles")
    print_list(angles)


def publish_angles(angles_list, arm_pos):
    angles_list[0] = arm_pos.joint_a
    angles_list[1] = arm_pos.joint_b
    angles_list[2] = arm_pos.joint_c
    angles_list[3] = arm_pos.joint_d
    angles_list[4] = arm_pos.joint_e
    angles_list[5] = arm_pos.joint_f
    print("at angles")
    print_list(angles)
    lcm_.publish("/")


def main():

    lcm_.subscribe("/arm_position", record_angle_callback)
    lcm_.handle()
    state = 0

    for i in range(0, 6):
        start[i] = angles[i]

    while(1):
        arm = ArmPosition()

        for joint in range(0, len(angles)):
            target = start[joint] + 1.0

            while(1):
                lcm_.handle()

                if (abs(target - angles[joint]) > 0.01 if joint != 1 else abs(target - angles[joint]) > 0.05):
                    if (state == 0):
                        state = 1
                    else:
                        state = 0
                        break

                if (state == 0):
                    print("moving forward")
                    # move 60 degrees
                    angles[joint] = target
                else:
                    # move back
                    print("moving backward")
                    target = start[joint] - 1.0
                    angles[joint] = target

                publish_angles(angles, arm)


if __name__ == '__main__':
    main()
