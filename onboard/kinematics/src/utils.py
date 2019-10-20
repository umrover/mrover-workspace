import copy
import numpy as np
from numpy import linalg as LA


def apply_transformation(transform, point):
    p = copy.deepcopy(point)
    p.append(1)
    # print(transform)
    world_point = np.matmul(transform, p)
    return world_point[:3]


def unit_vector(vector):
    """
        Return unit vector
    """
    if(LA.norm(vector) == 0):
        return vector
    return vector / LA.norm(vector)


def angle_between(v1, v2):
    """
        Computes angle between two vectors
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


# distance between point and line segment
def point_line_distance(end1, end2, point):
    t = 0
    for i in range(3):
        t += (point[i] - end1[i]) * (end2[i] - end1[i])
    t = 0 if t < 0 else min(t, 1)
    closest_point = np.zeros(3)
    for i in range(3):
        closest_point[i] = end1[i] + t * (end2[i] - end1[i])
    return np.linalg.norm(closest_point - point)


# Reference : https://stackoverflow.com/
# questions/2824478/shortest-distance-between-two-line-segments
def closest_dist_bet_lines(a0, a1, b0, b1,
                           clampAll=False, clampA0=False,
                           clampA1=False, clampB0=False,
                           clampB1=False):

    ''' Given two lines defined by numpy.array pairs (a0,a1,b0,b1)
        Return the closest points on each segment and their distance
    '''

    # If clampAll=True, set all clamps to True
    if clampAll:
        clampA0 = True
        clampA1 = True
        clampB0 = True
        clampB1 = True

    # Calculate denomitator
    A = a1 - a0
    B = b1 - b0
    magA = np.linalg.norm(A)
    magB = np.linalg.norm(B)

    _A = A / magA
    _B = B / magB

    cross = np.cross(_A, _B)
    denom = np.linalg.norm(cross)**2

    # If lines are parallel (denom=0) test if lines overlap.
    # If they don't overlap then there is a closest point solution.
    # If they do overlap, there are infinite closest positions,
    # but there is a closest distance
    if not denom:
        d0 = np.dot(_A, (b0 - a0))

        # Overlap only possible with clamping
        if clampA0 or clampA1 or clampB0 or clampB1:
            d1 = np.dot(_A, (b1 - a0))

            # Is segment B before A?
            if d0 <= 0 >= d1:
                if clampA0 and clampB1:
                    if np.absolute(d0) < np.absolute(d1):
                        return np.linalg.norm(a0-b0)
                    return np.linalg.norm(a0 - b1)

            # Is segment B after A?
            elif d0 >= magA <= d1:
                if clampA1 and clampB0:
                    if np.absolute(d0) < np.absolute(d1):
                        return np.linalg.norm(a1-b0)
                    return np.linalg.norm(a1 - b1)

        # Segments overlap, return distance between parallel segments
        return np.linalg.norm(((d0*_A) + a0) - b0)

    # Lines criss-cross: Calculate the projected closest points
    t = (b0 - a0)
    detA = np.linalg.det([t, _B, cross])
    detB = np.linalg.det([t, _A, cross])

    t0 = detA / denom
    t1 = detB / denom

    pA = a0 + (_A * t0)  # Projected closest point on segment A
    pB = b0 + (_B * t1)  # Projected closest point on segment B

    # Clamp projections
    if clampA0 or clampA1 or clampB0 or clampB1:
        if clampA0 and t0 < 0:
            pA = a0
        elif clampA1 and t0 > magA:
            pA = a1

        if clampB0 and t1 < 0:
            pB = b0
        elif clampB1 and t1 > magB:
            pB = b1

        # Clamp projection A
        if (clampA0 and t0 < 0) or (clampA1 and t0 > magA):
            dot = np.dot(_B, (pA - b0))
            if clampB0 and dot < 0:
                dot = 0
            elif clampB1 and dot > magB:
                dot = magB
            pB = b0 + (_B * dot)

        # Clamp projection B
        if (clampB0 and t1 < 0) or (clampB1 and t1 > magB):
            dot = np.dot(_A, (pB - a0))
            if clampA0 and dot < 0:
                dot = 0
            elif clampA1 and dot > magA:
                dot = magA
            pA = a0 + (_A * dot)

    return np.linalg.norm(pA - pB)


def compute_euler_angles(rot_mat):

    ''' Returns a vector of euler angles based on
        a given rotation matrix. The euler angles
        are in the form ZXZ '''

    # print("we have never tested this")
    # print(rot_mat)

    # R_one_three = rot_mat[0][2]
    # R_two_three = rot_mat[1][2]
    # R_three_one = rot_mat[2][0]
    # R_three_two = rot_mat[2][1]
    # R_three_three = rot_mat[2][2]
    # print(rot_mat)

    alpha = np.arctan2(rot_mat[0][2], -rot_mat[1][2])  # range -pi to pi
    # print(alpha)
    beta = np.arccos(rot_mat[2][2])  # range 0 to pi
    # print(beta)
    gamma = np.arctan2(rot_mat[2][0], rot_mat[2][1])  # range -pi to pi
    # print(gamma)

    angles = np.array([alpha, beta, gamma])

    # print("angles")
    # print(angles)

    return angles


def create_rotation_xform(change_basis, theta):
    # http://scipp.ucsc.edu/~haber/ph216/rotation_12.pdf
    # 2d rotation matrix to 3d rotation matrix about an axis of rotation

    # print(theta)

    # rot_mat = np.matrix([
    #             [1, -axis_rot[2]*theta, axis_rot[1] * theta]
    #             [axis_rot[2]*theta, 1, -axis_rot[0] * theta]
    #             [-axis_rot[2]*theta, axis_rot[0] * theta, 1]
    #             ])

    rot_mat = np.zeros((3, 3))

    rot_mat[0][0] = np.cos(theta)
    rot_mat[0][1] = -np.sin(theta)
    rot_mat[0][2] = 0
    rot_mat[1][0] = np.sin(theta)
    rot_mat[1][1] = np.cos(theta)
    rot_mat[1][2] = 0
    rot_mat[2][0] = 0
    rot_mat[2][1] = 0
    rot_mat[2][2] = 1

    # u_vec = np.norm(start_vec)
    # a_vec = np.norm(axis_rot)
    # ortho_vec = np.norm(np.cross(u_vec, a_vec))

    # change_basis[:, 0] = np.transpose(u_vec)
    # change_basis[:, 1] = np.transpose(a_vec)
    # change_basis[:, 2] = np.transpose(ortho_vec)
    # print("does inverse exist")
    # print(LA.pinv(change_basis))
    # print(np.matmul(LA.inv(change_basis), change_basis))
    std_basis = np.matmul(rot_mat, LA.pinv(change_basis))

    # print("blah")

    xform = np.matmul(change_basis, std_basis)

    # rot_mat[0][0] = 1
    # rot_mat[0][1] = -axis_rot[2]*theta
    # rot_mat[0][2] = axis_rot[1] * theta
    # rot_mat[1][0] = axis_rot[2]*theta
    # rot_mat[1][1] = 1
    # rot_mat[1][2] = -axis_rot[0] * theta
    # rot_mat[2][0] = -axis_rot[2]*theta
    # rot_mat[2][1] = axis_rot[0] * theta
    # rot_mat[2][2] = 1

    # print(rot_mat)

    return xform


def rot_xform_ypr(yaw, pitch, roll):
    # converts yaw pitch and roll (z, x, y) into rotation matrix
    c_one = np.cos(yaw)
    c_two = np.cos(pitch)
    c_three = np.cos(roll)
    s_one = np.sin(yaw)
    s_two = np.sin(pitch)
    s_three = np.sin(roll)

    rot_mat = np.zeros((3, 3))

    # https://en.wikipedia.org/wiki/Euler_angles

    rot_mat[0][0] = c_one * c_three - s_one * s_two * s_three
    rot_mat[0][1] = -(c_two * s_one)
    rot_mat[0][2] = c_one * s_three + c_three * s_one * s_two
    rot_mat[1][0] = c_three * s_one + c_one * s_two * s_three
    rot_mat[1][1] = c_one * c_two
    rot_mat[1][2] = s_one * s_three - c_one * c_three * s_two
    rot_mat[2][0] = - (c_two * s_three)
    rot_mat[2][1] = s_two
    rot_mat[2][2] = c_two * c_three

    print(rot_mat)

    return rot_mat


def ypr_to_euler_angs(yaw, pitch, roll):
    rot_mat = rot_xform_ypr(yaw, pitch, roll)
    return compute_euler_angles(rot_mat)


def degrees_to_radians(degrees):
    return degrees * np.pi / 180


def radians_to_degrees(radians):
    return radians * 180 / np.pi


def calculate_midpoint(joint1, joint2):
    return (joint2+joint1)/2


def calculate_COM(joint1, joint2, percent_length):
    return percent_length * (joint2 - joint1) + joint1


def calculate_torque(r, mass, rot_axis):
    g = -9.807
    force = np.array([0, 0, g * mass])
    torque = np.cross(r, force)
    torque_component = np.dot(torque, rot_axis)
    return torque_component
