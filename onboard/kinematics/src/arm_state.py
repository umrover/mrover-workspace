import numpy as np
import numpy.linalg as LA
import copy
import time
from collections import OrderedDict
from .utils import apply_transformation, closest_dist_bet_lines
from .utils import point_line_distance
from .utils import compute_euler_angles


class ArmState:
    '''

    Maintains all information about arm:
        geom : static information about the arm geometry
        angle : joint angles -- always changing during operation

    '''
    def __init__(self, geom):
        super(ArmState, self).__init__()
        self.geom = geom
        self.angles = OrderedDict()
        self.angle_time = 0
        self.prev_angles = OrderedDict()
        self.prev_angle_time = 0
        self.z_limit = 0
        self.ef_pos_world = np.array([0, 0, 0])
        self.ef_xform = np.eye(4)
        self.coms = np.array(np.size(self.all_joints))
        self.torques = OrderedDict()
        self.num_total_collision_parts = 23
        # self.f_to_ef = self.geom['endeffector']['origin']['xyz'] -
        #                             self.get_world_point('joint_f')

        for joint in self.all_joints:
            self.angles[joint] = 0.0
            self.torques[joint] = 0.0

        self.angles['joint_b'] = 1.0
        self.angles['joint_c'] = 1.0

        self._parts = ['CHASSIS-A SPHERE', 'A-B LINK', 'A-B CAPSULE 1',
                       'A-B CAPSULE 2', 'B-C LINK', 'B-C SPHERE 1',
                       'B-C CAPSULE 1', 'B-C CAPSULE 2', 'B-C CAPSULE 3',
                       'B-C CAPSULE 4', 'B-C CAPSULE 4', 'B-C CAPSULE 5',
                       'C-D LINK', 'C-D CAPSULE 1', 'C-D CAPSULE 2',
                       'D-E LINK', 'D-E CAPSULE 1', 'D-E CAPSULE 2',
                       'D-E CAPSULE 3', 'D-E SPHERE 1', 'E-F LINK',
                       'E-F CAPSULE 1', 'HAND LINK', 'HAND CAPSULE 1',
                       'HAND CAPSULE 2', 'HAND SPHERE 1', 'HAND CAPSULE 3',
                       'HAND CAPSULE 4', 'HAND CAPSULE 5', 'HAND CAPSULE 6']

        self._collision_mat = np.zeros((self.num_total_collision_parts,
                                        self.num_total_collision_parts))

        # Collision matrix for which links need
        # to be checked against each other

        # See teleop documentation for collision data:
        # Collision data chassis-sphere:
        self._collision_mat[0, 9:] = 1
        # A-B link:
        self._collision_mat[1:3, 9:] = 1
        # B-C link:
        self._collision_mat[3:9, 11:] = 1
        # C-D link:
        self._collision_mat[9:11, 15:] = 1
        # D-E link:
        self._collision_mat[11:15, 16:23] = 1

        coll_tp = np.transpose(self._collision_mat)
        self._collision_mat = coll_tp + self._collision_mat

    @property
    def base(self):
        return self.geom['base']

    @property
    def all_joints(self):
        return list(self.geom['joints'].keys())

    @property
    def all_links(self):
        return list(self.geom['links'].keys())

    @property
    def all_parts(self):
        return self._parts

    @property
    def collision_mat(self):
        return self._collision_mat

    def get_parent(self, joint):
        return self.geom['joints'][joint]['parent']

    def get_child(self, joint):
        return self.geom['joints'][joint]['child']

    def get_joint_type(self, joint):
        return self.geom['joints'][joint]['type']

    def get_joint_com(self, joint):
        transform = self.get_joint_transform(joint)
        translation = np.eye(4)
        x = self.geom['joints'][joint]['mass_data']['com']['x']
        y = self.geom['joints'][joint]['mass_data']['com']['y']
        z = self.geom['joints'][joint]['mass_data']['com']['z']

        translation[0:3][:, 3] = np.array([x, y, z])
        # print("Joint Com Calculation: ")
        # print()
        # print()
        # print("translation:")
        # print(translation)
        # print()
        # print("transformation:")
        # print(transform)
        # print()
        # print("output:")
        output = np.matmul(transform, translation)
        # print(output)
        # print()
        # print("joint xyz:")
        # print(self.get_joint_pos_world(joint))
        # print()
        # print()
        return output[0:3][:, 3]

    def get_joint_mass(self, joint):
        return self.geom['joints'][joint]['mass_data']['mass']

    def get_joint_axis(self, joint):
        return self.geom['joints'][joint]['axis']

    def get_joint_axis_world(self, joint):
        xform = self.get_joint_transform(joint)
        axis_local = self.get_joint_axis(joint)
        return apply_transformation(xform, axis_local)

    def get_joint_xyz(self, joint):
        return self.geom['joints'][joint]['origin']['xyz']

    def get_ef_xyz(self):
        return self.geom['endeffector']['origin']['xyz']

    def get_joint_rpy(self, joint):
        return self.geom['joints'][joint]['origin']['rpy']

    def get_joint_limit(self, joint):
        return self.geom['joints'][joint]['limit']

    def get_link_origin(self, link):
        return self.geom['links'][link]['origin']

    def get_link_rpy(self, link):
        return self.geom['links'][link]['rpy']

    def set_link_transform(self, link, transform):
        self.geom['links'][link]['matrix'] = transform

    def set_joint_transform(self, joint, transform):
        self.geom['joints'][joint]['matrix'] = transform

    def get_link_transform(self, link):
        return self.geom['links'][link]['matrix']

    def get_joint_transform(self, joint):
        return self.geom['joints'][joint]['matrix']

    def get_ef_transform(self):
        return self.ef_xform

    def get_link_shape(self, link, shape_num):
        shape = "shape_" + str(shape_num + 1)
        return self.geom['links'][link]['link_shapes'][shape]

    def get_link_joint_origin(self, link):
        return self.geom['links'][link]['visual']['origin']['joint_origin']

    def get_num_shapes(self, link):
        return self.geom['links'][link]['link_shapes']['shapes']

    def get_actuator_shape(self, link):
        return self.geom['links'][link]['actuator_shape']

    def get_link_radius(self, link):
        return self.get_actuator_shape(link)["radius"]

    def clicks_to_radians(self, joint, clicks):
        return 0

    def get_joint_pos_world(self, joint):
        transform_matrix = self.get_joint_transform(joint)
        # 4 x 4 homogeneous
        x = transform_matrix[0][3]
        y = transform_matrix[1][3]
        z = transform_matrix[2][3]
        return np.array([x, y, z])

    # def get_jf_to_ef(self){
    #     return self.f_to_ef
    # }

    def get_ef_pos_world(self):
        transform_matrix = self.ef_xform
        # 4 x 4 homogeneous
        x = transform_matrix[0][3]
        y = transform_matrix[1][3]
        z = transform_matrix[2][3]
        return np.array([x, y, z])

    # def set_end_effector_pos(self, xyz):
    #     self.ef_pos_world = xyz

    # def get_ef_xform(self):
    #     return self.ef_xform

    def set_ef_xform(self, xform):
        self.ef_xform = xform

    def get_world_point(self, link):
        transform_matrix = self.geom['links'][link]['matrix']
        # 4 x 4 homogeneous
        x = transform_matrix[0][3]
        y = transform_matrix[1][3]
        z = transform_matrix[2][3]
        return np.array([x, y, z])

    def get_world_point_angles(self, link):
        transform_matrix = self.ef_xform
        # 4 x 4 homogeneous
        x = transform_matrix[0][3]
        y = transform_matrix[1][3]
        z = transform_matrix[2][3]

        rotation_matrix = transform_matrix[:3, :3]

        euler_angles = compute_euler_angles(rotation_matrix)

        pos_array = np.array([x, y, z])

        # x, y, z, alpha, beta, gamma
        return np.append(pos_array, euler_angles)

    def get_coms(self):
        return self.coms

    def set_coms(self, center_of_masses):
        self.coms = center_of_masses

    def set_link_rpy(self, link, r, p, y):
        self.geom['links'][link]['rpy'] = {r, p, y}

    def set_angles(self, arm_position):
        self.prev_angles = copy.deepcopy(self.angles)
        self.prev_angle_time = copy.deepcopy(self.angle_time)
        self.angles['joint_a'] = arm_position.joint_a
        self.angles['joint_b'] = arm_position.joint_b
        self.angles['joint_c'] = arm_position.joint_c
        self.angles['joint_d'] = arm_position.joint_d
        self.angles['joint_e'] = -arm_position.joint_e
        # TODO: add time tracking
        self.angle_time = time.time()

    def get_angles(self):
        return [self.angles['joint_a'],
                self.angles['joint_b'],
                self.angles['joint_c'],
                self.angles['joint_d'],
                self.angles['joint_e'],
                self.angles['joint_f']]

    def get_prev_angles(self):
        return [self.prev_angles['joint_a'],
                self.prev_angles['joint_b'],
                self.prev_angles['joint_c'],
                self.prev_angles['joint_d'],
                self.prev_angles['joint_e'],
                self.prev_angles['joint_f']]

    def set_angles_list(self, arm_position):
        self.prev_angles = copy.deepcopy(self.angles)
        self.prev_angle_time = copy.deepcopy(self.angle_time)
        self.angles['joint_a'] = arm_position[0]
        self.angles['joint_b'] = arm_position[1]
        self.angles['joint_c'] = arm_position[2]
        self.angles['joint_d'] = arm_position[3]
        self.angles['joint_e'] = arm_position[4]
        # TODO: add time tracking
        self.angle_time = time.time()

    def capsule_zcheck(self, endpoint1, endpoint2):
        return endpoint1[2] > self.z_limit and endpoint2[2] > self.z_limit

    def sphere_zcheck(self, center, radius):
        return (center[2] - radius) > self.z_limit

    def capsule_capsule_check(self, link_1, link_2, part_1, part_2):
        '''
            Checks two capsules to see if they intersect each other
            Args:
                link_1 (str)
                link_2 (str)
                part_1 (str): ACT or LINK
                part_2 (str): ACT or LINK

            Returns:
                bool: do they collide?
        '''
        if part_1 == 'ACT':
            capsule_1 = self.get_actuator_shape(link_1)
        else:
            capsule_1 = self.get_link_shape(link_1)

        if part_2 == 'ACT':
            capsule_2 = self.get_actuator_shape(link_2)
        else:
            capsule_2 = self.get_link_shape(link_2)

        transform_1 = self.get_link_transform(link_1)
        transform_2 = self.get_link_transform(link_2)

        # Get endpoints for each capsule
        c1_link_p1 = list(capsule_1['point_1'].values())
        c1_link_p2 = list(capsule_1['point_2'].values())

        c2_link_p1 = list(capsule_2['point_1'].values())
        c2_link_p2 = list(capsule_2['point_2'].values())

        # Transform endpoints into world frame
        c1_link_p1 = apply_transformation(transform_1, c1_link_p1)
        c1_link_p2 = apply_transformation(transform_1, c1_link_p2)

        c2_link_p1 = apply_transformation(transform_2, c2_link_p1)
        c2_link_p2 = apply_transformation(transform_2, c2_link_p2)

        # note you might still have issues depending on angle of arm

        """
        if not self.capsule_zcheck(c1_link_p1, c1_link_p2):
            return True
        if not self.capsule_zcheck(c2_link_p1, c2_link_p2):
            return True
        """
        c1_rad = capsule_1['radius']
        c2_rad = capsule_2['radius']

        closest_dist = closest_dist_bet_lines(c1_link_p1,
                                              c1_link_p2, c2_link_p1,
                                              c2_link_p2, clampAll=True)

        return closest_dist < (c1_rad + c2_rad)

    def sphere_sphere_check(self, link_1, link_2):
        '''
            Checks if sphere and capsule intersect one another
            Args:
                link_1 (str)
                link_2 (str)
            Returns:
                bool: do they collide?
        '''
        sphere_1 = self.get_link_shape(link_1)
        sphere_2 = self.get_link_shape(link_2)

        if link_1 == 'hand':
            link_1 = 'e-f'
        if link_2 == 'hand':
            link_2 = 'e-f'
        transform_1 = self.get_link_transform(link_1)
        transform_2 = self.get_link_transform(link_2)

        sphere_1_c = list(sphere_1['center'].values())
        sphere_2_c = list(sphere_2['center'].values())

        sphere_1_c = apply_transformation(transform_1, sphere_1_c)
        sphere_2_c = apply_transformation(transform_2, sphere_2_c)

        sphere_1_rad = sphere_1['radius']
        sphere_2_rad = sphere_2['radius']

        """"
        if not self.sphere_zcheck(sphere_1_c, sphere_1_rad):
            return True
        if not self.sphere_zcheck(sphere_2_c, sphere_2_rad):
            return True
        """
        closest_dist = np.linalg.norm(np.array(sphere_1_c)
                                      - np.array(sphere_2_c))

        return closest_dist < sphere_1_rad + sphere_2_rad

    # def sphere_capsule_check(self, link_1, link_2, part_1, part_2):
    #     '''
    #         Checks if sphere and capsule intersect one another
    #         Args:
    #             link_1 (str)
    #             link_2 (str)
    #             part_1 (str): SPHERE
    #             part_2 (str): ACT OR LINK

    #         Returns:
    #             bool: do they collide?
    #     '''
    #     sphere = self.get_link_shape(link_1)

    #     if part_2 == 'ACT':
    #         capsule = self.get_actuator_shape(link_2)
    #     else:
    #         capsule = self.get_link_shape(link_2)

    #     transform_1 = self.get_link_transform(link_1)
    #     transform_2 = self.get_link_transform(link_2)

    #     sphere_c = list(sphere['center'].values())
    #     cp1 = list(capsule['point_1'].values())
    #     cp2 = list(capsule['point_2'].values())

    #     sphere_c = apply_transformation(transform_1, sphere_c)
    #     cp1 = apply_transformation(transform_2, cp1)
    #     cp2 = apply_transformation(transform_2, cp2)

    #     c_rad = capsule['radius']
    #     sphere_rad = sphere['radius']
    #     """
    #     if not self.capsule_zcheck(cp1, cp2):
    #         return True
    #     if not self.sphere_zcheck(sphere_c, sphere_rad):
    #         return True
    #     """
    #     eps = 0.0001
    #     temp = sphere_c + np.array([eps, eps, eps])
    #     closest_dist = closest_dist_bet_lines(cp1, cp2,
    #                                           sphere_c, temp, clampAll=True)

    #     return closest_dist < c_rad + sphere_rad

    def transform_parts(self):
        transformed_parts = []
        for link in self.all_links:
            # print(link)

            joint_origin = self.get_link_joint_origin(link)

            # print(joint_origin)

            transform = self.get_joint_transform(joint_origin)

            # print(transform)

            num_collision_parts = int(self.get_num_shapes(link))

            # print(num_collision_parts)

            for i in range(num_collision_parts):
                # print("looping")
                shape = self.get_link_shape(link, i)
                # print(shape)

                transformed_part = {'type': shape['type']}

                if (shape['type'] == 'capsule'):
                    cp1 = list(shape['point_1'].values())
                    cp2 = list(shape['point_2'].values())
                    cp1 = apply_transformation(transform, cp1)
                    cp2 = apply_transformation(transform, cp2)
                    transformed_part['points'] = [cp1, cp2]
                elif (shape['type'] == 'sphere'):
                    center = list(shape['center'].values())
                    center = apply_transformation(transform, center)
                    transformed_part['center'] = center
                transformed_part['radius'] = shape['radius']
                transformed_parts.append(transformed_part)
        return transformed_parts

    def obstacle_free(self):
        # get a list of all the transformed parts
        # print("or before?")
        transformed_parts = self.transform_parts()
        # print("does it fail after transformed parts?")

        # check all parts against each other if they need to be checked
        for i in range(1, len(transformed_parts) - 1):
            for j in range(i + 1, len(transformed_parts) - 1):
                if self.collision_mat[i][j]:
                    if self.link_link_check(transformed_parts[i],
                                            transformed_parts[j]):
                        return False

        return True

    def link_link_check(self, part1, part2):
        if (part1['type'] == 'capsule' and part2['type'] == 'capsule'):
            end_points1 = part1['points']
            end_points2 = part2['points']
            closest_dist = closest_dist_bet_lines(end_points1[0],
                                                  end_points1[1],
                                                  end_points2[0],
                                                  end_points2[1],
                                                  clampAll=True)
            return closest_dist < part1['radius'] + part2['radius']

        elif (part1['type'] == 'capsule' or part2['type'] == 'capsule'):
            if (part1['type'] == 'capsule'):
                end_points1 = part1['points']
                closest_dist = point_line_distance(end_points1[0],
                                                   end_points1[1],
                                                   part2['center'])
                return closest_dist < part1['radius'] + part2['radius']
            else:
                end_points1 = part2['points']
                closest_dist = point_line_distance(end_points1[0],
                                                   end_points1[1],
                                                   part1['center'])
                return closest_dist < part1['radius'] + part2['radius']
        else:
            closest_dist = LA.norm(part1['center'] - part2['center'])
            return closest_dist < part1['radius'] + part2['radius']
