import json
import numpy as np
import time
from collections import OrderedDict
from .utils import apply_transformation


class SAState:
    def __init__(self, geom, lcm):
        super(SAState, self).__init__()
        self.geom = self.read_geometry_from_JSON(geom)
        self.angles = OrderedDict()
        self.lcm_ = lcm
        self.angle_time = 0
        self.prev_angles = OrderedDict()
        self.prev_angle_time = 0
        self.z_limit = 0
        self.ef_pos_world = np.array([0, 0, 0])
        self.ef_xform = np.eye(4)
        self.coms = np.array(np.size(self.all_joints))
        self.torques = OrderedDict()

        for joint in self.all_joints:
            self.angles[joint] = 0.0
            self.torques[joint] = 0.0

    @property
    def all_joints(self):
        return list(self.geom['joints'].keys())

    def read_geometry_from_JSON(self, file):
        geom_file = file['geom_file']

        with open(geom_file) as f:
            geom = json.load(f, object_pairs_hook=OrderedDict)

        return geom

    def set_joint_transform(self, joint, transform):
        self.geom['joints'][joint]['matrix'] = transform

    def get_joint_transform(self, joint):
        return self.geom['joints'][joint]['matrix']

    def get_child(self, joint):
        return self.geom['joints'][joint]['child']

    def get_deposit_pos(self):
        xyz = self.geom['deposit']
        return [xyz['angle_0'], xyz['angle_1'], xyz['angle_2']]

    def get_joint_com(self, joint):
        transform = self.get_joint_transform(joint)
        translation = np.eye(4)
        x = self.geom['joints'][joint]['COM']['x']
        y = self.geom['joints'][joint]['COM']['y']
        z = self.geom['joints'][joint]['COM']['z']

        translation[0:3][:, 3] = np.array([x, y, z])
        output = np.matmul(transform, translation)
        return output[0:3][:, 3]

    def get_joint_mass(self, joint):
        return self.geom['joints'][joint]['mass']

    def get_joint_axis(self, joint):
        x = self.geom['joints'][joint]['rot_axis']["x"]
        y = self.geom['joints'][joint]['rot_axis']["y"]
        z = self.geom['joints'][joint]['rot_axis']["z"]
        return [x, y, z]

    def get_joint_rel_xyz(self, joint):
        x = self.geom['joints'][joint]['origin']['x']
        y = self.geom['joints'][joint]['origin']['y']
        z = self.geom['joints'][joint]['origin']['z']
        return [x, y, z]

    def get_joint_pos_world(self, joint):
        transform_matrix = self.get_joint_transform(joint)
        # 4 x 4 homogeneous
        x = transform_matrix[0][3]
        y = transform_matrix[1][3]
        z = transform_matrix[2][3]
        return np.array([x, y, z])

    def get_ef_pos_world(self):
        transform_matrix = self.ef_xform
        # 4 x 4 homogeneous
        x = transform_matrix[0][3]
        y = transform_matrix[1][3]
        z = transform_matrix[2][3]
        return np.array([x, y, z])

    def get_coms(self):
            return self.coms

    def set_coms(self, center_of_masses):
        self.coms = center_of_masses

    def get_torques(self):
        return [self.torques['joint_a'],
                self.torques['joint_b'], self.torques['joint_c']]

    def set_ef_xform(self, xform):
        self.ef_xform = xform

    def set_angles(self, arm_position):
        self.angles['joint_a'] = arm_position[0]
        self.angles['joint_b'] = arm_position[1]
        self.angles['joint_c'] = arm_position[2]
        print(arm_position)

    def get_angles(self):
        return [self.angles['joint_a'],
                self.angles['joint_b'],
                self.angles['joint_c']]

    def set_angles_list(self, arm_position):
        # self.prev_angles = copy.deepcopy(self.angles)
        # self.prev_angle_time = copy.deepcopy(self.angle_time)
        self.angles['joint_a'] = arm_position[0]
        self.angles['joint_b'] = arm_position[1]
        self.angles['joint_c'] = arm_position[2]
        # TODO: add time tracking
        self.angle_time = time.time()

    def get_joint_axis_world(self, joint):
        xform = self.get_joint_transform(joint)
        axis_local = self.get_joint_axis(joint)
        return apply_transformation(xform, axis_local)
