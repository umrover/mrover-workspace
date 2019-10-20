import numpy as np
from numpy import linalg as LA
import math
import random
from scipy.interpolate import CubicSpline
from .logger import logger
# from .kinematics import KinematicsSolver
# import copy
# import time


class Node:
    '''
    Node class for tree
    '''
    def __init__(self, config):
        super(Node, self).__init__()

        self.config = config
        self.parent = []
        self.children = set()
        self.cost = 0


class MotionPlanner:
    def __init__(self, robot_state, lcm, solver):
        super(MotionPlanner, self).__init__()
        self.lcm_ = lcm
        self.robot = robot_state
        self.solver = solver
        # print(self.robot.all_joints)
        self.all_limits = []
        for joint in self.robot.all_joints:
            limit = self.robot.get_joint_limit(joint)
            limit['lower'] = math.degrees(limit['lower'])
            limit['upper'] = math.degrees(limit['upper'])
            self.all_limits.append(limit)
        self.all_limits.pop()

        self.step_limits = [1, 1, 2, 3, 5]  # in degrees
        self.neighbor_dist = 3
        self.max_iterations = 1000
        self.i = 0

    def sample(self):
        '''
            Generate a random config based on the joint limits
        '''
        z_rand = []
        for limit in self.all_limits:
            z_rand.append(random.uniform(limit['lower'], limit['upper']))

        return np.array(z_rand)

    def nearest(self, tree_root, rand):
        '''
            Find nearest node in tree to a given random node in config space
        '''

        q = [tree_root]
        min_dist = float('inf')
        min_node = None

        while q:
            node = q.pop(0)

            dist = LA.norm(node.config - rand)

            if dist < min_dist:
                min_dist = dist
                min_node = node

            for child in node.children:
                q.append(child)

        return min_node

    def near(self, z_new):
        '''
        find neighbors of rand
        '''
        q = [self.root]
        neighbors = []
        while q:
            node = q.pop(0)
            dist = LA.norm(node.config - z_new)
            if dist < self.neighbor_dist:
                neighbors.append(node)
            for child in node.children:
                q.append(child)
        return neighbors

    def steer(self, start, end):

        line_vec = end - start.config

        if min(np.subtract(self.step_limits, abs(line_vec))) >= 0:
            return end

        new_config = np.array(start.config)

        min_t = float('inf')
        # parameterize the line
        for i in range(len(line_vec)):
            t = self.step_limits[i] * np.sign(line_vec[i]) / line_vec[i]
            if t < min_t:
                min_t = t

        for i in range(len(line_vec)):
            new_config[i] += min_t * line_vec[i]

        return new_config

    # shortest path optimazation for rrt*
    def choose_parent(self, z_near, z_nearest, z_new):
        '''
        best parent is one with least cost + distance to rand
        '''
        best_parent = z_nearest
        best_cost = z_nearest.cost + LA.norm(z_nearest.config - z_new)
        for current_node in z_near:
            current_cost = current_node.cost
            current_cost += LA.norm(current_node.config - z_new)
            if current_cost < best_cost:
                best_parent = current_node
                best_cost = current_cost
        '''
        hook new node up with the chosen parent
        '''
        new_node = Node(z_new)
        new_node.parent = best_parent
        best_parent.children.add(new_node)
        new_node.cost = best_cost
        self.x.append(z_new[0])
        self.y.append(z_new[1])
        return new_node

    # shortest path optimazation for rrt*
    def rewire(self, z_near, z_new):
        for node in z_near:
            new_cost = z_new.cost + LA.norm(node.config - z_new.config)
            if new_cost < node.cost:
                node.cost = new_cost
                node.parent.children.remove(node)
                node.parent = z_new
                z_new.children.add(node)
                # Note: may need to propogate diminished cost to children

    def backtrace_path(self, end, root):
        path = []
        node = end
        while node != root:
            config = node.config
            config = [math.radians(angle) for angle in config]
            path.append(config)
            node = node.parent
        config = root.config
        config = [math.radians(angle) for angle in config]
        path.append(config)
        return path

    def extend(self, tree, z_rand):
        # print(self.i)
        self.i += 1

        z_nearest = self.nearest(tree, z_rand)
        z_new = self.steer(z_nearest, z_rand)

        # blocked by obstacle/self collision
        # print("checking safety")
        if not self.solver.safe(np.radians(z_new)):
            # print("not safe")
            return Node(None)
        # print("safe")

        new_node = Node(z_new)
        new_node.parent = z_nearest
        z_nearest.children.add(new_node)
        new_node.cost = z_nearest.cost + LA.norm(z_nearest.config - z_new)
        return new_node

    def connect(self, tree, a_new):
        extension = self.extend(tree, a_new)
        config = extension.config
        while config is not None and not np.array_equal(config, a_new):
            extension = self.extend(tree, a_new)
            config = extension.config
        return extension

    def rrt_connect(self, target):
        start = [self.robot.angles["joint_a"],
                 self.robot.angles["joint_b"],
                 self.robot.angles["joint_c"],
                 self.robot.angles["joint_d"],
                 self.robot.angles["joint_e"]]
        start = [math.degrees(float(angle)) for angle in start]
        print("start root")
        print(start)
        self.start_root = Node(np.array(start))
        target = [math.degrees(float(angle)) for angle in target][:-1]
        self.goal_root = Node(np.array(target))

        for i in range(self.max_iterations):
            a_root = self.start_root if i % 2 else self.goal_root
            b_root = self.goal_root if i % 2 else self.start_root
            z_rand = self.sample()

            a_new = self.extend(a_root, z_rand)
            if a_new.config is not None:
                b_new = self.connect(b_root, a_new.config)
                # are the trees connected?
                if np.array_equal(a_new.config, b_new.config):
                    a_path = self.backtrace_path(a_new, a_root)
                    b_path = self.backtrace_path(b_new, b_root)
                    path = a_path
                    path.reverse()
                    middle = [math.radians(angle) for angle in a_new.config]
                    path.append(middle)
                    path.extend(b_path)
                    if not i % 2:
                        path.reverse()

                    cs = self.spline_fitting(path)
                    # return path
                    return cs

        # no path found
        logger.info('NO PATH FOUND!!')
        return []

    def spline_fitting(self, path):
        x_ = np.linspace(0, 1, len(path))
        cs = CubicSpline(x_, path)

        return cs
