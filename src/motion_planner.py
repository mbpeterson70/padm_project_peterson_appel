import numpy as np
from scipy.spatial import KDTree
from time import sleep
from copy import deepcopy

from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, interval_generator, set_joint_positions, \
    link_from_name, get_link_pose, get_joint_positions, set_pose
from src.utils import are_confs_close, set_tool_pose, get_tool_from_root
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import closest_inverse_kinematics
# get_tool_from_root
# tool is the link gripper attached
class MotionPlanner():
    '''
    Creates a plan to move gripper from some initial configuration to a goal pose.
    '''

    def __init__(self, rrt_edge_len=.2, rrt_goal_biasing=5, world=None, tol=1e-9):
        self.edge_len = rrt_edge_len
        self.goal_biasing = rrt_goal_biasing
        self.world = world
        lower_limits, upper_limits = \
            get_custom_limits(self.world.robot, self.world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)
        self.generator = generator = interval_generator(lower_limits, upper_limits)
        self.tool_link = link_from_name(self.world.robot, 'panda_hand')
        self.tol = tol

    def execute_motion_plan(self, plan):
        for conf in plan:
            set_joint_positions(self.world.robot, self.world.arm_joints, conf)
            sleep(.01)

    def motion_plan_rrt(self, conf_start, goal_pose):
        '''
        RRT algorithm to move from start configuration (series of joint angles) to 
        goal pose (gripper position and orientation.
        '''
        goal_conf = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, goal_pose, max_time=0.05), None)
        vertices = {conf_start}
        edges = dict()

        for i in range(int(1e20)):
            if i % self.goal_biasing == 0:
                conf_rand = tuple(goal_conf)
            else:
                conf_rand = self.get_random_configuration()
            conf_nearest, conf_nearest_dist = self.nearest_conf(vertices, conf_rand)
            if conf_nearest_dist > self.edge_len:
                conf_new = np.array(conf_nearest) + \
                    (np.array(conf_rand)-np.array(conf_nearest)) * self.edge_len / conf_nearest_dist
                conf_new = tuple(conf_new.tolist())
            else:
                conf_new = conf_rand
            print(conf_new)
            # TODO: Check for collision here!!
            if True:
                vertices.add(conf_new)
                edges[conf_new] = conf_nearest
                if np.linalg.norm(np.array(conf_new) - np.array(goal_conf)) < self.tol:
                    solution_path = self.path_to_vertex(conf_start, conf_new, edges)
                    return solution_path

    def get_random_configuration(self):
        '''Samples a random arm configuration (series of angles).'''
        return tuple(next(self.generator))

    def nearest_conf(self, vertices, conf_rand):
        '''Finds the neares configuration to the input random configuration.'''
        vertices_list = list(vertices)
        T = KDTree(np.array(vertices_list))
        closest_dist, closest_idx = T.query(np.array(conf_rand))
        closest = vertices_list[closest_idx]
        return closest, closest_dist

    def path_to_vertex(self, conf_start, conf_end, edges):
        '''Returns the total path from start configuration to end configuration.'''
        c = edges[conf_end]
        path = [conf_end, c]
        while c != conf_start:
            c = edges[c]
            path.append(c)
        path.reverse()
        return path

