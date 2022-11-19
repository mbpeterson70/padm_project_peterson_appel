import numpy as np
from scipy.spatial import KDTree
from time import sleep

from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, interval_generator, set_joint_positions, \
    link_from_name, get_link_pose
from src.utils import are_confs_close

class MotionPlanner():
    '''
    Creates a plan to move gripper from some initial configuration to a goal pose.
    '''

    def __init__(self, rrt_edge_len, world, tol=1e-3):
        self.edge_len = rrt_edge_len
        self.world = world
        lower_limits, upper_limits = \
            get_custom_limits(self.world.robot, self.world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)
        self.generator = generator = interval_generator(lower_limits, upper_limits)
        self.tool_link = link_from_name(self.world.robot, 'panda_hand')
        self.tol = tol

    def execute_motion_plan(self, plan):
        for conf in plan:
            set_joint_positions(self.world.robot, self.world.arm_joints, conf)
            sleep(.1)

    def motion_plan_rrt(self, conf_start, goal_pose):
        '''
        RRT algorithm to move from start configuration (series of joint angles) to 
        goal pose (gripper position and orientation.
        '''
        vertices = {conf_start}
        edges = dict()
        
        while True:
            conf_rand = self.get_random_configuration()
            conf_nearest, conf_nearest_dist = self.nearest_conf(vertices, conf_rand)
            if conf_nearest_dist > self.edge_len:
                conf_new = np.array(conf_nearest) + \
                    (np.array(conf_rand)-np.array(conf_nearest)) * self.edge_len / conf_nearest_dist
                conf_new = tuple(conf_new.tolist())
            else:
                conf_new = conf_rand
            pose_new = self.get_conf_gripper_pose(conf_new, conf_start)
            # TODO: Check for collision here!!
            if True:
                vertices.add(conf_new)
                edges[conf_new] = conf_nearest
                if self.close_to_goal(pose_new, goal_pose):
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
    
    def get_conf_gripper_pose(self, conf, orig_conf):
        '''Get the pose of the gripper for an arbitrary configuration.'''
        set_joint_positions(self.world.robot, self.world.arm_joints, conf)
        pose = get_link_pose(self.world.robot, self.tool_link)
        # reset back to original
        set_joint_positions(self.world.robot, self.world.arm_joints, orig_conf) 
        return pose

    def close_to_goal(self, pose, goal_pose):
        '''Checks if a pose is within some tolerance of the goal pose.'''
        # TODO: Add check for orientation here
        p = np.array(pose[0])
        goal_p = np.array(goal_pose[0])
        if np.linalg.norm(p-goal_p) < .3:
            print(np.linalg.norm(p-goal_p))
        return np.linalg.norm(p-goal_p) < self.tol

    def path_to_vertex(self, conf_start, conf_end, edges):
        '''Returns the total path from start configuration to end configuration.'''
        c = edges[conf_end]
        path = [conf_end, c]
        while c != conf_start:
            c = edges[c]
            path.append(c)
        path.reverse()
        return path

