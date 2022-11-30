import copy
import random
import numpy as np
import math
from scipy.spatial import KDTree
from time import sleep
from copy import deepcopy

# can get inverse kinematics from pybullet tools ikfast -> inverse kinematics to find 

from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, interval_generator, set_joint_positions, \
    link_from_name, get_link_pose, get_joint_positions, set_pose
from src.utils import are_confs_close, set_tool_pose, get_tool_from_root
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import closest_inverse_kinematics
# get_tool_from_root
# tool is the link gripper attached
from src.utils import are_confs_close    

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
            collision_free = self.collision_check(conf_new, self.world)
            if collision_free:
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

    

    def collision_check(self, conf_new, world):
        '''Checks if the new configuration is collision free wrt itself and world'''
        
        return True


#**********************************************************  RRT FOR BASE ***************************************************************#

    def base_rrt(self, base_start_pose, goal_pose):
        '''
        RRT algorithm that creates a motion plan for the base of the robot, accounts for dynamic constraints
        '''
        vertices = {base_start_pose}
        edges = dict()
        N = 0
        while True:
            N += 1

            if N % 10:
                # Goal bias every 10 cycle iterations
                rand_base_pose = goal_pose
            else:
                # Sample a random pose in the kitchen
                rand_base_pose = self.random_base_pose()

            # Determine the nearest pose to that in your 'graph' and calculate distance
            (nearest_pose, xy_dist_to_nearest, yaw_diff_to_nearest) = self.find_nearest_pose(vertices, rand_base_pose)

            # Steer from nearest pose to rand pose based on limits
            new_pose = self.steer_new_pose(rand_base_pose, nearest_pose, xy_dist_to_nearest, yaw_diff_to_nearest)

            # TODO: Check for collisions
            collision_free = self.collision_check(new_pose, self.world)

            # If collision free, add the new_pose to vertices list and edge
            if collision_free:
                vertices.add(new_pose)
                edges[new_pose] = nearest_pose

                # Check if you are in the goal region
                if self.base_close_to_goal(new_pose, goal_pose):
                    solution_path = self.path_to_vertex(base_start_pose, new_pose, edges)
                    return solution_path

    def random_base_pose(self):
        '''Samples a random base pose (x, y position and yaw angle) within world limits.'''
        # These limits were obtained by finding the limits of the black box on the ground of the sim
        x_limits = [0.5, 2]
        y_limits = [-1.5, 1.5]
        yaw_limits = [-np.pi, np.pi]
        
        # Randomly pick a value for each
        rand_x = random.uniform(x_limits[0], x_limits[1])
        rand_y = random.uniform(y_limits[0], y_limits[1])
        rand_yaw = random.uniform(yaw_limits[0], yaw_limits[1])

        #Return a random base position
        random_base_pose = (rand_x, rand_y, rand_yaw)
        return random_base_pose

    def find_nearest_pose(self, vertices, rand_base_pose):
        # Initialize a min distance variable
        min_dist_xy = np.inf
        min_diff_yaw = np.inf     

        # Unpack the new rand_base_pose tuple
        (x_rand, y_rand, yaw_rand) = rand_base_pose

        for v in vertices:
            # unpack the tuple
            (x, y, yaw) = v

            # Calc the euclidean distance between the x, y postion being test against the random position
            dist_xy = math.sqrt((x_rand - x)**2 + (y_rand - y)**2)

            #Account for the difference in yaw angle (gets the angle needed to move from angle yaw to yaw_rand)
            yaw_diff = ((yaw_rand - yaw) + np.pi) % (2*np.pi) - np.pi

            # *** Really want the yaw diff to get from yaw to the position xy (should constrain motion to move in direction of where you are pointing) ***
            #vector1 = np.array([x_rand - x, y_rand - y]) # vector from node v to new xy_rand
            #vector2 = np.array([math.sin(yaw) + x, math.cos(yaw) + y]) # unit vector from node v in yaw direction

            #angle_diff_to_get_to_v_rand = math.atan(np.cross(vector2, vector1)/np.dot(vector1, vector2))
            #yaw_diff = ((angle_diff_to_get_to_v_rand) + np.pi) % (2*np.pi) - np.pi
            
            # Check if this v is closest based on xy distance, if so update
            if dist_xy < min_dist_xy:
                min_dist_xy = dist_xy
                nearest_pose_xy = v
                diff_yaw_for_xy_v = yaw_diff

            # Check if this v is closest based on yaw angle, if so update
            if abs(yaw_diff) < abs(min_diff_yaw):    
                min_diff_yaw = yaw_diff
                nearest_pose_yaw = v
                dist_xy_for_yaw_v = dist_xy

        # Now have two nodes, potentially the same, that encompass the nodes closes based on distance and rotation
        # Now going to calculate which one is closer for distance AND rotation
        pose_denominator = min_dist_xy + dist_xy_for_yaw_v + 0.000000000001
        yaw_denominator = abs(min_diff_yaw) + abs(diff_yaw_for_xy_v) + 0.00000000001
        #print(f"pose denom: {pose_denominator}")
        #print(f"yaw denom: {yaw_denominator}")
        pose_test = (min_dist_xy/pose_denominator) + (diff_yaw_for_xy_v/yaw_denominator)
        yaw_test = (dist_xy_for_yaw_v/pose_denominator) + (abs(min_diff_yaw)/yaw_denominator)

        if pose_test < yaw_test:
            nearest_pose = nearest_pose_xy
            min_dist_xy = min_dist_xy
            min_diff_yaw = diff_yaw_for_xy_v
        else:
            nearest_pose = nearest_pose_yaw
            min_dist_xy = dist_xy_for_yaw_v
            min_diff_yaw = min_diff_yaw

        return (nearest_pose, min_dist_xy, min_diff_yaw)

    def steer_new_pose(self, rand_base_pose, nearest_pose, xy_dist_to_nearest, yaw_diff_to_nearest):
        # Unpack the nearest and rand tuples
        (x_near, y_near, yaw_near) = nearest_pose
        (x_rand, y_rand, yaw_rand) = rand_base_pose

        # Steer in direction of rand based on edge length limit
        xy_limit = self.edge_len
        yaw_limit = np.pi/25

        # Calculate the angle between the nearest heading angle and the angle required to move to the nearest point

        
        if xy_dist_to_nearest > xy_limit:
            new_pose_x = (xy_limit/xy_dist_to_nearest)*(x_rand - x_near) + x_near
            new_pose_y = (xy_limit/xy_dist_to_nearest)*(y_rand - y_near) + y_near
        
        if abs(yaw_diff_to_nearest) > yaw_limit:
            if yaw_diff_to_nearest > 0:
                new_pose_yaw = yaw_near + yaw_limit
            else:
                new_pose_yaw = yaw_near - yaw_limit
        
        # Assign the values of the new pose based on logic, Assumption = can only turn or move linearly
        # turn first then move linear
        if xy_dist_to_nearest > xy_limit and abs(yaw_diff_to_nearest) > yaw_limit:
            new_pose = (new_pose_x, new_pose_y, new_pose_yaw)
        elif xy_dist_to_nearest > xy_limit:
            new_pose = (new_pose_x, new_pose_y, yaw_rand)
        elif abs(yaw_diff_to_nearest) > yaw_limit:
            new_pose = (x_rand, y_rand, new_pose_yaw)
        else:
            new_pose = rand_base_pose        

        return new_pose

    def base_close_to_goal(self, new_pose, goal_pose):
        # Unpack poses
        (x_new, y_new, yaw_new) = new_pose
        (x_goal, y_goal, yaw_goal) = goal_pose

        # Define the margin of error for position and yaw
        position_bound = 0.1
        yaw_bound = np.pi/16

        # Initialize booleans for x, y and yaw
        x_within_bound = False
        y_within_bound = False
        yaw_within_bound = False

        # Test if each is within bound 
        if x_goal - position_bound < x_new and x_new < x_goal + position_bound:
            x_within_bound = True
        if y_goal - position_bound < y_new and y_new < y_goal + position_bound:
            y_within_bound = True
        if yaw_goal - yaw_bound < yaw_new and yaw_new < yaw_goal + yaw_bound:
            yaw_within_bound = True

        # If each true, goal was found, return true
        if x_within_bound and y_within_bound and yaw_within_bound:
            return True
        else:
            return False

    def path_to_vertex(self, conf_start, conf_end, edges):
        '''Returns the total path from start configuration to end configuration.'''
        c = edges[conf_end]
        path = [conf_end, c]
        while c != conf_start:
            c = edges[c]
            path.append(c)
        path.reverse()
        return path

    def execute_base_motion_plan(self, plan):
        for pose in plan:
            set_joint_positions(self.world.robot, self.world.base_joints, pose)
            sleep(.1)
        