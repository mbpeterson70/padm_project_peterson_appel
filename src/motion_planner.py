import copy
import random
import numpy as np
import math
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation
from time import sleep

# can get inverse kinematics from pybullet tools ikfast -> inverse kinematics to find 

from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, interval_generator, set_joint_positions, \
    link_from_name, get_link_pose, get_joint_positions, set_pose, pairwise_collision, single_collision, get_bodies, get_collision_data, contact_collision, \
        pairwise_link_collision, any_link_pair_collision, link_pairs_collision, get_all_links, get_link_name, clone_body, get_movable_joints, get_joint_name
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

    def __init__(self, rrt_edge_len_arm=.2, rrt_edge_len_base_xy=.1, rrt_edge_len_base_psi=np.pi/32,
        rrt_goal_biasing=5, world=None, tol=1e-9):
        self.edge_len_arm = rrt_edge_len_arm
        self.edge_len_base_xy = rrt_edge_len_base_xy
        self.edge_len_base_psi = rrt_edge_len_base_psi
        self.goal_biasing = rrt_goal_biasing
        self.world = world
        self.lower_limits, self.upper_limits = \
            get_custom_limits(self.world.robot, self.world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)
        self.generator = interval_generator(self.lower_limits, self.upper_limits)
        self.tool_link = link_from_name(self.world.robot, 'right_gripper')
        self.tol = tol
        self.visual = True

    def execute_motion_plan(self, plan, item=None, item_rot_init=None):
        if item is not None:
            print(item_rot_init)
            assert item_rot_init is not None
            body_name = self.world.get_body(item)
            pose_gripper_init = gripper_pose = get_link_pose(self.world.robot, self.tool_link)
            rot_gripper_init = Rotation.from_quat(pose_gripper_init[1]) # initial rotation
            # item_rot_init = Rotation.from_quat(item_quat_init)
        for conf in plan:
            set_joint_positions(self.world.robot, self.world.arm_joints, conf)
            if item is not None:
                gripper_pose = get_link_pose(self.world.robot, self.tool_link)
                gripper_rot = Rotation.from_quat(gripper_pose[1])
                item_rot_matrix =  gripper_rot.as_matrix() @ rot_gripper_init.as_matrix().T @ \
                    item_rot_init.as_matrix()
                item_rot_q = Rotation.from_matrix(item_rot_matrix).as_quat()
                set_pose(body_name, (gripper_pose[0], tuple(item_rot_q.tolist())))
            if self.visual:
                sleep(.01)

    def motion_plan_rrt(self, goal_pose, conf_start=None):
        '''
        RRT algorithm to move from start configuration (series of joint angles) to 
        goal pose (gripper position and orientation.
        '''

        if conf_start == None:
            conf_start = get_joint_positions(self.world.robot, self.world.arm_joints)
        goal_conf = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, goal_pose, max_time=0.05), None)
        goal_conf = self.unwrap_conf(conf_start, goal_conf)
        vertices = {conf_start}
        edges = dict()

        for i in range(int(1e20)):
            if i % self.goal_biasing == 0:
                conf_rand = tuple(goal_conf)
            else:
                conf_rand = self.get_random_configuration()
            conf_nearest, conf_nearest_dist = self.nearest_conf(vertices, conf_rand)
            if conf_nearest_dist > self.edge_len_arm:
                conf_new = np.array(conf_nearest) + \
                    (np.array(conf_rand)-np.array(conf_nearest)) * self.edge_len_arm / conf_nearest_dist
                conf_new = tuple(conf_new.tolist())
            else:
                conf_new = conf_rand
            # TODO: Check for collision here!!
            # collision_free = self.collision_check(conf_new, self.world)
            if True:
                vertices.add(conf_new)
                edges[conf_new] = conf_nearest
                if np.linalg.norm(np.array(conf_new) - np.array(goal_conf)) < self.tol:
                    solution_path = self.path_to_vertex(conf_start, conf_new, edges)
                    return solution_path

    def unwrap_conf(self, ref_conf, wrapped_conf):
        unwrapped_conf = []
        for i in range(len(ref_conf)):
            if np.abs(wrapped_conf[i] - ref_conf[i]) < np.pi:
                unwrapped_conf.append(wrapped_conf[i])
            else:
                diff = (wrapped_conf[i] % (2*np.pi)) - (ref_conf[i] % (2*np.pi))
                if abs(diff) < np.pi:
                    new_angle = ref_conf[i] + diff
                else:
                    new_angle = ref_conf[i] + diff - np.sign(diff)*2*np.pi
                while new_angle > self.upper_limits[i]:
                    new_angle -= 2*np.pi
                while new_angle < self.lower_limits[i]:
                    new_angle += 2*np.pi
                unwrapped_conf.append(new_angle)  
        return unwrapped_conf

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

#**********************************************************  RRT FOR BASE ***************************************************************#

    def base_rrt(self, goal_pose, base_start_pose=None):
        '''
        RRT algorithm that creates a motion plan for the base of the robot, accounts for dynamic constraints
        '''
        if base_start_pose == None:
            base_start_pose = get_joint_positions(self.world.robot, self.world.base_joints)
        vertices = {base_start_pose}
        edges = dict()
        N = 0

        self.clone_bot = clone_body(self.world.robot, None, collision=True, visual=False)
        self.clone_bot_joints = get_movable_joints(self.clone_bot)

        while True:
            N += 1

            if N % 2 == 0:
                # Goal bias every 10 cycle iterations
                rand_base_pose = goal_pose
            else:
                # Sample a random pose in the kitchen
                rand_base_pose = self.random_base_pose()

            # Determine the nearest pose to that in your 'graph' and calculate distance
            (nearest_pose, xy_dist_to_nearest, yaw_diff_to_nearest, rand_base_pose) = self.find_nearest_pose(vertices, rand_base_pose)

            # Steer from nearest pose to rand pose based on limits
            new_pose = self.steer_new_pose(rand_base_pose, nearest_pose, xy_dist_to_nearest, yaw_diff_to_nearest)
            if new_pose == None:
                continue

            # Check for collisions
            collision_free = self.base_collision_check(new_pose, self.clone_bot, self.clone_bot_joints)

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
        x_limits = [0.5, 2.0]
        y_limits = [-1.5, -0.5]
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

        rand_base_pose = (x_rand, y_rand, yaw_rand)

        return (nearest_pose, min_dist_xy, min_diff_yaw, rand_base_pose)

    def steer_new_pose(self, rand_base_pose, nearest_pose, xy_dist_to_nearest, yaw_diff_to_nearest):
        # Unpack the nearest and rand tuples
        (x_near, y_near, yaw_near) = nearest_pose
        (x_rand, y_rand, yaw_rand) = rand_base_pose

        # Steer in direction of rand based on edge length limit
        xy_limit = self.edge_len_base_xy
        yaw_limit = self.edge_len_base_psi

        # This checks that you are not moving sideways to better ensure motion constraints
        if x_rand - x_near == 0:
            return None
        delta_yaw = math.atan((y_rand - y_near)/(x_rand - x_near))
        if x_rand < x_near:
            yaw_rand_to_get_to_near = delta_yaw + np.pi
        else:
            yaw_rand_to_get_to_near = delta_yaw

        yaw_diff = ((yaw_rand - yaw_rand_to_get_to_near) + np.pi) % (2*np.pi) - np.pi

        if abs(yaw_diff) > yaw_limit:
            return None

        # If sideway motion test passed, check if position limit exceeded
        if xy_dist_to_nearest > xy_limit:
            new_pose_x = (xy_limit/xy_dist_to_nearest)*(x_rand - x_near) + x_near
            new_pose_y = (xy_limit/xy_dist_to_nearest)*(y_rand - y_near) + y_near
        
        # Check if yaw limit exceeded
        if abs(yaw_diff_to_nearest) > yaw_limit:
            if yaw_diff_to_nearest > 0:
                new_pose_yaw = yaw_near + yaw_limit
            else:
                new_pose_yaw = yaw_near - yaw_limit
        
        # Assign the values of the new pose based on logic, Assumption = can only turn or move linearly
        # turn first then move linear
        if xy_dist_to_nearest > xy_limit and abs(yaw_diff_to_nearest) > yaw_limit:
            new_pose = (new_pose_x, new_pose_y, new_pose_yaw)
        elif abs(yaw_diff_to_nearest) > yaw_limit:
            new_pose = (x_rand, y_rand, new_pose_yaw)
        elif xy_dist_to_nearest > xy_limit:
            new_pose = (new_pose_x, new_pose_y, yaw_rand)
        else:
            new_pose = rand_base_pose      

        return new_pose

    def base_collision_check(self, conf_new, clone_bot, clone_bot_joints):
        '''Checks if the new configuration is collision free wrt itself and world'''
        set_joint_positions(clone_bot, (clone_bot_joints[0], clone_bot_joints[1], clone_bot_joints[2]), conf_new)
        
        # Hard code error in the collision checking to disable collisions between x = [1.5,2]
        (x, y, yaw) = conf_new
        if x < 2 and x > 1.5:
            return True

        # Now check that the robot does not drive into any objects
        for body in get_bodies():
            if body != clone_bot and body != self.world.robot:
                if any_link_pair_collision(clone_bot, None, body):
                    print('collision')
                    return False
            
        return True

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
        elif -yaw_goal - yaw_bound < yaw_new and yaw_new < -yaw_goal + yaw_bound:
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

    def turn_and_drive(self, goal_pose, base_start_pose=None):
        set_joint_positions(self.clone_bot, (self.clone_bot_joints[0], self.clone_bot_joints[1], self.clone_bot_joints[2]), goal_pose)
        set_joint_positions(self.world.robot, self.world.base_joints, goal_pose)
        (start_x, start_y, start_yaw) = get_joint_positions(self.world.robot, self.world.base_joints)

        turn_list = np.linspace(np.pi, np.pi/2, 16)
        
        clone_move_list = np.linspace(-1.1, 0.57, 10)
        bot_move_list = np.linspace(-1.1, 0.57, 100)
        

        # move the clone_bot without visually sleeping the sim
        for turn in turn_list:
            set_joint_positions(self.clone_bot, (self.clone_bot_joints[0], self.clone_bot_joints[1], self.clone_bot_joints[2]), (start_x, start_y, turn))
        for move in clone_move_list:
            set_joint_positions(self.clone_bot, (self.clone_bot_joints[0], self.clone_bot_joints[1], self.clone_bot_joints[2]), (start_x, move, np.pi/2))

        # move the robot with visual sleep for sim
        for turn in turn_list:
            set_joint_positions(self.world.robot, self.world.base_joints, (start_x, start_y, turn))
            sleep(.1)
        for move in bot_move_list:
            set_joint_positions(self.world.robot, self.world.base_joints, (start_x, move, np.pi/2))
            sleep(.04)
        

        return
        