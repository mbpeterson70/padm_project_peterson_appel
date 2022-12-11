import numpy as np
from scipy.spatial.transform import Rotation
import time

from motion_planner import MotionPlanner
import world_params as wp

from pybullet_tools.utils import get_joint_positions, wait_for_user, get_link_pose, \
    link_from_name, plan_cartesian_motion, set_joint_positions, get_joint, get_joint_position, \
    set_joint_position, get_pose, set_pose
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import closest_inverse_kinematics
from src.utils import name_from_type

class ActivityExecutor():

    def __init__(self, activity_plan, world):
        self.activity_plan = activity_plan
        self.activity_idx = 0
        self.mp = MotionPlanner(rrt_edge_len_arm=.01, rrt_edge_len_base_xy=.1, world=world, tol=1e-9)
        self.world = world
        self.grabbing = None
        self.item_in_drawer = None
        self.testing_mode = False

    def execute_activity_plan(self):
        while self.activity_idx < len(self.activity_plan):
            self.next_activity()
            # wait_for_user()

    def next_activity(self):
        activity = self.activity_plan[self.activity_idx]
        print(f'Performing action: {activity.name} {activity.parameters}')
        if activity.name == 'move-to-base':
            self.move_to_base(activity.parameters)
            wait_for_user()
        elif activity.name == 'move-gripper' or activity.name == 'move-gripper-from-drawer' \
            or activity.name == 'move-item':
            self.move_gripper(activity.parameters)
        elif activity.name == 'open-drawer':
            self.open_drawer(activity.parameters)
        elif activity.name == 'close-drawer':
            self.close_drawer(activity.parameters)
        elif activity.name == 'grab-item':
            self.grab_item(activity.parameters)
        elif activity.name == 'release-object':
            self.release_object(activity.parameters)
        elif activity.name == 'move-item-to-drawer':
            self.move_item_to_drawer(activity.parameters)
        self.activity_idx += 1

    def move_to_base(self, params):
        if self.testing_mode:
            set_joint_positions(self.world.robot, self.world.base_joints, (0.67, .57, np.pi/2))
            return
        base_plan = self.mp.base_rrt(wp.BASE_COUNTER_POSE)
        #print(base_plan)
        wait_for_user()
        self.mp.execute_base_motion_plan(base_plan)
        self.mp.turn_and_drive(wp.BASE_COUNTER_POSE)

    def move_gripper(self, params):
        gripper = params[0]
        if len(params) == 3: # move item
            start_loc = params[1]
            end_loc = params[2]
        elif len(params) == 4:
            start_loc = params[2]
            end_loc = params[3]
        motion_plan = []

        if end_loc == 'at-handle':
            if self.testing_mode and start_loc == 'away-from-objects':
                return
            if start_loc == 'on-counter':
                motion_plan += self.intermediate_move_away(wp.POSE_AWAY2, wp.ATT_AWAY2)
            handle_link = link_from_name(self.world.kitchen, wp.HANDLE_NAME)
            handle_pose = get_link_pose(self.world.kitchen, handle_link)
            handle_position = tuple([handle_pose[0][0], handle_pose[0][1]-.1, handle_pose[0][2]])
            goal_pose = ((handle_position), wp.ATT_AT_HANDLE)
        elif end_loc == 'on-counter':
            countertop_link = link_from_name(self.world.kitchen, 'indigo_countertop')
            countertop_z = get_link_pose(self.world.kitchen, countertop_link)[0][2]
            countertop_position = tuple([wp.POSE_ON_COUNTER[0], wp.POSE_ON_COUNTER[1], 
                wp.POSE_ON_COUNTER[2] + countertop_z])
            if start_loc == 'on-burner':
                motion_plan += self.intermediate_move_away(wp.POSE_AWAY, wp.ATT_AWAY)
            if self.grabbing:
                goal_pose = (countertop_position, wp.ATT_ON_BURNER)
            else:
                goal_pose = (countertop_position, wp.ATT_ON_COUNTER)
        elif end_loc == 'on-burner':
            burner_link = link_from_name(self.world.kitchen, 'front_right_stove')
            burner_z = get_link_pose(self.world.kitchen, burner_link)[0][2]
            burner_position = tuple([wp.POSE_ON_BURNER[0], wp.POSE_ON_BURNER[1],
                wp.POSE_ON_BURNER[2] + burner_z])
            goal_pose = (burner_position, wp.ATT_ON_BURNER)
            motion_plan += self.intermediate_move_away(wp.POSE_AWAY, wp.ATT_AWAY)
        elif end_loc == 'away-from-objects':
            goal_pose = (wp.POSE_AWAY2, wp.ATT_AT_HANDLE)
        if motion_plan:
            motion_plan += self.mp.motion_plan_rrt(goal_pose, start_conf=motion_plan[-1])
        else:
            motion_plan = self.mp.motion_plan_rrt(goal_pose)
        if self.grabbing is None:
            self.mp.execute_motion_plan(motion_plan) 
        else:
            item_rot_init = Rotation.from_euler('xyz', [0, 0, np.pi / 4])
            item_tran_init = self.get_grabbing_translation()
            self.mp.execute_motion_plan(motion_plan, self.grabbing, item_rot_init, item_tran_init)

    def grab_handle(self, params):
        self.grabbing = wp.HANDLE_NAME

    def grab_item(self, params):
        gripper = params[0]
        item = params[1]
        if item == 'sp':
            self.grabbing = wp.SPAM_NAME
        elif item == 'su':
            self.grabbing = wp.SUGAR_NAME

    def open_drawer(self, params):
        self.move_drawer(1)
        
    def close_drawer(self, params):
        self.move_drawer(-1)

    def move_drawer(self, sign):
        # assert self.grabbing == wp.HANDLE_NAME
        interp_vec = sign*np.array(wp.DRAWER_OPEN_DIR) * wp.DRAWER_OPEN_DIST / wp.DRAWER_INTERP_NUM
        for i in range(wp.DRAWER_INTERP_NUM):
            gripper_link = link_from_name(self.world.robot, 'right_gripper')
            gripper_pose = get_link_pose(self.world.robot, gripper_link)
            goal_position = tuple((np.array(gripper_pose[0]) + interp_vec).tolist())
            goal_pose = (goal_position, gripper_pose[1])
            if not self.testing_mode:
                motion_plan = self.mp.motion_plan_rrt(goal_pose, collision=False)
                self.mp.execute_motion_plan(motion_plan) 
            drawer_joint = get_joint(self.world.kitchen, 'indigo_drawer_top_joint')
            drawer_conf = get_joint_position(self.world.kitchen, drawer_joint)
            set_joint_position(self.world.kitchen, drawer_joint, \
                drawer_conf + sign*wp.DRAWER_OPEN_DIST / (wp.DRAWER_INTERP_NUM*wp.DRAWER_OPEN_SCALE))
            if self.item_in_drawer is not None:
                body_name = self.world.get_body(self.item_in_drawer)
                item_pose = get_pose(body_name)
                item_position = np.array(item_pose[0])
                new_position = item_position + interp_vec
                set_pose(body_name, ((new_position.tolist()), item_pose[1]))
    
    def intermediate_move_away(self, away_position, away_attitude):
        intermediate_pose = (away_position, away_attitude)
        motion_plan = self.mp.motion_plan_rrt(intermediate_pose)
        return motion_plan
        # if self.grabbing is None:
        #     self.mp.execute_motion_plan(motion_plan) 
        # else:
        #     item_rot_init = Rotation.from_euler('xyz', [0, 0, np.pi / 4])
        #     item_tran_init = self.get_grabbing_translation()
        #     self.mp.execute_motion_plan(motion_plan, self.grabbing, item_rot_init, item_tran_init)

    def release_object(self, params):
        self.grabbing = None
    
    def move_item_to_drawer(self, params):
        gripper = params[0]
        item = params[1]
        if item == 'sp':
            self.item_in_drawer = wp.SPAM_NAME
        drawer_link = link_from_name(self.world.kitchen, wp.DRAWER_NAME)
        drawer_pose = get_link_pose(self.world.kitchen, drawer_link)
        gripper_drawer_position = tuple([drawer_pose[0][0], drawer_pose[0][1], 
            drawer_pose[0][2] + .1])
        goal_pose = (gripper_drawer_position, wp.ATT_ON_COUNTER)
        motion_plan = self.mp.motion_plan_rrt(goal_pose)
        item_rot_init = Rotation.from_euler('xyz', [0, 0, np.pi / 4])
        item_tran_init = self.get_grabbing_translation()
        self.mp.execute_motion_plan(motion_plan, self.grabbing, item_rot_init, item_tran_init)
    
    def move_gripper_from_drawer(self, params):
        pass

    def get_grabbing_translation(self):
        if self.grabbing == wp.SPAM_NAME:
            item_tran_init = [0, 0, -.1]
        elif self.grabbing == wp.SUGAR_NAME:
            item_tran_init = [0, .05, -.1]
        return item_tran_init
    
    


