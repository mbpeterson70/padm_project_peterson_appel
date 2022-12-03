import numpy as np

from motion_planner import MotionPlanner
import world_params as wp

from pybullet_tools.utils import get_joint_positions, wait_for_user, get_link_pose, \
    link_from_name, plan_cartesian_motion, set_joint_positions, get_joint
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import closest_inverse_kinematics

class ActivityExecutor():

    def __init__(self, activity_plan, world):
        self.activity_plan = activity_plan
        self.activity_idx = 0
        self.mp = MotionPlanner(rrt_edge_len_arm=.01, rrt_edge_len_base_xy=.1, world=world, tol=1e-9)
        self.world = world
        self.grabbing = None

    def execute_activity_plan(self):
        while self.activity_idx < len(self.activity_plan):
            self.next_activity()
            wait_for_user()

    def next_activity(self):
        activity = self.activity_plan[self.activity_idx]
        print(f'Performing action: {activity.name} {activity.parameters}')
        if activity.name == 'move-to-base':
            self.move_to_base(activity.parameters)
        elif activity.name == 'move-gripper':
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
        elif activity.name == 'move-gripper-from-drawer':
            self.move_gripper_from_drawer(activity.parameters)
        self.activity_idx += 1

    def move_to_base(self, params):
        #TODO: DELETE
        gripper_link = link_from_name(self.world.robot, 'panda_hand')
        print(get_link_pose(self.world.robot, gripper_link))
        base_plan = self.mp.base_rrt(wp.BASE_COUNTER_POSE)
        #print(base_plan)
        wait_for_user()
        self.mp.execute_base_motion_plan(base_plan)

    def move_gripper(self, params):
        gripper = params[0]
        start_loc = params[1]
        end_loc = params[2]
        if end_loc == 'at-handle':
            handle_link = link_from_name(self.world.kitchen, wp.HANDLE_NAME)
            handle_pose = get_link_pose(self.world.kitchen, handle_link)
            goal_pose = ((handle_pose[0]), wp.ATT_AT_HANDLE)
            motion_plan = self.mp.motion_plan_rrt(goal_pose)
            self.mp.execute_motion_plan(motion_plan) 

    def grab_handle(self, params):
        self.grabbing = wp.HANDLE_NAME

    def open_drawer(self, params):
        assert self.grabbing == wp.HANDLE_NAME
        interp_vec = np.array(wp.DRAWER_OPEN_DIR) * wp.DRAWER_OPEN_DIST / wp.DRAWER_INTERP_NUM
        for i in range(wp.DRAWER_INTERP_NUM):
            gripper_link = link_from_name(self.world.robot, 'right_gripper')
            gripper_pose = get_link_pose(self.world.robot, gripper_link)
            goal_position = tuple((np.array(gripper_pose[0]) + interp_vec).tolist())
            print(gripper_pose)
            print(goal_position)
            goal_pose = (goal_position, gripper_pose[1])
            motion_plan = self.mp.motion_plan_rrt(goal_pose)
            self.mp.execute_motion_plan(motion_plan) 
            set_joint_positions(self.world.kitchen, [get_joint(self.world.kitchen, 'indigo_drawer_top_joint')], [i*wp.DRAWER_OPEN_DIST / (wp.DRAWER_INTERP_NUM*wp.DRAWER_OPEN_SCALE)])

    def close_drawer(self, params):
        pass
    
    def grab_item(self, params):
        pass
    
    def release_object(self, params):
        self.grabbing = None
        pass
    
    def move_item_to_drawer(self, params):
        pass
    
    def move_gripper_from_drawer(self, params):
        pass
    
    


