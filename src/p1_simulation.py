from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import time

parent_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.join(parent_dir, 'padm-project-2022f'), 'ss-pybullet')))
sys.path.insert(0, os.path.abspath(os.path.join(parent_dir, 'padm-project-2022f')))

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, \
    COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, \
    create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, \
    set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, \
    get_movable_joints, get_joint_name, get_joint_state, get_joint_position, get_joint_positions
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

from activity_planner import ActivityPlanner
from motion_planner import MotionPlanner

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    print('limits')
    print(lower_limits)
    print(upper_limits)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def main():
    planner = ActivityPlanner('sugar_spam_pddl.pddl', 'p1.pddl')
    solution = planner.A_star_solver()
    for act in solution:
        print(f'{act.name} {act.parameters}')

    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    tool_link = link_from_name(world.robot, 'panda_hand')
    start_pose = get_link_pose(world.robot, tool_link)

    # Make true to check out these functions
    if False:
        print(get_joint_state(world.robot, joint) for joint in world.arm_joints)
        print(get_joint_position(world.robot, joint) for joint in world.arm_joints)
        print(get_joint_positions(world.robot, world.arm_joints))
        sample_fn = get_sample_fn(world.robot, world.arm_joints)
        print(sample_fn())
        print(f'ik_joints: {get_ik_joints(world.robot, PANDA_INFO, tool_link)}')
        print(f'link pose: {get_link_pose(world.robot, tool_link)}')
        end_pose = multiply(start_pose, Pose(Point(z=1.0)))
        print(f'end_pose: {end_pose}')
        new_pose = start_pose
        for i in range(10):
            # I think this translates the vector 1 unit along its z axis (from its quaternion)
            new_pose = multiply(new_pose, Pose(Point(z=1.0)))
            print(f'mult_transform: {new_pose}')
    
    wait_for_user()
    world._update_initial()

    # Make True to see how to move robot
    if False:
        for i in range(1,3):
            for j in range(4):
                goal_joints = list(get_joint_positions(world.robot, world.arm_joints))
                goal_joints[-i] = goal_joints[-i] + np.pi/4
                set_joint_positions(world.robot, world.arm_joints, goal_joints)
                print(f'link pose: {get_link_pose(world.robot, tool_link)}')
                wait_for_user()
    if True:
        mp = MotionPlanner(.4, world, tol=.1)
        plan = mp.motion_plan_rrt(get_joint_positions(world.robot, world.arm_joints), multiply(start_pose, Pose(Point(x=.3, z=.3))))
        wait_for_user()
        mp.execute_motion_plan(plan)


    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
