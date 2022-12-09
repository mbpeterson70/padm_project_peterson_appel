import time
import numpy as np
from pydrake.all import (AddMultibodyPlantSceneGraph, BsplineTrajectory,
                         DiagramBuilder, KinematicTrajectoryOptimization,
                         MinimumDistanceConstraint, Parser, PositionConstraint,
                         Rgba, RigidTransform, Role, Solve, Sphere,
                         )

class TrajectoryOp():
    '''
    Creates an optimized plan to move gripper from some initial position to goal position.
    '''

    def __init__(self, rrt_edge_len_arm=.2, world=None, tol=1e-9, q_len=7):
        self.edge_len_arm = rrt_edge_len_arm
        self.world = world
        self.tol = tol
        self.q_len = q_len

    def optimize_trajectory(self):
        # Set up PyDrake Trajectory Optimization Object
        trajopt = KinematicTrajectoryOptimization(self.q_len, 10)
        prog = trajopt.get_mutable_prog() #Getter for a mutable pointer to the optimization program.
        trajopt.AddDurationCost(1.0) #Adds a linear cost on the duration of the trajectory.
        trajopt.AddPathLengthCost(1.0) #Adds a cost on an upper bound on length of the path
        # trajopt.AddPositionBounds() #Adds bounding box constraints to enforce upper and lower bounds on the positions trajectory
        # trajopt.AddVelocityBounds() #Adds linear constraints to enforce upper and lower bounds on the velocity trajectory

        # Define the start constraint
        # start_constraint = PositionConstraint()
        # trajopt.AddPathPositionConstraint(start_constraint, 0)
        # prog.AddQuadraticErrorCost(np.eye(self.q_len), q0, trajopt.control_points()[:, 0]) # q0 = ???

        # Define the goal constraint
        # goal_constraint = PositionConstraint()
        # trajopt.AddPathPositionConstraint(goal_constraint, 0)
        # prog.AddQuadraticErrorCost(np.eye(self.q_len), q0, trajopt.control_points()[:, 0]) # q0 = ???

        # start and end with zero velocity
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 0)
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 1)

        # Solve once without the collisions and set that as the initial guess for
        # the version with collisions.
        result = Solve(prog)
        if not result.is_success():
            print("Trajectory optimization failed, even without collisions!")
            print(result.get_solver_id().name())

        return result
