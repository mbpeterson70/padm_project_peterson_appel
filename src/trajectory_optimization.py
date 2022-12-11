import time
import numpy as np
from scipy.interpolate import BSpline

from pydrake.solvers import MathematicalProgram, Solve

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
# ************************************************ Attempts to use standard Pydrake Mathematical Program **************************************************

        # Establish the Mathematical Program
        prog = MathematicalProgram()

        # Add 2 sets of 7 continous decision variables, 7 joints, 2 control points on bezier curve
        x1 = prog.NewContinuousVariables(7, "x1")
        x2 = prog.NewContinuousVariables(7, "x2")
        
        # Add constraints of -pi to pi for each joint
        constraint1 = prog.AddConstraint(lambda z: np.array([z[0], z[1], z[2], z[3], z[4], z[5], z[6]]), lb=[-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi], \
            ub=[np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi], vars=[[x1[0], x1[1], x1[2], x1[3], x1[4], x1[5], x1[6]]])
        constraint2 = prog.AddConstraint(lambda z: np.array([z[0], z[1], z[2], z[3], z[4], z[5], z[6]]), lb=[-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi], \
            ub=[np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi], vars=[[x2[0], x2[1], x2[2], x2[3], x2[4], x2[5], x2[6]]])

        u = np.linspace(0,1,100)

        # Attempt to optimize the trajectory of the arm using a bezier curve to map the trajectory of the joints
        # x0 = the start configuration
        # x3 = the goal configuration found

        result = Solve(prog)
        if not result.is_success():
            print("Trajectory optimization failed, even without collisions!")
            print(result.get_solver_id().name())

        return result

    def construct_bezier_curve(self, u_full, x0, x1, x2, x3):
        x = []
        for i in range(len(x0)):
            x.append([])

        for u in u_full:
            for i in range(len(x0)):
                x_i = ((1-u)**3)*x0[i] + (3*u*(1-u)**2)*x1[i] + 3*(u**2)*(1-u)*x2[i] + (u**3)*x3[i]
                x[i].append(x_i)
        
        return 

