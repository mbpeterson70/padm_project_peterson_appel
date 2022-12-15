import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.solvers import MathematicalProgram, Solve
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits


class TrajectoryOp():
    '''
    Creates an optimized plan to move gripper from some initial position to goal position.
    '''

    def __init__(self, rrt_edge_len_arm=.2, world=None, tol=1e-9, q_len=7):
        self.edge_len_arm = rrt_edge_len_arm
        self.world = world
        self.tol = tol
        self.q_len = q_len
        self.lower_limits, self.upper_limits = \
            get_custom_limits(self.world.robot, self.world.arm_joints, {}, circular_limits=CIRCULAR_LIMITS)


    # Bezier curve is a function of u. you can essentially make u = t/T. which would be the timestep you are on over the total number of timesteps.
    # with this formulation the configuration x(t) is a function of time. Make the 'cost' the psuedo euclidian distance for each joint to the end goal
    def optimize_trajectory(self, motion_plan):
        # Create a variable for the total number of time steps of the original motion_plan
        T = len(motion_plan)
        
        # Establish the Mathematical Program
        prog = MathematicalProgram()

        # Add Tx7 continuous variables
        # q = prog.NewContinuousVariables(T, 7, "q")
        x1 = prog.NewContinuousVariables(7, 'x1')
        x2 = prog.NewContinuousVariables(7, 'x2')

        # Constrain the first and last control points of the bezier curve to the start and goal configuration respectively 
        x0 = list(motion_plan[0])
        x3 = list(motion_plan[-1])
        # for i in range(len(x0)):
        #     prog.AddConstraint(q[0][i] == x0[i])
        #     prog.AddConstraint(q[-1][i] == x3[i])

        for t in range(T):
            u = t/T
            for i in range(len(x0)):
                der_sign = np.sign(x3[i] - x0[i])
                xdot = self.bezier_curve_der(u, x0[i], x1[i], x2[i], x3[i])
                xddot = self.bezier_curve_sec_der(u, x0[i], x1[i], x2[i], x3[i])
                prog.AddConstraint(der_sign * xdot >= 0)
                prog.AddConstraint(der_sign * xddot <= 0)
                #prog.AddConstraint(der_sign * xdot <= 2*np.pi)

        for t in range(T-1):
            k = t/T
            kp1 = (t+1)/T # k plus 1
            for i in range(len(x0)):
                der_sign = np.sign(x3[i] - x0[i])
                x_k = self.bezier_curve(k, x0[i], x1[i], x2[i], x3[i])
                x_kp1 = self.bezier_curve(kp1, x0[i], x1[i], x2[i], x3[i])
                prog.AddConstraint(der_sign * (x_kp1 - x_k) <= .1*np.pi)

        # Constrain all the points on the curve to be between -pi and pi
        # for t in range(T):
        #     for i in range(len(x0)):
                # prog.AddBoundingBoxConstraint(-np.pi, np.pi, q[t][i])
        # TODO: Add actual joint constraints
        prog.AddBoundingBoxConstraint(self.lower_limits, self.upper_limits, x1)
        prog.AddBoundingBoxConstraint(self.lower_limits, self.upper_limits, x2)

        # Set initial guess for x1 and x2
        prog.SetInitialGuess(x1, list(motion_plan[round(1*T/3)]))
        prog.SetInitialGuess(x2, list(motion_plan[round(2*T/3)]))

        # Constrain the states to bezier curve
        # for t in range(T):
        #     u = t/T
        #     # print(u)
        #     if (t != 0) and (t != (T-1)):
        #         for i in range(len(x0)):
        #             x = self.bezier_curve(u, x0[i], q[round(T/4)][i], q[round(3*T/4)][i], x3[i])
        #             prog.AddConstraint(q[t][i] == x)

        # Add the psuedo euclidean distance cost function for each point
        for t in range(T):
            u = t/T
            for i in range(len(x0)):
                x = self.bezier_curve(u, x0[i], x1[i], x2[i], x3[i])
                prog.AddQuadraticCost(((t**2)*(x-x3[i]))**2)
        # t = T - 1
        # u = t/T
        # x = self.bezier_curve(u, x0[6], x1[6], x2[6], x3[6])
        # prog.AddQuadraticCost((x-x3[6])**2)

        result = Solve(prog)

        if not result.is_success():
            print("Trajectory optimization failed, even without collisions!")
            print(result.get_solver_id().name())
            return None
        else:
            x1_sol = result.GetSolution(x1)
            x2_sol = result.GetSolution(x2)
            op_path = self.trajop(T, x0, x1_sol, x2_sol, x3)

            return op_path

    def bezier_curve(self, u, x0, x1, x2, x3):
        x = ((1-u)**3)*x0 + (3*u*(1-u)**2)*x1 + 3*(u**2)*(1-u)*x2 + (u**3)*x3
        return x

    def bezier_curve_der(self, u, x0, x1, x2, x3):
        xdot =  -3*(1-u)**2*x0 + 3*(1-u)**2*x1 - 6*u*(1-u)*x1 \
            - 3*u**2 * x2 + 6*u*(1-u) * x2 + 3*u**2 * x3
        return xdot

    def bezier_curve_sec_der(self, u, x0, x1, x2, x3):
        xddot = 6*(1-u)*x0 - 6*(1-u)*x1 - 6*(1-2*u)*x1 \
            - 6*u*x2 + 6*(1-2*u)*x2 + 6*u*x3
        return xddot

    def trajop(self, T, x0, x1, x2, x3):
        op_path = []
        joint1 = []
        U = []
        for t in range(T):
            u = t/T
            op_path.append([])
            for i in range(len(x0)):
                x = self.bezier_curve(u, x0[i], x1[i], x2[i], x3[i])
                if i == 1:
                    joint1.append(x)
                    U.append(u)
                op_path[t].append(x)

        if False:
            plt.xlabel('t/T (time/total time)'); plt.ylabel('Joint 1 angle (deg)'); plt.title('Joint 1 Optimized Trajectory')
            plt.plot(np.array(U),np.array(joint1)*180/np.pi)
            plt.show()
        
        return op_path
