import numpy as np
import os
from task_optim.test_scenario.one_com_waypoint_static_test import OneComWaypointStaticTest
from task_optim.solvers.robo_solver import RoboSolver
from task_optim.solvers.cma_solver import CmaSolver

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/First_Test/"

right_hand_starting_waypoints = np.array([[0.36, -0.23, 0.5]])
com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])

first_test = OneComWaypointStaticTest(root_path, right_hand_starting_waypoints, com_starting_waypoints)

# solver_parameters = {'max_iter':30, 'par':0.01, 'kernel':'RBF', 'maximizer':'CMAES'}
# solver = RoboSolver(first_test, solver_parameters)

solver_parameters = {'max_iter':50, 'initial_sigma':0.05}
solver = CmaSolver(first_test, solver_parameters)

solver.optimize()

solver.returnSolution(show_simulation=True)
