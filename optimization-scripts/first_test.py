import numpy as np
import os
from task_optim.test_scenario.one_com_waypoint_static_test import OneComWaypointStaticTest
from task_optim.solvers.robo_solver import RoboSolver
from task_optim.solvers.cma_solver import CmaSolver

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/Blah/"

right_hand_starting_waypoints = np.array([[0.36, -0.23, 0.5]])
com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])

# costs_to_use = ['tracking', 'goal', 'energy']
costs_to_use = ['tracking', 'goal']
# costs_to_use = ['tracking', 'energy']
# costs_to_use = ['goal', 'energy']
# costs_to_use = ['tracking']
# costs_to_use = ['goal']
# costs_to_use = ['energy']

####################################################

first_test = OneComWaypointStaticTest(root_path, right_hand_starting_waypoints, com_starting_waypoints, costs_to_use)

solver_parameters = {'max_iter':36, 'par':0.01, 'tolfun':1e-11, 'kernel':'Matern52', 'acquisition':'LCB', 'maximizer':'Direct'}
solver = RoboSolver(first_test, solver_parameters)

solver.optimize()
solver.returnSolution(show_simulation=False)

####################################################

first_test = OneComWaypointStaticTest(root_path, right_hand_starting_waypoints, com_starting_waypoints, costs_to_use)

solver_parameters = {'max_iter':36, 'initial_sigma':0.5, 'tolfun':1e-11}
solver = CmaSolver(first_test, solver_parameters)

solver.optimize()
solver.returnSolution(show_simulation=False)
