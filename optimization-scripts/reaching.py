import numpy as np
import os
from task_optim.test_scenario.one_com_waypoint_static_test import OneComWaypointStaticTest
from task_optim.solvers.robo_solver import RoboSolver
from task_optim.solvers.cma_solver import CmaSolver
from task_optim.solvers.bayes_opt_solver import BayesOptSolver
from task_optim.utils.video import *
from task_optim.utils.files import *

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/reaching/"

# original
right_hand_waypoints = np.array([[0.36, -0.23, 0.5]])
#right_hand_waypoints = np.array([[0.2, -0.1, 0.3]])
com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])

costs_to_use = ['tracking', 'goal', 'energy']
# costs_to_use = ['tracking', 'goal']
# costs_to_use = ['tracking', 'energy']
# costs_to_use = ['goal', 'energy']
# costs_to_use = ['tracking']
# costs_to_use = ['goal']
# costs_to_use = ['energy']

maxIter = 50
tolerance = 0.01
####################################################

bo_tests_path = root_path + "bo/"
first_test = OneComWaypointStaticTest(bo_tests_path, right_hand_waypoints, com_starting_waypoints, costs_to_use)

# bo_solver_parameters  = {'max_iter':maxIter, 'tolfun':tolerance, 'par':0.1, 'kernel':'Matern52', 'acquisition':'EI', 'maximizer':'Direct'}
bo_solver_parameters  = {'max_iter':maxIter, 'tolfun':tolerance, 'par':10.0, 'kernel':'Matern52', 'acquisition':'LCB', 'maximizer':'Direct'}
#solver = RoboSolver(first_test, bo_solver_parameters)
solver = BayesOptSolver(first_test, bo_solver_parameters)

solver.optimize()
solver.returnSolution(show_simulation=False)

####################################################

cma_tests_path = root_path + "cma/"
#first_test = OneComWaypointStaticTest(cma_tests_path, right_hand_waypoints, com_starting_waypoints, costs_to_use)

#cma_solver_parameters = {'max_iter':maxIter, 'tolfun':tolerance, 'initial_sigma':0.1}
#solver = CmaSolver(first_test, cma_solver_parameters)

#solver.optimize()
#solver.returnSolution(show_simulation=False)

####################################################

# replayOriginalAndOptimalReachingSimulation(getSortedTestDirs(bo_tests_path)[-1])
# replayOriginalAndOptimalReachingSimulation(getSortedTestDirs(cma_tests_path)[-1])
