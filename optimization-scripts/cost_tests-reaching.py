import numpy as np
import os
from task_optim.test_scenario.one_com_waypoint_static_test import OneComWaypointStaticTest
from task_optim.solvers.robo_solver import RoboSolver
from task_optim.solvers.cma_solver import CmaSolver

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/cost_tests-reaching/"

right_hand_starting_waypoints = np.array([[0.36, -0.23, 0.5]])
com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])


bo_test_path = root_path + "/bo/"
cma_test_path = root_path + "/cma/"

tolerance = 1e-11
maxIter = 44


costs_to_use=[['tracking', 'goal', 'energy'], ['tracking', 'energy'], ['tracking', 'goal'], ['goal', 'energy'], ['tracking'], ['goal'], ['energy']]

number_of_repeats = 10

bo_solver_parameters  = {'max_iter':maxIter, 'tolfun':tolerance, 'par':1000.0, 'kernel':'Matern52', 'acquisition':'EI', 'maximizer':'Direct'}
cma_solver_parameters = {'max_iter':maxIter, 'tolfun':tolerance, 'initial_sigma':0.1}
######################################################################################

print("\n\n\n\n\n\n=================================================")
print("Cost Tests: Reaching Experiment.")
print("=================================================\n\n\n")

for c in costs_to_use:
    for t in range(number_of_repeats):
        print("\n\n========================================")
        print("Test number:", t+1, "of", number_of_repeats, "tests.\nCosts to use:", c)
        print("========================================")

        print("\n\nUsing Bayesian Optimization...\n\n")
        first_test = OneComWaypointStaticTest(bo_test_path, right_hand_starting_waypoints, com_starting_waypoints, c)
        solver = RoboSolver(first_test, bo_solver_parameters)
        solver.optimize()
        solver.returnSolution(show_simulation=False)

        print("\n\nUsing CMA-ES...\n\n")
        first_test = OneComWaypointStaticTest(cma_test_path, right_hand_starting_waypoints, com_starting_waypoints, costs_to_use)
        solver = CmaSolver(first_test, cma_solver_parameters)
        solver.optimize()
        solver.returnSolution(show_simulation=False)
