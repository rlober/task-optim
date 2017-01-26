import numpy as np
import os
from task_optim.test_scenario.one_com_waypoint_static_test import OneComWaypointStaticTest
from task_optim.solvers.robo_solver import RoboSolver
from task_optim.solvers.cma_solver import CmaSolver
from task_optim.solvers.bayes_opt_solver import BayesOptSolver


root_path = os.path.join(os.path.expanduser("~"), "/Optimization_Tests/reaching_tests_target_set_01/")

costs_to_use=['tracking', 'goal', 'energy']

com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])

rh_targets_file_path = os.path.join(os.path.expanduser("~"), "/Code/bayesian-task-optimization/right_hand_targets_set_01.txt")

rh_targets = np.loadtxt(rh_targets_file_path)
######################################################################################

bo_solver_parameters  = {'max_iter':50, 'tolfun':0.01, 'par':10.0}
bo_test_path = root_path + "/bo/"

cma_solver_parameters  = {'max_iter':100, 'tolfun':1e-15, 'par':0.1}
cma_test_path = root_path + "/cma/"


######################################################################################

print("\n\n\n\n\n\n=================================================")
print("Random Right Hand Target Tests.")
print("=================================================\n\n\n")

for i,t in enumerate(rh_targets):
    print("\n\n========================================")
    print("Test number:", i+1, "of", n_samples, "tests.\nRight hand target:", t)
    print("========================================\n\n")

    trial_dir_name = "target_"+str(i).zfill(4)

    first_test = OneComWaypointStaticTest(bo_test_path, np.array([t]), com_starting_waypoints, costs_to_use, trial_dir=trial_dir_name)
    solver = BayesOptSolver(first_test, bo_solver_parameters)
    solver.optimize()
    solver.returnSolution(show_simulation=False)

    first_test = OneComWaypointStaticTest(cma_test_path, np.array([t]), com_starting_waypoints, costs_to_use, trial_dir=trial_dir_name)
    solver = CmaSolver(first_test, cma_solver_parameters)
    solver.optimize()
    solver.returnSolution(show_simulation=False)
