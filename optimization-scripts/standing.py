import numpy as np
import os
from task_optim.test_scenario.stand_up_test import StandUpTest
from task_optim.solvers.cma_solver import CmaSolver
from task_optim.utils.video import *
from task_optim.utils.files import *
from task_optim.solvers.bayes_opt_solver import BayesOptSolver


home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/standing/"

####################################################

com_starting_waypoints = np.array([[-0.0456897, -0.0872765, 0.380022]])
costs_to_use = ['tracking', 'goal', 'energy']

####################################################

bo_solver_parameters  = {'max_iter':50, 'tolfun':0.01, 'par':10.0}
bo_test_path = root_path + "/bo/"

cma_solver_parameters  = {'max_iter':100, 'tolfun':1e-15, 'par':0.1}
cma_test_path = root_path + "/cma/"

####################################################

first_test = StandUpTest(bo_test_path, com_starting_waypoints, costs_to_use)
solver = BayesOptSolver(first_test, bo_solver_parameters)
solver.optimize()
solver.returnSolution(show_simulation=False)

####################################################

# first_test = StandUpTest(cma_test_path, com_starting_waypoints, costs_to_use)
# solver = CmaSolver(first_test, cma_solver_parameters)
# solver.optimize()
# solver.returnSolution(show_simulation=False)

####################################################

replayOriginalAndOptimalStandingSimulation(getSortedTestDirs(bo_tests_path, test_name="StandUpTest")[-1])
# replayOriginalAndOptimalStandingSimulation(getSortedTestDirs(cma_tests_path, test_name="StandUpTest")[-1])
