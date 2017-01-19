import numpy as np
import os
from task_optim.test_scenario.stand_up_test import StandUpTest
from task_optim.solvers.robo_solver import RoboSolver
from task_optim.solvers.cma_solver import CmaSolver

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/parameter_tests-standing/"

com_starting_waypoints = np.array([[-0.0456897, -0.0872765, 0.380022]])

bo_test_path = root_path + "/bo/"
cma_test_path = root_path + "/cma/"

tolerance = 1e-11
maxIter = 44

bo_test_number = 1
cma_test_number = 1


costs_to_use = ['tracking', 'goal', 'energy']

######################################################################################

# RoBO Parameters to test:

pars = [0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0]
kernels = ['RBF', 'Matern52']
acquisitionFunctions = ['EI', 'LogEI', 'LCB']
maximizers = ['Direct', 'CMAES']
# Total number of tests
bo_n_tests = len(pars) * len(kernels) * len(acquisitionFunctions) * len(maximizers)


# CMA-ES Parameters to test:

sigmas = [0.0001, 0.0001, 0.01, 0.1, 0.5, 0.8, 0.99]
# Total number of tests
cma_n_tests = len(sigmas)

######################################################################################

print("\n\n\n\n\n\n=================================================")
print("Begining Bayesian Optimization trials.")
print("=================================================")

for p in pars:
    for k in kernels:
        for a in acquisitionFunctions:
            for m in maximizers:
                print("\n\n========================================")
                print("Test number:", bo_test_number, "of", bo_n_tests, "tests.")
                print("========================================")
                first_test = StandUpTest(bo_test_path, com_starting_waypoints, costs_to_use)
                solver_parameters = {'max_iter':maxIter, 'tolfun':tolerance, 'par':p, 'kernel':k, 'acquisition':a, 'maximizer':m}
                solver = RoboSolver(first_test, solver_parameters)
                solver.optimize()
                solver.returnSolution(show_simulation=False)
                bo_test_number += 1

######################################################################################

print("\n\n\n\n\n\n=================================================")
print("Begining CMA-ES trials.")
print("=================================================")

for s in sigmas:
    print("\n\n========================================")
    print("Test number:", cma_test_number, "of", cma_n_tests, "tests.")
    print("========================================")
    first_test = StandUpTest(cma_test_path, com_starting_waypoints, costs_to_use)
    solver_parameters = {'max_iter':maxIter, 'tolfun':tolerance, 'initial_sigma':s}
    solver = CmaSolver(first_test, solver_parameters)
    solver.optimize()
    solver.returnSolution(show_simulation=False)
    cma_test_number += 1
