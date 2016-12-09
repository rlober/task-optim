import numpy as np
import os
from task_optim.test_scenario.stand_up_test import StandUpTest
from task_optim.solvers.robo_solver import RoboSolver
from task_optim.solvers.cma_solver import CmaSolver

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/Test_Data/"

com_starting_waypoints = np.array([[-0.0456897, -0.0872765, 0.380022]])

costs_to_use = ['tracking', 'goal', 'energy']
# costs_to_use = ['tracking', 'goal']
# costs_to_use = ['tracking', 'energy']
# costs_to_use = ['goal', 'energy']
# costs_to_use = ['tracking']
# costs_to_use = ['goal']
# costs_to_use = ['energy']

####################################################

first_test = StandUpTest(root_path, com_starting_waypoints, costs_to_use)

solver_parameters = {'max_iter':2, 'par':0.01, 'tolfun':1e-11, 'kernel':'Matern52', 'acquisition':'LCB', 'maximizer':'CMAES'}
solver = RoboSolver(first_test, solver_parameters)

solver.optimize()
solver.returnSolution(show_simulation=False)

####################################################

first_test = StandUpTest(root_path, right_hand_starting_waypoints, com_starting_waypoints, costs_to_use)

solver_parameters = {'max_iter':2, 'initial_sigma':0.5, 'tolfun':1e-11}
solver = CmaSolver(first_test, solver_parameters)

solver.optimize()
solver.returnSolution(show_simulation=False)
