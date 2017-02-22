import numpy as np
import os
from task_optim.test_scenario.stand_up_test import StandUpTest
from task_optim.utils.video import *
from task_optim.utils.files import *
from task_optim.solvers.bayes_opt_solver import BayesOptSolver


home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/standing_with_force/"

####################################################

com_starting_waypoints = np.array([[-0.0456897, -0.0872765, 0.380022]])
costs_to_use = ['tracking', 'goal', 'energy']

####################################################

bo_solver_parameters  = {'max_iter':20, 'tolfun':0.01, 'par':0.004, 'length_scale':1e4, 'length_scale_bounds':(1e-2, 1e10), 'nu':15.0, 'max_sigma':0.1, 'adaptive_par':False}

####################################################

first_test = StandUpTest(root_path, com_starting_waypoints, costs_to_use, apply_force=True)
solver = BayesOptSolver(first_test, bo_solver_parameters)
solver.optimize()
solver.returnSolution(show_simulation=True)
