import numpy as np
import os
from task_optim.test_scenario.stand_up_test import StandUpTest
from task_optim.solvers.cma_solver import CmaSolver
from task_optim.utils.video import *
from task_optim.utils.files import *
from task_optim.solvers.bayes_opt_solver import BayesOptSolver


home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/tcfm/standing/"

####################################################

original_com_waypoints = np.array([[-0.0456897, -0.0872765, 0.380022]])
optimal_com_waypoints = np.array([[1.463701214163619369e-02, -1.238322207935628055e-01, 2.207297705684966582e-01]])
costs_to_use = ['tracking', 'goal', 'energy']

####################################################

# first_test = StandUpTest(root_path, original_com_waypoints, costs_to_use)

####################################################

first_test = StandUpTest(root_path, optimal_com_waypoints, costs_to_use)

####################################################
