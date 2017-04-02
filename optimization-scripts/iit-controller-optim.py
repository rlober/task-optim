import numpy as np
import os
from task_optim.test_scenario.iit_stand_up_test import IitStandUpTest
# from task_optim.utils.video import *
# from task_optim.utils.files import *
from task_optim.solvers.bayes_opt_solver import BayesOptSolver
from task_optim.sim_tools.simulate import GazeboSimulation

if __name__ == "__main__":
    home_path = os.path.expanduser("~")
    root_path = home_path + "/Optimization_Tests/iit_standing/"

    ####################################################

    com_starting_waypoints = np.array([[0.0799, -0.09, 0.1501]])
    # com_starting_waypoints = np.array([[0.15, -0.09, 0.1]]) # Works
    costs_to_use = ['tracking', 'goal', 'energy']

    ####################################################

    bo_solver_parameters  = {'max_iter':2, 'tolfun':0.01, 'par':0.004, 'length_scale':1e4, 'length_scale_bounds':(1e-2, 1e10), 'nu':15.0, 'max_sigma':0.1, 'adaptive_par':False}

    ####################################################

    # GazeboSimulation("/home/ryan/Code/codyco-superbuild/main/icub-gazebo-wholebody/worlds/icub_standup_world/icub_standup_world", verbose=True)

    first_test = IitStandUpTest(root_path, com_starting_waypoints, costs_to_use)


    solver = BayesOptSolver(first_test, bo_solver_parameters)
    solver.optimize()
    solver.returnSolution(show_simulation=False)
