import numpy as np
import os
import pickle
from task_optim.test_scenario.iit_stand_up_test import IitStandUpTest
# from task_optim.utils.video import *
# from task_optim.utils.files import *
from task_optim.solvers.bayes_opt_solver import BayesOptSolver
from task_optim.sim_tools.simulate import GazeboSimulation

if __name__ == "__main__":
    home_path = os.path.expanduser("~")
    root_path = home_path + "/Optimization_Tests/iit_standing_rollouts/"

    ####################################################

    # com_starting_waypoints = np.array([[0.064, -0.08, 0.25]]) # Straight line
    # com_starting_waypoints = np.array([[0.12, -0.08, 0.16]]) # move over feet then up
    com_starting_waypoints = np.array([[0.07, -0.074, 0.066]])
    costs_to_use = ['tracking', 'goal', 'energy']

    test = IitStandUpTest(root_path, com_starting_waypoints, costs_to_use, using_real_robot=True)
