import numpy as np
from optim import *
import os

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/"

right_hand_starting_waypoints = np.array([[0.36, -0.23, 0.5]])
com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])

robo_task = ReachingWithBalance(root_path, right_hand_starting_waypoints, com_starting_waypoints, True)



st = "RoBO"
# st = "BayesOpt"
# st = "cmaes"
runOptimization(robo_task, max_iter=10, solver_type=st, cost_saturation=1000.0)
