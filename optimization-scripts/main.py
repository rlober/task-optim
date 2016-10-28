import numpy as np
from optim import *
import os

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/"

right_hand_starting_waypoints = np.array([[0.24, -0.27, 0.64],[0.30, -0.10, 0.54],[0.36,  0.00, 0.44]])
com_starting_waypoints = np.array([[0.024, -0.060, 0.500],[0.025, -0.061, 0.501],[0.026, -0.062, 0.502]])

robo_task = ReachingWithBalance(root_path, right_hand_starting_waypoints, com_starting_waypoints)

runOptimization(robo_task, max_iter=30, useRoBO=False, cost_saturation=1000.0)
