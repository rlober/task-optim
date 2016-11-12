import numpy as np
import os
from test_scenario import *
from solvers import *

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/First_Test/"

right_hand_starting_waypoints = np.array([[0.36, -0.23, 0.5]])
com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])

first_test = OneComWaypointStaticTest(root_path, right_hand_starting_waypoints, com_starting_waypoints)

solver_parameters = {'max_iter':3}

solver = RoboSolver(first_test, solver_parameters)

solver.optimize()

solver.returnSolution()
