import numpy as np
from files import *
from simulate import *
import os

home_path = os.path.expanduser("~")

rightHandStartingWaypoints = np.array([[0.24, -0.27, 0.64],[0.30, -0.10, 0.54],[0.36,  0.00, 0.44]])
comStartingWaypoints = np.array([[0.024, -0.060, 0.500],[0.025, -0.061, 0.501],[0.026, -0.062, 0.502]])



root_path = home_path + "/Optimization_Tests/"
trial_dir_name = "Test_" + getDateAndTimeString()
trial_dir_path = root_path + trial_dir_name + "/"
os.makedirs(trial_dir_path)


optimization_iteration = 0
iteration_dir_name = "Iteration_" + str(optimization_iteration).zfill(3)
iteration_dir_path = trial_dir_path + iteration_dir_name + "/"
os.makedirs(iteration_dir_path)

pathToRightHandWptFile = iteration_dir_path + "/rightHandWaypoints.txt"
pathToComWptFile = iteration_dir_path + "/comWaypoints.txt"

np.savetxt(pathToRightHandWptFile, rightHandStartingWaypoints)
np.savetxt(pathToComWptFile, comStartingWaypoints)

executeWaypoints(pathToRightHandWptFile, pathToComWptFile, iteration_dir_path)

taskData = getDataFromFiles(iteration_dir_path)

for t in taskData:
    t.printData()
