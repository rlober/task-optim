import numpy as np
import os
from task_optim.test_scenario.iit_stand_up_test import IitStandUpTest
from task_optim.sim_tools.simulate import GazeboSimulation

if __name__ == "__main__":
    home_path = os.path.expanduser("~")
    root_path = home_path + "/corl_2017_data/simulation_optimization_data_02/"

    ####################################################

    # com_starting_waypoints = np.array([[0.064, -0.08, 0.25]]) # Straight line
    com_starting_waypoints = np.array([[0.070002, -0.074316,  0.065696]]) # Optimal
    costs_to_use = ['tracking', 'goal', 'energy']


    ####################################################

    first_test = IitStandUpTest(root_path, com_starting_waypoints, costs_to_use)
    import sys
    sys.exit()

    dirs = [os.path.join(root_path,o) for o in os.listdir(root_path) if os.path.isdir(os.path.join(root_path,o))]
    dirs.sort()
    iteration_dirs = dirs[:-1]

    start_idx = 12
    for i, it_dir in enumerate(iteration_dirs[start_idx:]):

        print("Running trial from:")
        print(it_dir)
        # Get waypoints
        first_test.com_waypoints = np.loadtxt(it_dir+"/comWaypoints.txt")[1,:].reshape(1,3)
        # Set iteration_dir path and number
        first_test.iteration_dir_path = it_dir
        first_test.optimization_iteration = i + start_idx
        print("first_test.iteration_dir_path", first_test.iteration_dir_path)
        print("first_test.optimization_iteration", first_test.optimization_iteration)
        first_test.createIterationDir()
        first_test.trySimulation()
