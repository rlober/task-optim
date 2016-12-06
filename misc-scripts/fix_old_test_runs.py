import pickle
import os
import sys
import re
sys.path.append("../optimization-scripts")
from task_optim.utils.files import *
from task_optim.test_scenario.one_com_waypoint_static_test import OneComWaypointStaticTest

sys.path.append("../plotting-scripts")
from one_com_waypoint_static.plot import *

root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests/parameter_tests')

sub_tests_dir = ['bo', 'cma']

for s in sub_tests_dir:


    root_tests_dir = os.path.join(root_dir, s)

    dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]

    test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match('OneComWaypointStaticTest.*', d)])

    for i, test in enumerate(test_dirs):

        print("Fixing test", i+1, "of", len(test_dirs))

        dirs = [d for d in os.listdir(test) if os.path.isdir(os.path.join(test, d))]

        iter_dirs = sorted([os.path.join(test, d) for d in dirs if re.match('Iteration_.*', d)])
        orig_dir = iter_dirs[0]

        orig_com_task_data, orig_rh_task_data = getDataFromFiles(orig_dir)

        # save costs used
        costs_used_path = os.path.join(test, 'costs_used.pickle')
        costs_used = ['tracking', 'goal', 'energy']
        pickle.dump(costs_used, open( costs_used_path, "wb" ) )


        tmp = DataPlots([orig_com_task_data, orig_rh_task_data], costs_used, 1.0)

        # Convert opt_data
        opt_data_path = os.path.join(test, 'opt_data.pickle')
        opt_data = pickle.load( open( opt_data_path, 'rb' ) )

        if type(opt_data)==list:
            # Old format:
            # self.opt_data = [self.__class__.__name__, self.X, self.Y, self.opt_row, self.optimal_params, self.optimal_cost, self.original_cost]
            solver = opt_data[0]
            X = opt_data[1]
            Y = opt_data[2]
            opt_row = opt_data[3]
            optimal_params = opt_data[4]
            optimal_cost = opt_data[5]
            original_cost = opt_data[6]

            Y_init = tmp.total_cost.sum()
            n_iter = len(iter_dirs)

            new_opt_data = {'solver':solver, 'X':X, 'Y':Y, 'Y_init':Y_init, 'opt_row':opt_row, 'optimal_params':optimal_params, 'optimal_cost':optimal_cost, 'original_cost':original_cost, 'n_iter':n_iter}

            pickle.dump(new_opt_data, open( opt_data_path, "wb" ) )
        else:
            optimal_params = opt_data['optimal_params']


        root_path = ''
        right_hand_starting_waypoints = np.array([[0.36, -0.23, 0.5]])
        com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])
        tmp_test = OneComWaypointStaticTest(root_path, right_hand_starting_waypoints, com_starting_waypoints, costs_used, skip_init=True, trial_dir=test+'/')
        tmp_test.playOptimalSolution(optimal_params, False)
