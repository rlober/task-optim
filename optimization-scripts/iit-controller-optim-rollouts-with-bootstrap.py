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

    test_02_data_path = root_path + "/test_02_no_bootstrap_13_april_11:31/"
    test_03_data_path = root_path + "/test_03_no_bootstrap_par_0.01_13_april_12:44/"

    test_02_opt_data = pickle.load( open(os.path.join(test_02_data_path, 'opt_data.pickle'), 'rb' ) )
    test_03_opt_data = pickle.load( open(os.path.join(test_03_data_path, 'opt_data.pickle'), 'rb' ) )

    test_02_X = test_02_opt_data['X']
    test_02_Y = test_02_opt_data['Y']
    test_02_Y_init = test_02_opt_data['Y_init']
    test_02_Y_raw = test_02_Y * test_02_Y_init

    test_03_X = test_03_opt_data['X']
    test_03_Y = test_03_opt_data['Y']
    test_03_Y_init = test_03_opt_data['Y_init']
    test_03_Y_raw = test_03_Y * test_03_Y_init

    Y_init = max([test_02_Y_init, test_03_Y_init])

    X = np.vstack((test_02_X, test_03_X))
    Y_raw = np.vstack((test_02_Y_raw, test_03_Y_raw))

    # X = np.vstack((test_02_X, test_02_opt_data['optimal_params'], test_03_X, test_03_opt_data['optimal_params']))
    # Y_raw = np.vstack((test_02_Y_raw, test_02_opt_data['observed_optimal_cost'], test_03_Y_raw, test_03_opt_data['observed_optimal_cost']))

    Y = Y_raw / Y_init

    opt_row, opt_col = np.unravel_index(np.argmin(Y), np.shape(Y))
    optimal_params = X[[opt_row], :].copy()

    ####################################################

    np.set_printoptions(precision=4)

    print("============== Test 02 Data ==============")
    for p,c,r in zip(test_02_X, test_02_Y, test_02_Y_raw):
        print(p, " --> %4.3f \t raw: %6.3f" % (c, r))
    print("Y_init = %4.3f" % (test_02_Y_init))

    print("============== Test 03 Data ==============")
    for p,c,r in zip(test_03_X, test_03_Y, test_03_Y_raw):
        print(p, " --> %4.3f \t raw: %6.3f" % (c, r))
    print("Y_init = %4.3f" % (test_03_Y_init))

    print("============== Combined Test Data ==============")
    print("Y_init = %4.3f" % Y_init)
    for i, (p,c) in enumerate(zip(X,Y)):
        if i == np.argmin(Y):
            print(p, " --> %4.3f" % (c), "<< optimum")
        else:
            print(p, " --> %4.3f" % (c))

    ####################################################

    com_starting_waypoints = optimal_params

    print("\n\ncom_starting_waypoints:", com_starting_waypoints)

    orig_bounds = np.array([[0.0, 0.16], [-0.18, -0.02], [0.05, 0.3]])

    box_size = 0.05
    lb = (com_starting_waypoints - box_size)
    ub = (com_starting_waypoints + box_size)

    lb = np.where(lb<orig_bounds[:,0], orig_bounds[:,0], lb)
    ub = np.where(ub>orig_bounds[:,1], orig_bounds[:,1], ub)

    lb = np.reshape(lb, (3,1))
    ub = np.reshape(ub, (3,1))
    bounds = np.hstack((lb,ub))

    print("\n\n==================================")
    print("Original Bounds:\n", orig_bounds)
    print("----------------------------------")
    print("Rollout Bounds:\n", bounds)
    print("==================================")

    ####################################################


    bo_solver_parameters  = {'max_iter':10, 'tolfun':0.01, 'par':0.01, 'length_scale':1, 'length_scale_bounds':(1e-2, 1e10), 'nu':(6./2.), 'max_sigma':0.1, 'adaptive_par':False}


    ####################################################

    costs_used = pickle.load( open(os.path.join(test_02_data_path, 'costs_used.pickle'), 'rb' ) )
    test = IitStandUpTest(root_path, com_starting_waypoints, costs_used, using_real_robot=True, com_bounds=bounds)

    X = np.vstack((X, test.X_init))
    Y = np.vstack((Y, (test.Y_init / Y_init)))

    test.X_init = X[[0],:]
    test.Y_init = Y_init
    test.X_init_original = test.X_init
    test.Y_init_original = test.Y_init

    solver = BayesOptSolver(test, bo_solver_parameters, X, Y)
    solver.optimize()
    solver.returnSolution(show_simulation=False)
