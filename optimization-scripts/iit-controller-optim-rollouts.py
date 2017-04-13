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


    data_set = 2


    sim_optim_data_path = root_path + "/simulation_optimization_data_01/"
    bo_solver_parameters  = {'max_iter':10, 'tolfun':0.01, 'par':0.01, 'length_scale':1, 'length_scale_bounds':(1e-2, 1e10), 'nu':(6./2.), 'max_sigma':0.1, 'adaptive_par':True}

    if data_set == 2:
        sim_optim_data_path = root_path + "/simulation_optimization_data_02/"
        bo_solver_parameters  = {'max_iter':10, 'tolfun':0.01, 'par':0.1, 'length_scale':1, 'length_scale_bounds':(1e-2, 1e10), 'nu':(6./2.), 'max_sigma':0.1, 'adaptive_par':False}


    opt_data = pickle.load( open(os.path.join(sim_optim_data_path, 'opt_data.pickle'), 'rb' ) )
    costs_used = pickle.load( open(os.path.join(sim_optim_data_path, 'costs_used.pickle'), 'rb' ) )
    # cost_weights = pickle.load( open(os.path.join(sim_optim_data_path, 'cost_weights.pickle'), 'rb' ) )
    # solver_parameters = pickle.load( open(os.path.join(sim_optim_data_path, 'solver_parameters.pickle'), 'rb' ) )



    # self.test.Y_init = opt_data['Y_init']
    # self.opt_row = opt_data['opt_row']
    # self.optimal_params = opt_data['optimal_params']
    # self.optimal_cost = opt_data['optimal_cost']
    # self.original_cost = opt_data['original_cost']
    # self.test.optimization_iteration-1 = opt_data['n_iter']

    ####################################################

    com_starting_waypoints = opt_data['optimal_params']

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

    # bo_solver_parameters  = {'max_iter':10, 'tolfun':0.01, 'par':0.0002, 'length_scale':1, 'length_scale_bounds':(1e-2, 1e10), 'nu':(6./2.), 'max_sigma':0.1, 'adaptive_par':True}
    # bo_solver_parameters  = {'max_iter':30, 'tolfun':0.01, 'par':0.01, 'length_scale':1, 'length_scale_bounds':(1e-2, 1e10), 'nu':(6./2.), 'max_sigma':0.1, 'adaptive_par':False}

    ####################################################



    test = IitStandUpTest(root_path, com_starting_waypoints, costs_used, using_real_robot=True, com_bounds=bounds)
    X = opt_data['X']
    Y = opt_data['Y']

    X = np.vstack((X, test.X_init))
    Y = np.vstack((Y, (test.Y_init / opt_data['Y_init'])))

    test.X_init = opt_data['X'][[0],:]
    test.Y_init = opt_data['Y_init']
    test.X_init_original = test.X_init
    test.Y_init_original = test.Y_init


    solver = BayesOptSolver(test, bo_solver_parameters, X, Y)
    solver.optimize()
    solver.returnSolution(show_simulation=False)
