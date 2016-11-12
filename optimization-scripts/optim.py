from files import *
from simulate import *
import os
from bayes_opt import *
import cma

import GPy
from robo.models.gpy_model import GPyModel
from robo.acquisition.lcb import LCB
from robo.acquisition.ei import EI
from robo.maximizers.cmaes import CMAES
# from robo.maximizers.direct import Direct
# from robo.maximizers.gradient_ascent import GradientAscent
# from robo.maximizers.grid_search import GridSearch
# from robo.maximizers.scipy_optimizer import SciPyOptimizer
# from robo.maximizers.stochastic_local_search import StochasticLocalSearch



from robo.solver.bayesian_optimization import BayesianOptimization
from robo.task.base_task import BaseTask
import numpy as np

# The optimization function that we want to optimize. It gets a numpy array with shape (N,D) where N >= 1 are the number of datapoints and D are the number of features
class ReachingWithBalance(BaseTask):

    def __init__(self, root_path, right_hand_starting_waypoints, com_starting_waypoints, visualize=False):


        self.root_path = root_path
        self.right_hand_waypoints = right_hand_starting_waypoints
        self.com_waypoints = com_starting_waypoints
        self.createTrialDir()



        # if visualize:
        #     self.visualize()
        # else:
        self.optimization_iteration = 0
        self.iterateSimulation()
        self.X_init = self.getInitialX()
        self.Y_init = self.calculateTotalCost()

        print("X_init: ", self.X_init)
        print("Y_init: ", self.Y_init)
        self.X_init_original = self.X_init
        self.Y_init_original = self.Y_init

        # self.setBounds()
        self.X_lower = self.task_data[0].lower_bounds
        self.X_upper = self.task_data[0].upper_bounds
        print("Lower bounds: ", self.X_lower)
        print("Upper bounds: ", self.X_upper)

        self.X_lower_original = self.X_lower
        self.X_upper_original = self.X_upper

        super(ReachingWithBalance, self).__init__(self.X_lower, self.X_upper)

    # def setBounds(self):
    #     right_hand_bounds_x_lower = 0.0
    #     right_hand_bounds_x_upper = 0.5
    #     right_hand_bounds_y_lower = -0.8
    #     right_hand_bounds_y_upper = 0.5
    #     right_hand_bounds_z_lower = 0.0
    #     right_hand_bounds_z_upper = 0.8
    #
    #     right_hand_bounds_lower = [right_hand_bounds_x_lower, right_hand_bounds_y_lower, right_hand_bounds_z_lower]
    #     right_hand_bounds_upper = [right_hand_bounds_x_upper, right_hand_bounds_y_upper, right_hand_bounds_z_upper]
    #
    #     com_bounds_x_lower = -0.02
    #     com_bounds_x_upper = 0.06
    #     com_bounds_y_lower = -0.2
    #     com_bounds_y_upper = 0.2
    #     com_bounds_z_lower = 0.2
    #     com_bounds_z_upper = 0.6
    #
    #     com_bounds_lower = [com_bounds_x_lower, com_bounds_y_lower, com_bounds_z_lower]
    #     com_bounds_upper = [com_bounds_x_upper, com_bounds_y_upper, com_bounds_z_upper]
    #
    #     task_bounds_lower = [com_bounds_lower, right_hand_bounds_lower]
    #     task_bounds_upper = [com_bounds_upper, right_hand_bounds_upper]
    #
    #     lower_bounds = []
    #     upper_bounds = []
    #     for t, l_bnds, u_bnds in zip(self.task_data, task_bounds_lower, task_bounds_upper):
    #         lower_bounds += l_bnds * t.nMiddleWaypoints()
    #         upper_bounds += u_bnds * t.nMiddleWaypoints()
    #
    #     for lb, ub in zip(lower_bounds, upper_bounds):
    #         if lb >= ub:
    #             print("\n\n\n\n\n\n ERROR: your bounds are contradictory (lb>=ub):", lb, ">=", ub)
    #     self.X_lower = np.array(lower_bounds)
    #     self.X_upper = np.array(upper_bounds)


    def createTrialDir(self):
        self.trial_dir_name = "Test_" + getDateAndTimeString()
        self.trial_dir_path = self.root_path + self.trial_dir_name + "/"
        os.makedirs(self.trial_dir_path)

    def createIterationDir(self, isOptimal=False):
        if isOptimal:
            self.iteration_dir_name = "Optimal_Solution"
        else:
            self.iteration_dir_name = "Iteration_" + str(self.optimization_iteration).zfill(3)

        self.iteration_dir_path = self.trial_dir_path + self.iteration_dir_name + "/"
        os.makedirs(self.iteration_dir_path)
        if isOptimal:
            self.right_hand_waypoint_file_path = self.iteration_dir_path + "/rightHandWaypoints_optimal.txt"
            self.com_waypoint_file_path = self.iteration_dir_path + "/comWaypoints_optimal.txt"
        else:
            self.right_hand_waypoint_file_path = self.iteration_dir_path + "/rightHandWaypoints.txt"
            self.com_waypoint_file_path = self.iteration_dir_path + "/comWaypoints.txt"

        np.savetxt(self.right_hand_waypoint_file_path, self.right_hand_waypoints)
        np.savetxt(self.com_waypoint_file_path, self.com_waypoints)
        print("Saving com waypoints\n", self.com_waypoints, "\n to: ", self.com_waypoint_file_path)
        print("Saving right hand waypoints\n", self.right_hand_waypoints, "\n to: ", self.right_hand_waypoint_file_path)

    def iterateSimulation(self):
        print("Simulating new parameters...")
        self.createIterationDir()
        simulate(self.right_hand_waypoint_file_path, self.com_waypoint_file_path, self.iteration_dir_path, verbose=False, visual=False)
        try:
            self.task_data = getDataFromFiles(self.iteration_dir_path)
        except:
            print("Simulation failed. Re-running.")
            killProcesses()
            simulate(self.right_hand_waypoint_file_path, self.com_waypoint_file_path, self.iteration_dir_path, verbose=False, visual=False)
            self.task_data = getDataFromFiles(self.iteration_dir_path)

        self.n_tasks = len(self.task_data)
        self.optimization_iteration += 1
        print("Simulation complete.")

    def objective_function(self, x):
        self.extractTaskWaypointsFromSolutionVector(x.flatten())
        self.iterateSimulation()
        return self.calculateTotalCost()

    def playOptimalSolution(self, x):
        self.extractTaskWaypointsFromSolutionVector(x.flatten())
        print("Simulating optimal parameters...")
        self.createIterationDir(isOptimal=True)
        simulate(self.right_hand_waypoint_file_path, self.com_waypoint_file_path, self.iteration_dir_path, verbose=True, visual=True, askUserForReplay=True, goToHome=True)
        self.task_data = getDataFromFiles(self.iteration_dir_path)
        self.n_tasks = len(self.task_data)
        observed_cost = self.calculateTotalCost()
        print("Simulation complete.")
        print("Observed cost: ", observed_cost)
        return observed_cost

    def extractTaskWaypointsFromSolutionVector(self, x):
        # i = 0
        # waypoint_list = []
        # for t in self.task_data:
        #     j = i + t.nDof() * t.nMiddleWaypoints()
        #     wpts = x[i:j].reshape((t.nMiddleWaypoints(),t.nDof()))
        #     wpts = np.vstack((wpts, t.goal()))
        #     waypoint_list.append(wpts)
        #     i=j

        # self.com_waypoints = waypoint_list[0]
        # self.right_hand_waypoints = waypoint_list[1]
        self.com_waypoints = np.array([x])

    def calculateTotalCost(self):
        j_tracking = 0.0
        j_goal = 0.0
        j_energy = self.task_data[0].energyCost()
        for t in self.task_data:
            j_tracking += t.trackingCost()
            j_goal += t.goalCost()
            np.savetxt(self.iteration_dir_path +"/"+ t.name.lower() +"_j_tracking.txt", [t.trackingCost()])
            np.savetxt(self.iteration_dir_path +"/"+ t.name.lower() +"_j_goal.txt", [t.goalCost()])
            np.savetxt(self.iteration_dir_path +"/"+ t.name.lower() +"_j_energy.txt", [t.energyCost()])

        j_total = j_tracking + j_goal + j_energy

        np.savetxt(self.iteration_dir_path + "/J_tracking.txt", [j_tracking])
        np.savetxt(self.iteration_dir_path + "/J_goal.txt", [j_goal])
        np.savetxt(self.iteration_dir_path + "/J_energy.txt", [j_energy])
        np.savetxt(self.iteration_dir_path + "/J_total.txt", [j_total])

        return np.array([[j_total]])

    def getInitialX(self):
        # X = np.array([])
        # for t in self.task_data:
        #     X = np.hstack( (X, t.middleWaypointsFlattened()) )
        # return np.array([X])
        return np.array([self.task_data[0].goal()])

    def visualize(self):
        pass





def initializeRoboSolver(solver_task):
    # kernel = GPy.kern.Matern52(input_dim=solver_task.n_dims)
    kernel = GPy.kern.RBF(input_dim=solver_task.n_dims)
    model = GPyModel(kernel)

    acquisition_func = LCB(model, X_upper=solver_task.X_upper, X_lower=solver_task.X_lower, par=0.01)
    # acquisition_func = EI(model, X_upper=solver_task.X_upper, X_lower=solver_task.X_lower, par=0.1)

    maximizer = CMAES(acquisition_func, solver_task.X_lower, solver_task.X_upper)
    # maximizer = SciPyOptimizer(acquisition_func, solver_task.X_lower, solver_task.X_upper)
    # maximizer = GradientAscent(acquisition_func, solver_task.X_lower, solver_task.X_upper)

    bo = BayesianOptimization(acquisition_func=acquisition_func,
                              model=model,
                              maximize_func=maximizer,
                              task=solver_task)

    return bo


def runOptimization(robo_task, max_iter=20, solver_type="RoBO", cost_saturation=2.0):
    optimal_params = []
    optimal_cost = []
    original_cost = []

    optimization_converged = False
    if solver_type == "RoBO":
        solver = initializeRoboSolver(robo_task)

        X = robo_task.X_init
        Y = -1.0 * (robo_task.Y_init / robo_task.Y_init)
        original_cost = Y.copy()
        while (robo_task.optimization_iteration <= max_iter) and not optimization_converged:
            print("\n\nStarting iteration: ", robo_task.optimization_iteration)
            print("------------------\tCurrent Observations\t-------------------")
            print("\t\t\t\t[parameters] --> [cost]")
            np.set_printoptions(precision=3)
            i = 0
            for p,c in zip(X,Y):
                if i == np.argmax(Y):
                    print("iteration:", str(i).zfill(3), "\t", p," --> ", c, " << current best")
                else:
                    print("iteration:", str(i).zfill(3), "\t", p," --> ", c)
                i+=1
            print("---------------------------------------------------------------------")
            X_new = robo_task.retransform(solver.choose_next(X, Y))
            print("New parameters to test:\nCOM [x, y, z]\n", X_new)#.reshape(4,3)[0:2,:])
            # print("Right Hand [x, y, z]\n", X_new.reshape(4,3)[2:,:])
            Y_new = -1.0 * (robo_task.objective_function(X_new) / robo_task.Y_init)
            Y_new = np.where((Y_new < -1*cost_saturation),  -1*cost_saturation, Y_new)

            print("Actual cost after simulation: \n", Y_new)
            X = np.vstack((X, X_new))
            Y = np.vstack((Y, Y_new))

        opt_row, opt_col = np.unravel_index(np.argmax(Y), np.shape(Y))
        optimal_params = X[[opt_row], :].copy()
        #  Correct for maximization
        optimal_cost = -1*robo_task.Y_init*Y[[opt_row], :].copy()
        original_cost = -1*robo_task.Y_init*original_cost
        print("\n\n\n==================================================\n")
        print("Best solution taken from iteration ", opt_row, ": ")
        print("COM [x, y, z]\n", optimal_params)#.reshape(4,3)[0:2,:])
        # print("Right Hand [x, y, z]\n", optimal_params.reshape(4,3)[2:,:])
        print("Orignal Cost: ", original_cost, "Optimal Cost: ", optimal_cost)
        print("Testing optimal solution...")
        observed_optimal_cost = robo_task.playOptimalSolution(optimal_params)
        print("==================================================\n")

    elif solver_type == "BayesOpt":
        solver = BayesOpt(robo_task.X_init_original,  robo_task.Y_init_original, robo_task.X_lower_original, robo_task.X_upper_original, cost_saturation)
        solver = CMAES(robo_task.X_init_original,  robo_task.Y_init_original, robo_task.X_lower_original, robo_task.X_upper_original, cost_saturation)

        X_new = solver.getFirstGuess()

        while (robo_task.optimization_iteration <= max_iter) and not optimization_converged:
            print("------\nObserved costs:\n", solver.Y.flatten(), "\n-------")
            print("New parameters to test:\nCOM [x, y, z]\n", X_new.reshape(4,3)[0:2,:], '\nRight Hand [x, y, z]\n', X_new.reshape(4,3)[2:,:])
            print("--Parameter estimations--\nCost mean: ", solver.est_cost_mean, " and  variance: ", solver.est_cost_var)

            Y_new = robo_task.objective_function(X_new)
            print("Actual cost after simulation: \n", Y_new)
            X_new = solver.update(X_new, Y_new)

    elif solver_type == "cmaes":

        X1 = robo_task.X_init_original
        Y1 = robo_task.Y_init_original
        X_lowerBound = robo_task.X_lower_original
        X_upperBound = robo_task.X_upper_original

        X_range = X_upperBound - X_lowerBound
        X1_norm = (X1 - X_lowerBound)/X_range
        cmaes_solver = cma.CMAEvolutionStrategy((X), 0.5)

        while not cmaes_solver.stop() and (robo_task.optimization_iteration <= max_iter):
            solutions = cmaes_solver.ask()
            costs = []
            for s in solutions:
                cost = robo_task.objective_function(np.array(s))
                cost_norm = cost/Y1
                cost_norm = np.where((y_norm>cost_saturation), cost_saturation, y_norm)
                costs.append(cost_norm)

            cmaes_solver.tell(solutions, costs)
            cmaes_solver.logger.add()  # write data to disc to be plotted
            cmaes_solver.disp()

        cmaes_solver.result_pretty()
        cma.plot()  # shortcut for cmaes_solver.logger.plot()
        plt.show(block=True)
