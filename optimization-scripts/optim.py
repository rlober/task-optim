from files import *
from simulate import *
import os


import GPy
from robo.models.gpy_model import GPyModel
from robo.acquisition.lcb import LCB
from robo.maximizers.cmaes import CMAES
from robo.maximizers.direct import Direct
from robo.maximizers.gradient_ascent import GradientAscent
from robo.maximizers.grid_search import GridSearch
from robo.maximizers.scipy_optimizer import SciPyOptimizer
from robo.maximizers.stochastic_local_search import StochasticLocalSearch



from robo.solver.bayesian_optimization import BayesianOptimization
from robo.task.base_task import BaseTask
import numpy as np

# The optimization function that we want to optimize. It gets a numpy array with shape (N,D) where N >= 1 are the number of datapoints and D are the number of features
class ReachingWithBalance(BaseTask):

    def __init__(self, root_path, right_hand_starting_waypoints, com_starting_waypoints):
        self.root_path = root_path
        self.right_hand_waypoints = right_hand_starting_waypoints
        self.com_waypoints = com_starting_waypoints
        self.createTrialDir()

        self.optimization_iteration = 0
        self.iterateSimulation()
        self.X_init = self.getInitialX()
        self.Y_init = self.calculateTotalCost()

        print("X_init: ", self.X_init)
        print("Y_init: ", self.Y_init)

        self.setBounds()
        print("Lower bounds: ", self.X_lower)
        print("Upper bounds: ", self.X_upper)
#        super(ReachingWithBalance, self).__init__(self.X_lower, self.X_upper)

    def setBounds(self):
        right_hand_bounds_x_lower = 0.0
        right_hand_bounds_x_upper = 0.5
        right_hand_bounds_y_lower = -0.8
        right_hand_bounds_y_upper = 0.5
        right_hand_bounds_z_lower = 0.0
        right_hand_bounds_z_upper = 0.8

        right_hand_bounds_lower = [right_hand_bounds_x_lower, right_hand_bounds_y_lower, right_hand_bounds_z_lower]
        right_hand_bounds_upper = [right_hand_bounds_x_upper, right_hand_bounds_y_upper, right_hand_bounds_z_upper]

        com_bounds_x_lower = -0.02
        com_bounds_x_upper = 0.06
        com_bounds_y_lower = -0.2
        com_bounds_y_upper = 0.2
        com_bounds_z_lower = 0.2
        com_bounds_z_upper = 0.6

        com_bounds_lower = [com_bounds_x_lower, com_bounds_y_lower, com_bounds_z_lower]
        com_bounds_upper = [com_bounds_x_upper, com_bounds_y_upper, com_bounds_z_upper]

        task_bounds_lower = [com_bounds_lower, right_hand_bounds_lower]
        task_bounds_upper = [com_bounds_upper, right_hand_bounds_upper]

        lower_bounds = []
        upper_bounds = []
        for t, l_bnds, u_bnds in zip(self.task_data, task_bounds_lower, task_bounds_upper):
            lower_bounds += l_bnds * t.nMiddleWaypoints()
            upper_bounds += u_bnds * t.nMiddleWaypoints()

        for lb, ub in zip(lower_bounds, upper_bounds):
            if lb >= ub:
                print("\n\n\n\n\n\n ERROR: your bounds are contradictory (lb>=ub):", lb, ">=", ub)
        self.X_lower = np.array(lower_bounds)
        self.X_upper = np.array(upper_bounds)

    def createTrialDir(self):
        self.trial_dir_name = "Test_" + getDateAndTimeString()
        self.trial_dir_path = self.root_path + self.trial_dir_name + "/"
        os.makedirs(self.trial_dir_path)

    def createIterationDir(self):
        self.iteration_dir_name = "Iteration_" + str(self.optimization_iteration).zfill(3)
        self.iteration_dir_path = self.trial_dir_path + self.iteration_dir_name + "/"
        os.makedirs(self.iteration_dir_path)
        self.right_hand_waypoint_file_path = self.iteration_dir_path + "/rightHandWaypoints.txt"
        self.com_waypoint_file_path = self.iteration_dir_path + "/comWaypoints.txt"
        np.savetxt(self.right_hand_waypoint_file_path, self.right_hand_waypoints)
        np.savetxt(self.com_waypoint_file_path, self.com_waypoints)

    def iterateSimulation(self):
        print("Simulating new parameters...")
        self.createIterationDir()
        simulate(self.right_hand_waypoint_file_path, self.com_waypoint_file_path, self.iteration_dir_path)
        self.task_data = getDataFromFiles(self.iteration_dir_path)
        self.n_tasks = len(self.task_data)
        self.optimization_iteration += 1
        print("Simulation complete.")

    def objective_function(self, x):
        self.extractTaskWaypointsFromSolutionVector(x.flatten())
        self.iterateSimulation()
        return self.calculateTotalCost()

    def extractTaskWaypointsFromSolutionVector(self, x):
        i = 0
        waypoint_list = []
        for t in self.task_data:
            j = i + t.nDof() * t.nMiddleWaypoints()
            wpts = x[i:j].reshape((t.nMiddleWaypoints(),t.nDof()))
            wpts = np.vstack((wpts, t.goal()))
            waypoint_list.append(wpts)
            i=j

        self.com_waypoints = waypoint_list[0]
        self.right_hand_waypoints = waypoint_list[1]

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
        X = np.array([])
        for t in self.task_data:
            X = np.hstack( (X, t.middleWaypointsFlattened()) )
        return np.array([X])




def initializeRoboSolver(solver_task):
    # kernel = GPy.kern.Matern52(input_dim=solver_task.n_dims)
    kernel = GPy.kern.RBF(input_dim=solver_task.n_dims)
    model = GPyModel(kernel)

    acquisition_func = LCB(model,
                         X_upper=solver_task.X_upper,
                         X_lower=solver_task.X_lower,
                         par=0.1)

    # maximizer = CMAES(acquisition_func, solver_task.X_lower, solver_task.X_upper)
    # maximizer = SciPyOptimizer(acquisition_func, solver_task.X_lower, solver_task.X_upper)
    maximizer = GradientAscent(acquisition_func, solver_task.X_lower, solver_task.X_upper)

    bo = BayesianOptimization(acquisition_func=acquisition_func,
                              model=model,
                              maximize_func=maximizer,
                              task=solver_task)

    return bo


# print bo.choose_next()
