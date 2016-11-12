from ..files import *
from ..simulate import *
import os
from robo.task.base_task import BaseTask
import numpy as np


class BaseTest(BaseTask):

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
        self.X_init_original = self.X_init
        self.Y_init_original = self.Y_init

        self.setBounds()

        print("Lower bounds: ", self.X_lower)
        print("Upper bounds: ", self.X_upper)

        self.X_lower_original = self.X_lower
        self.X_upper_original = self.X_upper

        super(BaseTest, self).__init__(self.X_lower, self.X_upper)


    """
    Implement these methods...
    """
    def getInitialX(self):
        pass

    def extractTaskWaypointsFromSolutionVector(self, x):
        pass

    def setBounds(self):
        pass



    """
    These methods are generic to any test.
    """
    def createTrialDir(self):
        self.trial_dir_name = self.__class__.__name__ + getDateAndTimeString()
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

    def visualize(self):
        # TODO: Implement generically.
        pass
