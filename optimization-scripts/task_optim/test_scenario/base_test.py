from ..utils.files import *
from ..sim_tools.simulate import *
import os
from robo.task.base_task import BaseTask
import numpy as np
import pickle


class BaseTest(BaseTask):

    def __init__(self, root_path, costs=['tracking', 'goal', 'energy'], skip_init=False, trial_dir=None, cost_weights=None):

        head, tail = os.path.split(os.path.abspath(__file__))
        self.rootPath = os.path.abspath(head+"/../../../")

        self.costs = costs
        self.useTrackingCost = False
        self.useGoalCost = False
        self.useEnergyCost = False

        self.weights_used = {}

        for c in self.costs:
            if c == 'tracking':
                self.useTrackingCost = True
                if cost_weights is not None:
                    if 'tracking' in cost_weights:
                        self.tracking_cost_weight = cost_weights['tracking']
                    else:
                        self.tracking_cost_weight = 1.0
                else:
                    self.tracking_cost_weight = 1.0

                self.weights_used['tracking'] = self.tracking_cost_weight

            if c == 'goal':
                self.useGoalCost = True
                if cost_weights is not None:
                    if 'goal' in cost_weights:
                        self.goal_cost_weight = cost_weights['goal']
                    else:
                        self.goal_cost_weight = 1.0
                else:
                    self.goal_cost_weight = 1.0

                self.weights_used['goal'] = self.goal_cost_weight

            if c == 'energy':
                self.useEnergyCost = True
                if cost_weights is not None:
                    if 'energy' in cost_weights:
                        self.energy_cost_weight = cost_weights['energy']
                    else:
                        self.energy_cost_weight = 1.0
                else:
                    self.energy_cost_weight = 1.0

                self.weights_used['energy'] = self.energy_cost_weight


        if not skip_init:
            self.root_path = root_path

            print("Using the following costs:", self.costs)

            if trial_dir is not None:
                self.trial_dir_name = trial_dir
                self.trial_dir_path = os.path.join(self.root_path, self.trial_dir_name)
                if os.path.exists(self.trial_dir_path):
                    print("WARNING: The specified trial_dir --", self.trial_dir_path, "-- already exists and will be overwritten.")
                else:
                    os.makedirs(self.trial_dir_path)
            else:
                self.createTrialDir()

            self.costs_used_pickle_path = os.path.join(self.trial_dir_path, "costs_used.pickle")
            pickle.dump(self.costs, open( self.costs_used_pickle_path, "wb" ) )

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

        else:
            self.trial_dir_path = trial_dir
            self.initSkipMethod()

    """
    Implement these methods...
    """
    def initSkipMethod(self):
        pass

    def getInitialX(self):
        pass

    def extractTaskWaypointsFromSolutionVector(self, x):
        pass

    def setBounds(self):
        pass

    def getControllerArgs(self):
        pass

    def getClientArgs(self, isOptimal=False):
        pass

    def getGazeboWorld(self):
        pass

    def saveWaypointsToFile(self, iteration_dir_path, isOptimal=False):
        pass


    """
    These methods are generic to any test.
    """
    def createTrialDir(self):
        self.trial_dir_name = self.__class__.__name__ + "_" + getDateAndTimeString()
        self.trial_dir_path = self.root_path + self.trial_dir_name + "/"
        os.makedirs(self.trial_dir_path)

    def createIterationDir(self, isOptimal=False):
        if isOptimal:
            self.iteration_dir_name = "Optimal_Solution"
        else:
            self.iteration_dir_name = "Iteration_" + str(self.optimization_iteration).zfill(3)

        self.iteration_dir_path = self.trial_dir_path + self.iteration_dir_name + "/"
        try:
            os.makedirs(self.iteration_dir_path)
        except:
            print("Warning, the dir:", self.iteration_dir_path, "already exists. Overwriting previous data.")

        self.saveWaypointsToFile(self.iteration_dir_path, isOptimal)

    def trySimulation(self):
        number_of_trials = 0
        max_trials = 20
        while number_of_trials <= max_trials:
            try:
                simulate(self.getControllerArgs(), self.getClientArgs(), self.getGazeboWorld(), self.iteration_dir_path, verbose=False, visual=False)
                self.task_data = getDataFromFiles(self.iteration_dir_path)
                break
            except:
                print("Simulation failed. Rerunning. Current number of trials rerun: "+str(number_of_trials))
                killProcesses()
                time.sleep(2.0)
                killProcesses()
                number_of_trials += 1

        if number_of_trials > max_trials:
            print("Couldn't get the simulation to work after "+str(max_trials)+". I am giving up.")

    def iterateSimulation(self):
        print("Simulating new parameters...")
        self.createIterationDir()
        self.trySimulation()
        self.n_tasks = len(self.task_data)
        self.optimization_iteration += 1
        print("Simulation complete.")

    def objective_function(self, x):
        self.extractTaskWaypointsFromSolutionVector(x.flatten())
        self.iterateSimulation()
        return self.calculateTotalCost()

    def playOptimalSolution(self, x, show_simulation=True):
        self.extractTaskWaypointsFromSolutionVector(x.flatten())
        print("Simulating optimal parameters...")
        self.createIterationDir(isOptimal=True)
        if show_simulation:
            simulate(self.getControllerArgs(), self.getClientArgs(isOptimal=True), self.getGazeboWorld(), self.iteration_dir_path, verbose=True, visual=True, askUserForReplay=True)
        else:
            self.trySimulation()

        self.task_data = getDataFromFiles(self.iteration_dir_path)
        self.n_tasks = len(self.task_data)
        observed_cost = self.calculateTotalCost()
        print("Simulation complete.")
        print("Observed cost: ", observed_cost)
        return observed_cost


    def calculateTotalCost(self):
        j_total = 0.0
        j_tracking = 0.0
        j_goal = 0.0
        j_energy = self.task_data[0].energyCost()
        for t in self.task_data:
            j_tracking += t.trackingCost()
            j_goal += t.goalCost()

        if self.useTrackingCost:
            j_total += self.tracking_cost_weight * j_tracking
        if self.useGoalCost:
            j_total += self.goal_cost_weight * j_goal
        if self.useEnergyCost:
            j_total += self.energy_cost_weight * j_energy


        return np.array([[j_total]])

    def visualize(self):
        # TODO: Implement generically.
        pass
