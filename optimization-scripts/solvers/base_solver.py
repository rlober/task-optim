import numpy as np


class BaseSolver(object):
    """docstring for BaseSolver."""
    def __init__(self, test, solver_parameters):

        self.test = test
        self.solver_parameters = solver_parameters
        self.X = self.test.X_init
        self.Y = (self.test.Y_init / self.test.Y_init)
        self.original_cost = self.Y.copy()

        self.initializeSolver()

    def initializeSolver(self):
        pass

    def updateSolver(self):
        pass

    def solverFinished(self):
        return True

    def optimize(self):
        while not self.solverFinished():
            self.printObservationInfo()
            self.updateSolver()

    def returnSolution(self):
        opt_row, opt_col = np.unravel_index(np.argmax(self.Y), np.shape(self.Y))
        self.optimal_params = self.X[[opt_row], :].copy()
        #  Correct for maximization
        self.optimal_cost = self.test.Y_init*Y[[opt_row], :].copy()
        self.original_cost = self.test.Y_init*self.original_cost
        print("\n\n==================================================")
        print("\t\t\tOptimization Complete!")
        print("==================================================")
        print("Total number of iterations:", self.test.optimization_iteration)
        print("Best solution taken from iteration:", opt_row)
        print("Optimal Parameters:\n", self.optimal_params)
        print("Orignal Cost:", self.original_cost, "Optimal Cost:", self.optimal_cost)
        print("Testing optimal solution...")
        self.observed_optimal_cost = self.test.playOptimalSolution(self.optimal_params)
        print("Cost observed after test:", self.observed_optimal_cost)
        print("==================================================\n")

    def printObservationInfo(self):
        print("\n\nStarting iteration: ", self.test.optimization_iteration)
        print("------------------\tCurrent Observations\t-------------------")
        print("\t\t\t[parameters] --> [cost]")
        np.set_printoptions(precision=3)
        i = 0
        for p,c in zip(self.X, self.Y):
            if i == np.argmin(self.Y):
                print("iteration:", str(i).zfill(3), "\t", p," --> ", c, " << current best")
            else:
                print("iteration:", str(i).zfill(3), "\t", p," --> ", c)
            i+=1
        print("---------------------------------------------------------------------")
