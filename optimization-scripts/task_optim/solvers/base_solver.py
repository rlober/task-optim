import numpy as np
import pickle

class BaseSolver(object):
    """docstring for BaseSolver."""
    def __init__(self, test, solver_parameters, X=None, Y=None):

        self.test = test
        self.solver_parameters = solver_parameters

        self.base_pickle_path = self.test.trial_dir_path
        self.param_pickle_path = self.base_pickle_path + "/solver_parameters.pickle"
        pickle.dump(self.solver_parameters, open( self.param_pickle_path, "wb" ) )

        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("Initializing "+self.__class__.__name__+" with the following parameters:")
        print(self.solver_parameters)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        if X is None or Y is None:
            self.X = self.test.X_init
            self.Y = (self.test.Y_init / self.test.Y_init)
            self.original_cost = self.Y.copy()
            self.using_existing_data = False
        else:
            self.X = X
            self.Y = Y
            self.original_cost = self.Y[[0],:]
            self.using_existing_data = True


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

    def returnSolution(self, show_simulation=True):
        self.opt_row, opt_col = np.unravel_index(np.argmin(self.Y), np.shape(self.Y))
        self.optimal_params = self.X[[self.opt_row], :].copy()
        #  Correct for maximization
        self.optimal_cost = self.test.Y_init*self.Y[[self.opt_row], :].copy()
        self.original_cost = self.test.Y_init*self.original_cost
        print("\n\n==============================================")
        print("\tOptimization Complete!")
        print("==============================================")
        print("Total number of iterations:", self.test.optimization_iteration-1)
        print("Best solution taken from iteration:", self.opt_row)
        print("Optimal Parameters:\n", self.optimal_params)
        print("Orignal Cost:", self.original_cost, "Optimal Cost:", self.optimal_cost)

        self.compileAndSaveOptimizationData()

        print("Testing optimal solution...\n")
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        self.observed_optimal_cost = self.test.playOptimalSolution(self.optimal_params, show_simulation)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("\nCost observed after test:", self.observed_optimal_cost)
        print("==============================================\n")



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

    def compileAndSaveOptimizationData(self):
        self.opt_data = {'solver':self.__class__.__name__, 'X':self.X, 'Y':self.Y, 'Y_init':self.test.Y_init, 'opt_row':self.opt_row, 'optimal_params':self.optimal_params, 'optimal_cost':self.optimal_cost, 'original_cost':self.original_cost, 'n_iter':self.test.optimization_iteration-1}

        self.opt_data_pickle_path = self.base_pickle_path + "/opt_data.pickle"
        pickle.dump(self.opt_data, open( self.opt_data_pickle_path, "wb" ) )
        print("Saved optimization data to:", self.opt_data_pickle_path)
