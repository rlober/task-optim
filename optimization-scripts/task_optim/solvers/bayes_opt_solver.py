from .base_solver import BaseSolver
import numpy as np
from sklearn import gaussian_process
import cma

class BayesOptSolver(BaseSolver):
    """docstring for BayesOptSolver."""
    def __init__(self, test, solver_parameters, X=None, Y=None):
        assert(solver_parameters['max_iter'] != None)

        self.par = 1.0
        if 'par' in solver_parameters:
            self.par = solver_parameters['par']

        self.length_scale = 1000.0
        if 'length_scale' in solver_parameters:
            self.length_scale = solver_parameters['length_scale']

        self.length_scale_bounds = (1e-1, 1e8)
        if 'length_scale_bounds' in solver_parameters:
            self.length_scale_bounds = solver_parameters['length_scale_bounds']

        self.nu = 10.0
        if 'nu' in solver_parameters:
            self.nu = solver_parameters['nu']

        self.max_sigma = 0.5
        if 'max_sigma' in solver_parameters:
            self.max_sigma = solver_parameters['max_sigma']

        self.n_restarts_optimizer = 10
        if 'n_restarts_optimizer' in solver_parameters:
            self.n_restarts_optimizer = solver_parameters['n_restarts_optimizer']

        self.adaptive_par = False
        if 'adaptive_par' in solver_parameters:
            self.adaptive_par = solver_parameters['adaptive_par']



        super(BayesOptSolver, self).__init__(test, solver_parameters, X, Y)

    def initializeSolver(self):
        # self.kernel = 0.5*gaussian_process.kernels.Matern(length_scale=100.0, length_scale_bounds=(1e-1, 1e6), nu=16.0)
        self.kernel = self.max_sigma * gaussian_process.kernels.Matern(length_scale=self.length_scale, length_scale_bounds=self.length_scale_bounds, nu=self.nu)

        self.gp = gaussian_process.GaussianProcessRegressor(self.kernel, n_restarts_optimizer=self.n_restarts_optimizer)

    def updateSolver(self):
        # Save old incumbent
        self.old_incumbent = self.getIncumbent()[0]
        # Put X 0-1
        X_scaled = self.test.transform(self.X)
        # Get the next solution to test
        next_solution_to_test = self.chooseNext(X_scaled, self.Y)
        # The provided solution is scaled bectween 0-1 so we retransform it back to physical coordinates
        X_new = self.test.retransform(next_solution_to_test)

        thresh = 2.0
        skip_this_solution_candidate = False
        if self.test.using_real_robot:
            print("\n===================================================\n")
            print("Current CoM waypoint to test:")
            print(X_new)
            print("\n===================================================\n")
            usr_input = input("Should we try this solution? ([y]/n):")
            if usr_input == '' or usr_input == 'y' or usr_input == 'Y':
                print("Trying...")
            else:
                print("Skipping...")
                skip_this_solution_candidate = True

        if skip_this_solution_candidate:
            self.test.optimization_iteration += 1
            Y_new = np.array([[1.0]])
        else:
            # test X_new
            Y_new = self.test.objective_function(X_new)
            # scale the cost wrt the original cost
            Y_new = ( Y_new / self.test.Y_init )
            if Y_new > thresh:
                Y_new = np.array([[thresh]])

        print('nextSolutionToTest:\n', X_new, '\nPredicted cost mean and variance:', self.gp.predict(next_solution_to_test, return_std=True), '\nLCB:', self.LCB(next_solution_to_test))
        print('getIncumbent:\n', self.getIncumbent())
        self.X = np.vstack((self.X, X_new))
        self.Y = np.vstack((self.Y, Y_new))
        if self.adaptive_par:
            self.par *= 0.1
            print('par:', self.par)


    def chooseNext(self, X, Y):
        self.gp.fit(X, Y)

        start_point = np.random.rand(1, X.shape[1])

        # self.verbose_level = -9
        self.verbose_level = 0
        self.n_func_evals = 10000
        self.restarts = 1
        res = cma.fmin(self.LCB,
                x0=start_point[0],
                sigma0=0.8,
                restarts=self.restarts,
                options={"bounds": [self.test.X_lower, self.test.X_upper],
                         "verbose": self.verbose_level,
                         "maxfevals": self.n_func_evals})

        if res[0] is None:
            print("CMA-ES did not find anything. \nReturn random configuration instead.")
            return start_point

        return np.array([res[0]])


    def LCB(self, X):
        if len(X.shape)==1:
            X = np.array([X])
        mean, variance = self.gp.predict(X, return_std=True)
        variance = np.reshape(variance, (variance.size, 1))
        #print('mean.shape', mean.shape)
        #print('variance.shape', variance.shape)
        lcb = (mean - self.par * np.sqrt(variance))[0,0]
        #print('lcb', lcb)
        return lcb

    def getIncumbent(self):
        opt_row, opt_col = np.unravel_index(np.argmin(self.Y), np.shape(self.Y))
        incumbent = self.X[[opt_row], :].copy()
        obeserved_cost = self.Y[[opt_row], :].copy()
        pred_cost, pred_variance = self.gp.predict(self.test.transform(incumbent), return_std=True)
        return incumbent, obeserved_cost, pred_cost, pred_variance



    def solverFinished(self):
        factor = 4
        if self.using_existing_data:
            factor = 1

        if self.test.optimization_iteration > factor:
            # check for tolerance:
            if 'tolfun' in self.solver_parameters:
                deltaSol = np.linalg.norm(self.X[-1,:] - self.old_incumbent)
                print('deltaSol:',deltaSol)
                if deltaSol <= self.solver_parameters['tolfun']:
                    print("Solution tolerance,", self.solver_parameters['tolfun'], "reached. Stopping optimization.")
                    return True
                else:
                    if ((self.test.optimization_iteration-1) % 5 == 0) and (self.test.optimization_iteration <= self.solver_parameters['max_iter']):
                        print("\n\n\n================================================\n")
                        print("You have run", (self.test.optimization_iteration-1), "optimization iterations, without converging.")
                        inc_data = self.getIncumbent()
                        print("The current best waypoint is:", inc_data[0], "with the following parameters:")
                        print("\tObserved Cost:", inc_data[1])
                        print("\tPredicted Cost:", inc_data[2])
                        print("\tPredicted Variance:", inc_data[3])
                        print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
                        inp_var = input("Do you wish to continue optimizing? [(y)/n]:")
                        if inp_var=="n" or inp_var=="N":
                            print("Stopping Optimization.")
                            print("\n================================================\n")
                            return True
                        else:
                            print("Continuing Optimization.")
                            print("\n================================================\n")


        if (self.test.optimization_iteration <= self.solver_parameters['max_iter']):
            return False

        else:
            print("Maximum number of iterations exceeded. Stopping optimization.")
            return True
