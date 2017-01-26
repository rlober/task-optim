from .base_solver import BaseSolver
import numpy as np
from sklearn import gaussian_process
import cma

class BayesOptSolver(BaseSolver):
    """docstring for BayesOptSolver."""
    def __init__(self, test, solver_parameters):
        assert(solver_parameters['max_iter'] != None)

        self.par = 1.0
        if 'par' in solver_parameters:
            self.par = solver_parameters['par']
        super(BayesOptSolver, self).__init__(test, solver_parameters)

    def initializeSolver(self):
        self.kernel = gaussian_process.kernels.Matern(length_scale=100.0, length_scale_bounds=(1e-1, 1000.0), nu=2.0)
        self.gp = gaussian_process.GaussianProcessRegressor(self.kernel, n_restarts_optimizer=10)

    def updateSolver(self):

        # Put X 0-1
        X_scaled = self.test.transform(self.X)
        # Get the next solution to test
        next_solution_to_test = self.chooseNext(X_scaled, self.Y)
        # The provided solution is scaled bectween 0-1 so we retransform it back to physical coordinates
        X_new = self.test.retransform(next_solution_to_test)
        # test X_new
        Y_new = self.test.objective_function(X_new)
        # scale the cost wrt the original cost
        Y_new = ( Y_new / self.test.Y_init )
        thresh = 2.0
        if Y_new > thresh:
            Y_new = np.array([[thresh]])

        self.X = np.vstack((self.X, X_new))
        self.Y = np.vstack((self.Y, Y_new))

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

    def solverFinished(self):
        if self.X.shape[0] >= 2:
            # check for tolerance:
            if 'tolfun' in self.solver_parameters:
                deltaSol = np.linalg.norm(self.X[-1,:] - self.X[-2,:])
                if deltaSol <= self.solver_parameters['tolfun']:
                    print("Solution tolerance,", self.solver_parameters['tolfun'], "reached. Stopping optimization.")
                    return True


        if (self.test.optimization_iteration <= self.solver_parameters['max_iter']):
            return False

        else:
            print("Maximum number of iterations exceeded. Stopping optimization.")
            return True
