from .base_solver import BaseSolver
import numpy as np
import GPy
from robo.models.gpy_model import GPyModel
from robo.acquisition.lcb import LCB
from robo.acquisition.ei import EI
from robo.acquisition.log_ei import LogEI
from robo.maximizers.cmaes import CMAES
from robo.maximizers.direct import Direct
# from robo.maximizers.gradient_ascent import GradientAscent
# from robo.maximizers.grid_search import GridSearch
# from robo.maximizers.scipy_optimizer import SciPyOptimizer
# from robo.maximizers.stochastic_local_search import StochasticLocalSearch

from robo.solver.bayesian_optimization import BayesianOptimization

class RoboSolver(BaseSolver):
    """docstring for RoboSolver."""
    def __init__(self, test, solver_parameters):
        assert(solver_parameters['max_iter'] != None)
        super(RoboSolver, self).__init__(test, solver_parameters)

    def setKernel(self):
        if 'kernel' in self.solver_parameters:
            if self.solver_parameters['kernel'] == 'Matern52':
                self.kernel = GPy.kern.Matern52(input_dim=self.test.n_dims)
            elif self.solver_parameters['kernel'] == 'RBF':
                self.kernel = GPy.kern.RBF(input_dim=self.test.n_dims)
            else:
                self.kernel = GPy.kern.RBF(input_dim=self.test.n_dims)
        else:
            self.kernel = GPy.kern.RBF(input_dim=self.test.n_dims)

    def setModel(self):
        self.model = GPyModel(self.kernel)


    def setAcquisitionFunction(self):
        # Set par factor
        if 'par' in self.solver_parameters:

            if (self.solver_parameters['par'] > 0.0):
                self.par = self.solver_parameters['par']
            else:
                self.par = 0.01
        else:
            self.par = 0.01

        if 'acquisition' in self.solver_parameters:

            if self.solver_parameters['acquisition'] == 'EI':
                self.acquisition = EI(self.model, X_upper=self.test.X_upper, X_lower=self.test.X_lower, par=self.par)
            elif self.solver_parameters['acquisition'] == 'LogEI':
                self.acquisition = LogEI(self.model, X_upper=self.test.X_upper, X_lower=self.test.X_lower, par=self.par)
            elif self.solver_parameters['acquisition'] == 'LCB':
                self.acquisition = LCB(self.model, X_upper=self.test.X_upper, X_lower=self.test.X_lower, par=self.par)
            else:
                self.acquisition = LCB(self.model, X_upper=self.test.X_upper, X_lower=self.test.X_lower, par=self.par)

        else:
            self.acquisition = LCB(self.model, X_upper=self.test.X_upper, X_lower=self.test.X_lower, par=self.par)


    def setMaximizer(self):
        if 'maximizer' in self.solver_parameters:

            if self.solver_parameters['maximizer'] == 'SciPyOptimizer':
                pass
                # self.maximizer = SciPyOptimizer(self.acquisition, self.test.X_lower, self.test.X_upper)

            elif self.solver_parameters['maximizer'] == 'GradientAscent':
                pass
                # self.maximizer = GradientAscent(self.acquisition, self.test.X_lower, self.test.X_upper)

            elif self.solver_parameters['maximizer'] == 'Direct':
                pass
                self.maximizer = Direct(self.acquisition, self.test.X_lower, self.test.X_upper)

            elif self.solver_parameters['maximizer'] == 'GridSearch':
                pass
                # self.maximizer = GridSearch(self.acquisition, self.test.X_lower, self.test.X_upper)

            elif self.solver_parameters['maximizer'] == 'CMAES':
                self.maximizer = CMAES(self.acquisition, self.test.X_lower, self.test.X_upper)
            else:
                self.maximizer = CMAES(self.acquisition, self.test.X_lower, self.test.X_upper)

        else:
            self.maximizer = CMAES(self.acquisition, self.test.X_lower, self.test.X_upper)

    def setBayesianOptimizationSolver(self):
        self.bo = BayesianOptimization(acquisition_func=self.acquisition,
                                  model=self.model,
                                  maximize_func=self.maximizer,
                                  task=self.test)


    def initializeSolver(self):
        # Create kernel
        self.setKernel()
        # Create model
        self.setModel()
        # Create acquisiton function
        self.setAcquisitionFunction()
        # Create maximizer
        self.setMaximizer()
        # Construct RoBO Bayesian optimization solver
        self.setBayesianOptimizationSolver()

    def updateSolver(self):
        # Multiply by -1 because we are maximizing with RoBO
        # Y = self.Y.copy() * -1.0
        # Get the next solution to test
        next_solution_to_test = self.bo.choose_next(self.X, self.Y)
        # The provided solution is scaled bectween 0-1 so we retransform it back to physical coordinates
        X_new = self.test.retransform(next_solution_to_test)
        # test X_new
        Y_new = self.test.objective_function(X_new)
        # scale the cost wrt the original cost
        Y_new = ( Y_new / self.test.Y_init )

        self.X = np.vstack((self.X, X_new))
        self.Y = np.vstack((self.Y, Y_new))

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
