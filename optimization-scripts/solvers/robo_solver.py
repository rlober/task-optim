from .base_solver import BaseSolver

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

class RoboSolver(BaseSolver):
    """docstring for RoboSolver."""
    def __init__(self, test, solver_parameters):
        assert(solver_parameters['max_iter'] != None)
        super(RoboSolver, self, test, solver_parameters).__init__()

    def initializeSolver(self):
        # self.kernel = GPy.kern.Matern52(input_dim=self.test.n_dims)
        self.kernel = GPy.kern.RBF(input_dim=self.test.n_dims)
        self.model = GPyModel(self.kernel)

        self.acquisition = LCB(self.model, X_upper=self.test.X_upper, X_lower=self.test.X_lower, par=0.01)
        # self.acquisition = EI(self.model, X_upper=self.test.X_upper, X_lower=self.test.X_lower, par=0.1)

        self.maximizer = CMAES(self.acquisition, self.test.X_lower, self.test.X_upper)
        # self.maximizer = SciPyOptimizer(self.acquisition, self.test.X_lower, self.test.X_upper)
        # self.maximizer = GradientAscent(self.acquisition, self.test.X_lower, self.test.X_upper)

        self.bo = BayesianOptimization(acquisition_func=self.acquisition,
                                  model=self.model,
                                  maximize_func=self.maximizer,
                                  task=self.test)

    def updateSolver(self):
        # Multiply by -1 because we are maximizing with RoBO
        Y = self.Y.copy() * -1.0
        # Get the next solution to test
        next_solution_to_test = self.bo.choose_next(self.X, Y)
        # The provided solution is scaled bectween 0-1 so we retransform it back to physical coordinates
        X_new = self.test.retransform(next_solution_to_test)
        # test X_new
        Y_new = self.test.objective_function(X_new)
        # scale the cost wrt the original cost and re-multiply by -1
        Y_new = -1.0 * ( Y_new / self.test.Y_init )

        self.X = np.vstack((self.X, X_new))
        self.Y = np.vstack((self.Y, Y_new))

    def solverFinished(self):
        return (self.test.optimization_iteration <= self.solver_parameters['max_iter'])
