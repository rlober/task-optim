

import GPy
from robo.models.gpy_model import GPyModel
from robo.acquisition.lcb import LCB
from robo.maximizers.cmaes import CMAES
from robo.solver.bayesian_optimization import BayesianOptimization
from robo.task.base_task import BaseTask
import numpy as np

# The optimization function that we want to optimize. It gets a numpy array with shape (N,D) where N >= 1 are the number of datapoints and D are the number of features
class ReachingWithBalance(BaseTask):

    def __init__(self):
        self.X_lower = np.array([-5, 0])
        self.X_upper = np.array([10, 15])
        super(ReachingWithBalance, self).__init__(self.X_lower, self.X_upper)

    def objective_function(self, x):
        y = (x[:, 1] - (5.1 / (4 * np.pi ** 2)) * x[:, 0] ** 2 + 5 * x[:, 0] / np.pi - 6) ** 2
        y += 10 * (1 - 1 / (8 * np.pi)) * np.cos(x[:, 0]) + 10

        return y[:, np.newaxis]

task = ReachingWithBalance()
# kernel = GPy.kern.Matern52(input_dim=task.n_dims)
kernel = GPy.kern.RBF(input_dim=task.n_dims)
model = GPyModel(kernel)

acquisition_func = LCB(model,
                     X_upper=task.X_upper,
                     X_lower=task.X_lower,
                     par=0.1)

maximizer = CMAES(acquisition_func, task.X_lower, task.X_upper)

bo = BayesianOptimization(acquisition_func=acquisition_func,
                          model=model,
                          maximize_func=maximizer,
                          task=task)

bo.run()
