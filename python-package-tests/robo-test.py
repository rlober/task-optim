

import GPy
from robo.models.gpy_model import GPyModel
from robo.acquisition.lcb import LCB
from robo.acquisition.ei import EI
from robo.acquisition.log_ei import LogEI
from robo.maximizers.cmaes import CMAES
from robo.maximizers.direct import Direct
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

class ExampleTask(BaseTask):
    def __init__(self):
        X_lower = np.array([0])
        X_upper = np.array([6])
        super(ExampleTask, self).__init__(X_lower, X_upper)

    def objective_function(self, x):
        return np.sin(3 * x) * 4 * (x - 1) * (x + 2)

task = ExampleTask()

# task = ReachingWithBalance()
# kernel = GPy.kern.Matern52(input_dim=task.n_dims)
kernel = GPy.kern.RBF(input_dim=task.n_dims)
model = GPyModel(kernel)

PAR = 0.1
# acquisition_func = LCB(model, X_upper=task.X_upper, X_lower=task.X_lower, par=PAR)
# acquisition_func = EI(model, X_upper=task.X_upper, X_lower=task.X_lower, par=PAR)
acquisition_func = LogEI(model, X_upper=task.X_upper, X_lower=task.X_lower, par=PAR)

maximizer = Direct(acquisition_func, task.X_lower, task.X_upper)

bo = BayesianOptimization(acquisition_func=acquisition_func,
                          model=model,
                          maximize_func=maximizer,
                          task=task)

x_best, fval = bo.run()

print('x_best', task.retransform(x_best), 'fval', fval)

import matplotlib.pyplot as plt

plt.figure()
X= np.linspace(0,6,100)
plt.plot(X, task.objective_function(X))
plt.plot(task.retransform(x_best), task.objective_function(task.retransform(x_best)), 'ro')
plt.show(block=True)

# from robo.fmin import fmin
#
# def objective_function(x):
#     return  np.sin(3 * x) * 4 * (x - 1) * (x + 2)
#
# X_lower = np.array([0])
# X_upper = np.array([6])
#
# x_best, fval = fmin(objective_function, X_lower, X_upper)
#
# print('x_best', x_best, 'fval', fval)
