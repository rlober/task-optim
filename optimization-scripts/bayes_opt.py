import numpy as np
import GPy
import matplotlib.pyplot as plt
from scipy import optimize


class BayesOpt:
    """docstring for BayesOpt."""
    def __init__(self, X_init, Y_init, X_lower, X_upper):
        self.X_init = X_init
        self.Y_init = Y_init
        self.X = self.X_init
        self.Y = self.Y_init
        self.X_lower = X_lower
        self.X_upper = X_upper
        self.bounds = list(zip(self.X_lower, self.X_upper))

        self.input_dimension = self.X_init.shape[1]
        self.gamma = 1.0
        self.last_solution =  self.X_init.flatten()
        self.solutions = []
        self.iteration = 0
        # self.shouldOptimize = False
        self.shouldOptimize = True

        self.gpy_kernel = GPy.kern.RBF(input_dim=self.input_dimension, variance=0.00001, lengthscale=100.0)
        self.gpy_model = GPy.models.GPRegression(self.X,self.Y,self.gpy_kernel)
        self.gpy_model.optimize(messages=False)
        self.first_guess = self.minimizeAcquisitionFunction()

    def getFirstGuess(self):
        return self.first_guess

    def setBounds(self, bounds):
        self.bounds = bounds

    def updateModel(self, newX, newY, optimize=False):
        self.X = np.vstack((self.X, newX))
        self.Y = np.vstack((self.Y, newY))

        print("\n\nX:\n", self.X)
        print("\n\nY:\n", self.Y)
        self.gpy_model.set_XY(self.X, self.Y)
        if optimize:
            self.gpy_model.optimize(messages=False)

    def LCB(self, X):
        if len(X.shape)==1:
            X = np.array([X])
        mean, variance = self.gpy_model.predict(X)#, full_cov=False)
        return mean - self.gamma * variance

    def minimizeAcquisitionFunction(self):
        self.solutions.append(optimize.minimize(self.LCB, x0=self.X_init.flatten(), method='L-BFGS-B', bounds=self.bounds))
        self.last_solution = np.array([self.solutions[-1].x])
        self.last_solution = np.where((self.last_solution > self.X_upper), self.X_upper, self.last_solution)
        self.last_solution = np.where((self.last_solution < self.X_lower), self.X_lower, self.last_solution)
        est_cost_mean, est_cost_var = self.gpy_model.predict(self.last_solution)
        print("\n\n cost estimate: ", est_cost_mean, " with variance of: ", est_cost_var)
        return self.last_solution

    def update(self, newX, newY):
        self.iteration += 1
        print("\n\nIteration: ", self.iteration)
        # if (self.iteration % 5) == 0:
        #     self.shouldOptimize = True
        # else:
        #     self.shouldOptimize = False
        self.updateModel(newX, newY, self.shouldOptimize)
        return self.minimizeAcquisitionFunction()
