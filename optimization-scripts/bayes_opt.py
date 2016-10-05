import numpy as np
import GPy
import matplotlib.pyplot as plt
from scipy import optimize


class BayesOpt:
    """docstring for BayesOpt."""
    def __init__(self, X_init, Y_init, bounds):
        self.X_init = X_init
        self.Y_init = Y_init
        self.X = self.X_init
        self.Y = self.Y_init
        self.input_dimension = self.X_init.shape[1]
        self.gamma = 1.0
        self.last_solution = np.array([[0.0]])
        self.solutions = []
        self.iteration = 0

        self.gpy_kernel = GPy.kern.RBF(input_dim=self.input_dimension, variance=1.0, lengthscale=0.1)
        self.gpy_model = GPy.models.GPRegression(self.X,self.Y,self.gpy_kernel)

        self.first_guess = self.minimizeAcquisitionFunction()

    def getFirstGuess(self):
        return self.first_guess

    def setBounds(self, bounds):
        self.bounds = bounds

    def updateModel(self, newX, newY, optimize=False):
        self.X = np.vstack((self.X, newX))
        self.Y = np.vstack((self.Y, newY))
        self.gpy_model.set_XY(self.X, self.Y)
        if optimize:
            self.gpy_model.optimize(messages=False)

    def LCB(self, X):
        if len(X.shape)==1:
            X = np.array([X])
        mean, variance = self.gpy_model.predict(X)#, full_cov=False)
        return mean - self.gamma * variance

    def minimizeAcquisitionFunction(self):
        self.solutions.append(optimize.minimize(self.LCB, x0=self.last_solution, method='L-BFGS-B', bounds=self.bounds))
        self.last_solution = np.array([solutions[-1].x])
        return self.last_solution

    def update(self, newX, newY):
        self.updateModel(newX, newY, self.shouldOptimize)
        return self.minimizeAcquisitionFunction()
