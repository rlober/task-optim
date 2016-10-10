import numpy as np
import GPy
import matplotlib.pyplot as plt
from scipy import optimize


class BayesOpt:
    """docstring for BayesOpt."""
    def __init__(self, X_init, Y_init, X_lower, X_upper, cost_saturation=2):
        self.cost_saturation = cost_saturation
        self.X_init = X_init
        self.Y_init = Y_init
        self.X_lower = X_lower
        self.X_upper = X_upper
#        print('x_l:', X_lower, '\n\nx_u:', X_upper)
        self.X_range = self.X_upper - self.X_lower
        self.input_dimension = self.X_init.shape[1]
        self.X = self.normalizeX(self.X_init)
        self.Y = self.normalizeY(self.Y_init)

        zeros = np.zeros((self.input_dimension,))
        ones = np.ones((self.input_dimension,))
        self.bounds = list(zip(zeros, ones))
        # self.bounds = list(zip(self.X_lower, self.X_upper))

        self.gamma = 1.0
        self.last_solution =  self.X_init.flatten()
        self.solutions = []
        self.iteration = 0
        # self.shouldOptimize = False
        self.shouldOptimize = True

        self.gpy_kernel = GPy.kern.RBF(input_dim=self.input_dimension, variance=0.01, lengthscale=0.10)
        self.gpy_model = GPy.models.GPRegression(self.X,self.Y,self.gpy_kernel)
        self.gpy_model.optimize(messages=False)
        self.first_guess = self.minimizeAcquisitionFunction()

    def normalizeX(self, X):
        x_norm = (X - self.X_lower)/self.X_range
 #       print(X, '\n', self.X_lower, '\n', self.X_range, '\n', x_norm, '\n')
        return x_norm

    def unNormalizeX(self, X):
        return X * self.X_range + self.X_lower

    def normalizeY(self, Y):
        y_norm = Y/self.Y_init
        y_norm = np.where((y_norm>self.cost_saturation), self.cost_saturation, y_norm)
        return y_norm

    def unNormalizeY(self, Y):
        return Y*self.Y_init

    def getFirstGuess(self):
        return self.first_guess

    def setBounds(self, bounds):
        self.bounds = bounds

    def updateModel(self, newX, newY, optimize=False):
        self.X = np.vstack((self.X, self.normalizeX(newX) ))
        self.Y = np.vstack((self.Y, self.normalizeY(newY) ))

#        print("\n\nX:\n", self.X)
 #       print("\n\nY:\n", self.Y)
        self.gpy_model.set_XY(self.X, self.Y)
        if optimize:
            self.gpy_model.optimize(messages=False)

    def LCB(self, X):
        if len(X.shape)==1:
            X = np.array([X])
        mean, variance = self.gpy_model.predict(X)#, full_cov=False)
        return mean - self.gamma * variance

    def minimizeAcquisitionFunction(self):

  #      print(self.bounds)
   #     print(self.normalizeX(self.X_init).flatten())
        self.solutions.append(optimize.minimize(self.LCB, x0=self.normalizeX(self.X_init).flatten(), method='L-BFGS-B', bounds=self.bounds))

  #      print("normalized solution: ", np.array([self.solutions[-1].x]))
        self.last_solution = np.array([self.solutions[-1].x])
        self.last_solution = self.unNormalizeX(self.last_solution)
        self.last_solution = np.where((self.last_solution > self.X_upper), self.X_upper, self.last_solution)
        self.last_solution = np.where((self.last_solution < self.X_lower), self.X_lower, self.last_solution)


        self.est_cost_mean, self.est_cost_var = self.gpy_model.predict(self.normalizeX(self.last_solution))
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
