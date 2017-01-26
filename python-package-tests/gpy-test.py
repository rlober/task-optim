#import necessary modules, set up the plotting
import numpy as np
import matplotlib;matplotlib.rcParams['figure.figsize'] = (8,6)
from matplotlib import pyplot as plt
import GPy


X = np.array([[0.0],[0.1],[0.2],[0.8]])
Y = np.array([[0.8],[1.0],[0.4],[2.0]])

kernel = GPy.kern.RBF(input_dim=1)#, variance=0.1, lengthscale=0.1)
m = GPy.models.GPRegression(X,Y,kernel)

fig = m.plot()

m.optimize(messages=True)


fig_opt = m.plot()
plt.show(block=True)
