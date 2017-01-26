#import necessary modules, set up the plotting
import numpy as np
import matplotlib;matplotlib.rcParams['figure.figsize'] = (8,6)
from matplotlib import pyplot as plt
import numpy as np
from sklearn import gaussian_process

import cma

X = np.array([[0.0],[0.1],[0.2],[0.21],[0.8], [1.0]])
Y = np.array([[0.8],[1.0],[0.4],[2.0],[2.0],[2.0]])
x= np.reshape(np.linspace(0,1,100), (100,1))
kernel = 0.5 * gaussian_process.kernels.Matern(length_scale=1.0, length_scale_bounds=(1e-1, 10.0), nu=1.5)
gp = gaussian_process.GaussianProcessRegressor(kernel)
gp.fit(X, Y)

y_pred, sigma = gp.predict(x, return_std=True)
sigma = np.reshape(sigma, (sigma.shape[0],1))
# print(y_pred)
#
# plt.figure()
# plt.plot(x, y_pred,lw=3)
# plt.plot(X,Y,'bo',ms=20)
# plt.show(block=True)


par = 1.0
def LCB(x, user_data=None):
    x = np.array([x])
    mean, var = gp.predict(x, return_std=True)
    var = np.reshape(var, (var.shape[0],1))
    return mean - par*np.sqrt(var)

# from DIRECT import solve
# x_opt, fmin, ierror = solve(LCB,[0.0],[1.0])

# res = cma.fmin(LCB, [0.5], sigma0=0.8)

fmin_l_bfgs_b

print(x_opt)
#
# fig = plt.figure()
# plt.plot(X, Y, 'r.', markersize=10, label=u'Observations')
# low = y_pred - sigma
# high = y_pred + sigma
# plt.fill_between(x[:,0], high[:,0], low[:,0], alpha=0.5, color='b')
# plt.plot(x, y_pred, 'b-', label=u'Prediction')
# plt.plot(x, LCB(x), 'y', lw=3, label=u'Prediction')
# plt.xlabel('$x$')
# plt.ylabel('$f(x)$')
# plt.legend(loc='upper left')
# plt.show(block=True)


#
# kernel = GPy.kern.RBF(input_dim=1)#, variance=0.1, lengthscale=0.1)
# m = GPy.models.GPRegression(X,Y,kernel)
#
# fig = m.plot()
#
# m.optimize(messages=True)
#
#
# fig_opt = m.plot()
# plt.show(block=True)
