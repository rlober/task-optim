import numpy as np
from optim import *
from bayes_opt import *
import os

home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/"

right_hand_starting_waypoints = np.array([[0.24, -0.27, 0.64],[0.30, -0.10, 0.54],[0.36,  0.00, 0.44]])
com_starting_waypoints = np.array([[0.024, -0.060, 0.500],[0.025, -0.061, 0.501],[0.026, -0.062, 0.502]])

robo_task = ReachingWithBalance(root_path, right_hand_starting_waypoints, com_starting_waypoints)

# solver = initializeRoboSolver(robo_task)
#
# max_iter = 2
# X = robo_task.X_init
# Y = robo_task.Y_init
# for i in range(max_iter):
#     X_new = solver.choose_next(X, Y)
#     print(X_new)
#     Y_new = robo_task.objective_function(X_new)
#     print(Y_new)
#     X = np.vstack((X, X_new))
#     Y = np.vstack((Y, Y_new))

#print('wndjkawndkjwndjknawjkd', robo_task.X_lower, robo_task.X_upper)

solver = BayesOpt(robo_task.X_init,  robo_task.Y_init, robo_task.X_lower, robo_task.X_upper)

X_new = solver.getFirstGuess()
max_iter = 30
for i in range(max_iter):
    print("------\nObserved costs:\n", solver.Y.flatten(), "\n-------")
    print("New parameters to test:\nCOM [x, y, z]\n", X_new.reshape(4,3)[0:2,:], '\nRight Hand [x, y, z]\n', X_new.reshape(4,3)[2:,:])
    print("--Parameter estimations--\nCost mean: ", solver.est_cost_mean, " and  variance: ", solver.est_cost_var)

    Y_new = robo_task.objective_function(X_new)
    print("Actual cost after simulation: \n", Y_new)
    X_new = solver.update(X_new, Y_new)
