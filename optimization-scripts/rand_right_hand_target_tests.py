import numpy as np
import os
from task_optim.test_scenario.one_com_waypoint_static_test import OneComWaypointStaticTest
from task_optim.solvers.robo_solver import RoboSolver
from task_optim.solvers.cma_solver import CmaSolver
import matplotlib.pyplot as plt


def plotRightHandTargets(xmin, xmax, ymin, ymax, zmin, zmax):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(n_samples):
        ax.scatter(rh_targets[i,0], rh_targets[i,1], rh_targets[i,2], c='b', marker='o')

    ax.plot([xmin,xmax], [ymin,ymin], [zmin,zmin], 'r')
    ax.plot([xmin,xmax], [ymax,ymax], [zmin,zmin], 'r')
    ax.plot([xmin,xmin], [ymin,ymax], [zmin,zmin], 'r')
    ax.plot([xmax,xmax], [ymin,ymax], [zmin,zmin], 'r')

    ax.plot([xmin,xmax], [ymin,ymin], [zmax,zmax], 'r')
    ax.plot([xmin,xmax], [ymax,ymax], [zmax,zmax], 'r')
    ax.plot([xmin,xmin], [ymin,ymax], [zmax,zmax], 'r')
    ax.plot([xmax,xmax], [ymin,ymax], [zmax,zmax], 'r')

    ax.plot([xmax,xmax], [ymax,ymax], [zmin,zmax], 'r')
    ax.plot([xmax,xmax], [ymin,ymin], [zmin,zmax], 'r')
    ax.plot([xmin,xmin], [ymin,ymin], [zmin,zmax], 'r')
    ax.plot([xmin,xmin], [ymax,ymax], [zmin,zmax], 'r')

    ax.set_xlim([xmin, xmax])
    ax.set_ylim([ymin, ymax])
    ax.set_zlim([zmin, zmax])


    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    plt.show(block=True)



home_path = os.path.expanduser("~")
root_path = home_path + "/Optimization_Tests/rand_right_hand_target_tests/"

right_hand_starting_waypoints = np.array([[0.36, -0.23, 0.5]])
com_starting_waypoints = np.array([[0.015, -0.11, 0.51]])

n_samples = 100
rh_targets = np.random.rand(n_samples, 3)


xmin = -0.10
xmax = 0.60
x_range = xmax - xmin

ymin = -0.40
ymax = 0.20
y_range = ymax - ymin

zmin = 0.10
zmax = 1.00
z_range = zmax - zmin

rh_targets[:,0] = rh_targets[:,0] * x_range + xmin
rh_targets[:,1] = rh_targets[:,1] * y_range + ymin
rh_targets[:,2] = rh_targets[:,2] * z_range + zmin


tolerance = 1e-11
maxIter = 44


costs_to_use=['tracking', 'goal', 'energy']

bo_solver_parameters  = {'max_iter':maxIter, 'tolfun':tolerance, 'par':1000.0, 'kernel':'Matern52', 'acquisition':'EI', 'maximizer':'Direct'}
bo_test_path = root_path + "/bo/"


######################################################################################

print("\n\n\n\n\n\n=================================================")
print("Random Right Hand Target Tests.")
print("=================================================\n\n\n")

for i,t in enumerate(rh_targets):
    print("\n\n========================================")
    print("Test number:", i+1, "of", n_samples, "tests.\nRight hand target:", t)
    print("========================================")

    first_test = OneComWaypointStaticTest(bo_test_path, t, com_starting_waypoints, costs_to_use)
    solver = RoboSolver(first_test, bo_solver_parameters)
    solver.optimize()
    solver.returnSolution(show_simulation=False)
