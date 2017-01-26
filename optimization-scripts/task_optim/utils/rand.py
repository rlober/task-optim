import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D


def generateRandomRightHandTargets(n_samples=100, save_path=None, lower_bounds=None, upper_bounds=None):

    rh_targets = np.random.rand(n_samples, 3)
    lb = lower_bounds
    ub = upper_bounds
    if lower_bounds is not None and upper_bounds is not None:
        xmin = lower_bounds[0]
        xmax = upper_bounds[0]
        ymin = lower_bounds[1]
        ymax = upper_bounds[1]
        zmin = lower_bounds[2]
        zmax = upper_bounds[2]

    else:
        lb = 3*[None]
        ub = 3*[None]
        xmin = -0.10
        xmax = 0.60
        ymin = -0.40
        ymax = 0.20
        zmin = 0.00
        zmax = 1.00
        lb[0] = xmin
        ub[0] = xmax
        lb[1] = ymin
        ub[1] = ymax
        lb[2] = zmin
        ub[2] = zmax

    x_range = xmax - xmin
    y_range = ymax - ymin
    z_range = zmax - zmin

    rh_targets[:,0] = rh_targets[:,0] * x_range + xmin
    rh_targets[:,1] = rh_targets[:,1] * y_range + ymin
    rh_targets[:,2] = rh_targets[:,2] * z_range + zmin

    if save_path is not None and os.path.exists(save_path):
        np.savetxt(os.path.join(save_path, "rh_targets.txt"), rh_targets)

    return rh_targets, lb, ub




def plotRightHandTargets(rh_targets, lower_bounds, upper_bounds):
    xmin = lower_bounds[0]
    xmax = upper_bounds[0]
    ymin = lower_bounds[1]
    ymax = upper_bounds[1]
    zmin = lower_bounds[2]
    zmax = upper_bounds[2]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(rh_targets.shape[0]):
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
