import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def getScaledCostsAndColorMap(costs):
    scaled_costs = costs / np.max(costs)
    cmap = matplotlib.cm.get_cmap('viridis')
    colors = cmap(scaled_costs.flatten())
    return scaled_costs, colors

def plotComWaypointBounds(ax, lower_bounds, upper_bounds):

    xmin = lower_bounds[0]
    ymin = lower_bounds[1]
    zmin = lower_bounds[2]

    xmax = upper_bounds[0]
    ymax = upper_bounds[1]
    zmax = upper_bounds[2]

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

def plot3dScatter(waypoints, costs, lower_bounds, upper_bounds):
    scaled_costs, colors = getScaledCostsAndColorMap(costs)

    # plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(len(colors)):
        ax.scatter(waypoints[i,0], waypoints[i,1], waypoints[i,2], c=colors[i,:], marker='o')

    plotComWaypointBounds(ax, lower_bounds, upper_bounds)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    range_max = max(upper_bounds-lower_bounds)*1.2
    means = (upper_bounds + lower_bounds) / 2
    new_lowers = means - (range_max / 2)
    new_uppers = means + (range_max / 2)


    ax.set_xlim([new_lowers[0], new_uppers[0]])
    ax.set_ylim([new_lowers[1], new_uppers[1]])
    ax.set_zlim([new_lowers[2], new_uppers[2]])

    return fig, ax
    # plt.show()
