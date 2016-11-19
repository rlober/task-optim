import numpy as np

def randrange(n, vmin, vmax):
    return ((vmax - vmin)*np.random.rand(n) + vmin).reshape((n,1))

def getFakeData(n_points=100):
    xmin = 0.0
    xmax = 2.0

    ymin = 0.0
    ymax = 1.0

    zmin = 0.0
    zmax = 3.0

    lower_bounds = [xmin, ymin, zmin]
    upper_bounds = [xmax, ymax, zmax]

    n = n_points
    x_coords = randrange(n, xmin, xmax)
    y_coords = randrange(n, ymin, ymax)
    z_coords = randrange(n, zmin, zmax)

    costmin = 0.0
    costmax = 20.0

    costs = randrange(n, costmin, costmax)
    waypoints = np.hstack((x_coords,y_coords,z_coords))

    return waypoints, costs, lower_bounds, upper_bounds
