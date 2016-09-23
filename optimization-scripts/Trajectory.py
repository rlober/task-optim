import numpy as np
import matplotlib.pyplot as plt

def PlotTrajectory(times, waypoints, timeline, trajectory, title='Trajectory', show=False):
    l = ['x', 'y', 'z', 'rx', 'ry', 'rz']
    c = ['r', 'g', 'b', 'y', 'k', 'm']
    figureWidth = 16
    figureHeight = figureWidth / 16 * 9
    fig = plt.figure(num=title, figsize=(figureWidth, figureHeight), dpi=80, facecolor='grey', edgecolor='k')
    for i in range(waypoints.shape[0]):
        ax = fig.add_subplot(1,waypoints.shape[0],i+1)
        ax.plot(times, waypoints[i,:], color=c[i], ls='', marker='o', markersize=10)
        ax.plot(timeline_poly, trajectory_poly[i,:], color=c[i], linewidth=2)
        ax.set_ylabel(l[i] + ' (m)')
        ax.set_xlabel('time (s)')
        ax.set_xlim([-1.0, 6.0])
    if show:
        plt.show()


def PolynomialTrajectory(times, waypoints, degree=4, timelineDt=0.01):
    nDof = waypoints.shape[0]
    timeline = np.arange(times[0], times[-1]+timelineDt, timelineDt)
    trajectory = np.zeros( ( nDof, timeline.shape[0] ) )
    for i in range(nDof):
        coeffs = np.polyfit(times, waypoints[i,:], degree)
        polynomial1D = np.poly1d(coeffs)
        for j, t in enumerate(timeline):
            trajectory[i,j] = polynomial1D(t)

    return timeline, trajectory

def BlendedLineTrajectory(times, waypoints):
    nDof = waypoints.shape[0]
    timeline = np.arange(times[0], times[-1]+timelineDt, timelineDt)
    # traj = [x
    #         y
    #         z
    #          ]
    trajectory = np.zeros( ( nDof*3, timeline.shape[0] ) )

    # Repeat the first and last waypoints
    waypoints = np.hstack( (times[0], times, times[-1] ))


    return timeline, trajectory






########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################


waypoints = np.array([  [0.1,  0.2,  1.0,  0.8,  0.8],
                        [0.5,  1.0,  1.0,  0.2,  0.9],
                        [-1.0, 0.1, -2.0, -0.8,  0.0]])


times = np.array([0.0, 1.0, 2.5, 3.1, 5.0])

degree_poly = 3
timeline_poly, trajectory_poly = PolynomialTrajectory(times, waypoints, degree_poly)

PlotTrajectory(times, waypoints, timeline_poly, trajectory_poly, 'Polynomial Trajectory')
plt.show()
