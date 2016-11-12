import math
import numpy as np
import matplotlib.pyplot as plt

waypoints = np.array([0.1,  0.2,  1.0,  0.8,  0.8])
indexes = np.linspace(0, waypoints.size, waypoints.size)
nDiscreteSteps = 100
discreteIndexes = np.linspace(0, waypoints.size, nDiscreteSteps)
positions = np.zeros( (nDiscreteSteps ) )
coeffs = np.polyfit(indexes, waypoints, 4)
polynomial1D = np.poly1d(coeffs)
for j, t in enumerate(discreteIndexes):
    positions[j] = polynomial1D(t)


figureWidth = 16
figureHeight = figureWidth / 16 * 9 * 2
fig = plt.figure(num='traj', figsize=(figureWidth, figureHeight), dpi=80, facecolor='grey', edgecolor='k')
ax = fig.add_subplot(1,1,1)
ax.plot(indexes, waypoints, ls='', marker='o', markersize=10)
ax.plot(discreteIndexes, positions, linewidth=2)
ax.set_ylabel('q (m)')
ax.set_xlabel('time (s)')
plt.show()


VEL_MAX = 1.0 # m/s
ACC_MAX = 5.0  # m/s^2

timeline = np.zeros(positions.size)
velocities = np.zeros(positions.size)
accelerations = np.zeros(positions.size)
for i in range(1,positions.size-1):
    pos_im1 = positions[i-1]
    pos_i = positions[i]
    pos_ip1 = positions[i+1]


    dt_vel = math.fabs(pos_ip1 - pos_i)/VEL_MAX
    dt_acc = math.sqrt(math.fabs(pos_ip1 - 2*pos_i + pos_im1)/ACC_MAX)

    dt = max((dt_vel, dt_acc))

    timeline[i] = timeline[i-1]+dt

timeline[-1] = timeline[-2]

velocities = np.diff(positions) / np.diff(timeline)
velocities = np.hstack( ( np.zeros((1)), velocities) )
accelerations = np.diff(velocities) / np.diff(timeline)
accelerations = np.hstack( ( np.zeros((1)), accelerations) )

print(positions.shape, velocities.shape, accelerations.shape)
# print(timeline)
figureWidth = 16
figureHeight = figureWidth / 16 * 9 * 2
fig = plt.figure(num='traj', figsize=(figureWidth, figureHeight), dpi=80, facecolor='grey', edgecolor='k')
axpos = fig.add_subplot(3,1,1)
axpos.plot(timeline, positions, linewidth=2)
axpos.set_ylabel('q (m)')
axpos.set_xlabel('time (s)')

axvel = fig.add_subplot(3,1,2)
axvel.plot(timeline, velocities, linewidth=2)
axvel.set_ylabel('qd (m)')
axvel.set_xlabel('time (s)')

axacc = fig.add_subplot(3,1,3)
axacc.plot(timeline, accelerations, linewidth=2)
axacc.set_ylabel('qdd (m)')
axacc.set_xlabel('time (s)')
plt.show()
