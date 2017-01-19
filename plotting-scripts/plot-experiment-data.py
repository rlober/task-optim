import os
import re
import sys
sys.path.append("../optimization-scripts")
from task_optim.utils.files import *
import pickle
import matplotlib
import matplotlib.pyplot as plt
sys.path.append("../plotting-scripts")
from one_com_waypoint_static import *

icub_joint_names = ["torso_pitch", "torso_roll", "torso_yaw", "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll", "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"]


def calculateTotalCost(task_data, costs=['tracking', 'goal', 'energy']):
    j_total = 0.0
    j_tracking = 0.0
    j_goal = 0.0
    j_energy = task_data[0].energyCost()
    for t in task_data:
        j_tracking += t.trackingCost()
        j_goal += t.goalCost()

    for c in costs:
        if c == 'tracking':
            j_total += j_tracking
        if c == 'goal':
            j_total += j_goal
        if c == 'energy':
            j_total += j_energy

    return j_total



# costs_used=['tracking', 'goal', 'energy']
# costs_used=['tracking', 'goal']
# costs_used=['energy']

reaching_data_path = '/home/ryan/Code/bayesian-task-optimization/experiments/reaching'
plot_save_dir = reaching_data_path
original_data = getDataFromFiles(os.path.join(reaching_data_path, 'original_data'))
optimal_data = getDataFromFiles(os.path.join(reaching_data_path, 'optimal_data'))


cuttoff_time = 30.2435
# cuttoff_time = 40.0
original_data[0].cutOffDataAfter(cuttoff_time)
original_data[1].cutOffDataAfter(cuttoff_time)
optimal_data[0].cutOffDataAfter(cuttoff_time)
optimal_data[1].cutOffDataAfter(cuttoff_time)

# print('Original')
# print('real', original_data[1].real[-1,:])
# print('ref', original_data[1].ref[-1,:])
# print('ref', original_data[1].goal())
# print('error:', np.linalg.norm(original_data[1].ref[-1,:]-original_data[1].real[-1,:]))
# print('Optimal')
# print('real', optimal_data[1].real[-1,:])
# print('ref', optimal_data[1].ref[-1,:])
# print('ref', optimal_data[1].goal())
# print('error:', np.linalg.norm(optimal_data[1].ref[-1,:]-optimal_data[1].real[-1,:]))
#
#
# plt.figure()
# plt.plot(original_data[1].time, original_data[1].goalPositionErrorNorm(), 'r')
# plt.plot(optimal_data[1].time, optimal_data[1].goalPositionErrorNorm(), 'b')
# plt.show()



costs_used=[['tracking', 'goal', 'energy'], ['tracking', 'energy'], ['tracking', 'goal'], ['goal', 'energy'], ['tracking'], ['goal'], ['energy']]

for c in costs_used:
    print(c)
    print('Total original cost:', calculateTotalCost(original_data, c))
    print('Total optimal cost:', calculateTotalCost(optimal_data, c))



# cost_scaling_factor = calculateTotalCost(original_data, costs_used)
# print('Total original cost:', calculateTotalCost(original_data, costs_used))
# print('Total optimal cost:', calculateTotalCost(optimal_data, costs_used))

# print("Plotting Original task set data.")
# orig = plot.DataPlots(original_data, costs_used, cost_scaling_factor)
# orig.plotIndividualCosts(show_plot=False, save_dir=plot_save_dir, filename='Original_Tasks_IndividualCosts')
# orig.plotCostPercentages(show_plot=False, save_dir=plot_save_dir, filename='Original_Tasks_CostPercentages')
# orig.plotJacobianRanks(show_plot=False, save_dir=plot_save_dir, filename='Original_Tasks_JacobianRanks')
# orig.plotJointPositions(show_plot=False, save_dir=plot_save_dir, filename='Original_Tasks_JointPositions')
#
# print("Plotting Optimal task set data.")
# opt = plot.DataPlots(optimal_data, costs_used, cost_scaling_factor)
# opt.plotIndividualCosts(show_plot=False, save_dir=plot_save_dir, filename='Optimal_Tasks_IndividualCosts')
# opt.plotCostPercentages(show_plot=False, save_dir=plot_save_dir, filename='Optimal_Tasks_CostPercentages')
# opt.plotJacobianRanks(show_plot=False, save_dir=plot_save_dir, filename='Optimal_Tasks_JacobianRanks')
# opt.plotJointPositions(show_plot=False, save_dir=plot_save_dir, filename='Optimal_Tasks_JointPositions')
#
# print("Plotting Original/Optimal comparaisons.")
# orig.compare(opt, save_dir=plot_save_dir)
