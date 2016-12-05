import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os
import sys
sys.path.append("../")
from color_palette import palettes


def checkAndCreateDir(dir_path):
    if not os.path.exists(dir_path):
        print(dir_path, 'Did not exist. Creating...')
        os.makedirs(dir_path)

    return os.path.abspath(dir_path)

def saveAndShow(figure, show_plot=False, save_dir=None, filename=None):

    if show_plot:
        plt.show()

    if save_dir is not None:
        full_path = checkAndCreateDir(save_dir)
        if filename is None:
            filename = 'figure'
        filename += '.png'
        save_path = os.path.join(full_path, filename)
        figure.savefig(save_path, format='png')

    plt.close(figure)


class DataPlots():
    """docstring for DataPlots."""
    def __init__(self, task_data):
        self.com_task_data = task_data[0]
        self.rh_task_data = task_data[1]

        self.com_tracking_cost = self.com_task_data.positionErrorSquaredNormTimeAveraged()
        self.rh_tracking_cost = self.rh_task_data.positionErrorSquaredNormTimeAveraged()

        self.com_goal_cost = self.com_task_data.goalPositionErrorSquaredNormPenalized()
        self.rh_goal_cost = self.rh_task_data.goalPositionErrorSquaredNormPenalized()

        self.energy_cost = self.com_task_data.torquesSquaredNormTimeAveragedScaled()

        self.total_cost = self.com_tracking_cost + self.rh_tracking_cost + self.com_goal_cost + self.rh_goal_cost + self.energy_cost

        self.max_total_cost = max(self.total_cost)

        self.total_cost = self.total_cost / self.max_total_cost

        self.com_color = palettes.Blues()
        self.rh_color = palettes.Reds()
        self.energy_color = palettes.Greens()
        self.total_color = palettes.Purples()

    def plotIndividualCosts(self, show_plot=False, save_dir=None, filename='plotIndividualCosts'):

        fig, (tracking_ax, goal_ax, energy_ax, total_ax) = plt.subplots(4, sharex=True, num=None, figsize=(8, 10), facecolor='w', edgecolor='k')

        tracking_ax.set_ylabel(r'$j_{tracking}$')
        tracking_ax.plot(self.com_task_data.time, self.com_tracking_cost, color=self.com_color.light, lw=3, label='com_tracking_cost')
        tracking_ax.plot(self.rh_task_data.time, self.rh_tracking_cost, color=self.rh_color.light, lw=3, label='rh_tracking_cost')
        tracking_ax.legend()

        goal_ax.set_ylabel(r'$j_{goal}$')
        goal_ax.plot(self.com_task_data.time, self.com_goal_cost , color=self.com_color.dark, lw=3, label='com_goal_cost')
        goal_ax.plot(self.rh_task_data.time, self.rh_goal_cost, color=self.rh_color.dark, lw=3, label='rh_goal_cost')
        goal_ax.legend()

        energy_ax.set_ylabel(r'$j_{energy}$')
        energy_ax.plot(self.com_task_data.time, self.energy_cost, color=self.energy_color.medium, lw=3, label='energy_cost')
        energy_ax.legend()

        total_ax.set_ylabel(r'$j_{total}$')
        total_ax.plot(self.com_task_data.time, self.total_cost, color=self.total_color.medium, lw=3, label='total_cost')
        total_ax.set_xlabel('time (sec)')
        total_ax.legend()
        saveAndShow(fig, show_plot, save_dir, filename)


    def plotCostPercentages(self, show_plot=False, save_dir=None, filename='plotCostPercentages'):

        # normalize costs
        self.com_tracking_cost_normalized = (self.com_tracking_cost / self.total_cost)*100.0/self.max_total_cost
        self.rh_tracking_cost_normalized = (self.rh_tracking_cost / self.total_cost)*100.0/self.max_total_cost
        self.com_goal_cost_normalized = (self.com_goal_cost / self.total_cost)*100.0/self.max_total_cost
        self.rh_goal_cost_normalized = (self.rh_goal_cost / self.total_cost)*100.0/self.max_total_cost
        self.energy_cost_normalized = (self.energy_cost / self.total_cost)*100.0/self.max_total_cost

        # Add them so they stack on top of each other.
        self.rh_tracking_cost_normalized += self.com_tracking_cost_normalized
        self.com_goal_cost_normalized += self.rh_tracking_cost_normalized
        self.rh_goal_cost_normalized += self.com_goal_cost_normalized
        self.energy_cost_normalized += self.rh_goal_cost_normalized

        title = "Cost Percentages During Movement"
        fig, (overlay_ax) = plt.subplots(1, 1, num=title, figsize=(10, 8), facecolor='w', edgecolor='k')
        overlay_ax.plot(self.com_task_data.time, self.com_tracking_cost_normalized, color=self.com_color.light, lw=3)
        overlay_ax.plot(self.com_task_data.time, self.rh_tracking_cost_normalized, color=self.rh_color.light, lw=3)
        overlay_ax.plot(self.com_task_data.time, self.com_goal_cost_normalized, color=self.com_color.dark, lw=3)
        overlay_ax.plot(self.com_task_data.time, self.rh_goal_cost_normalized, color=self.rh_color.dark, lw=3)
        overlay_ax.plot(self.com_task_data.time, self.energy_cost_normalized, color=self.energy_color.medium, lw=3)

        overlay_ax.fill_between(self.com_task_data.time, self.rh_goal_cost_normalized, self.energy_cost_normalized, color=self.energy_color.medium, alpha=0.3, label='energy_cost')
        overlay_ax.fill_between(self.com_task_data.time, self.com_goal_cost_normalized, self.rh_goal_cost_normalized, color=self.rh_color.dark, alpha=0.3, label='rh_goal_cost')
        overlay_ax.fill_between(self.com_task_data.time, self.rh_tracking_cost_normalized, self.com_goal_cost_normalized, color=self.com_color.dark, alpha=0.3, label='com_goal_cost')
        overlay_ax.fill_between(self.com_task_data.time, self.com_tracking_cost_normalized, self.rh_tracking_cost_normalized, color=self.rh_color.light, alpha=0.3, label='rh_tracking_cost')
        overlay_ax.fill_between(self.com_task_data.time, 0, self.com_tracking_cost_normalized, color=self.com_color.light, alpha=0.3, label='com_tracking_cost')


        overlay_ax.set_ylabel('Percent of Total Cost')
        overlay_ax.set_xlabel('time (sec)')
        overlay_ax.set_title(title)
        plt.legend()
        saveAndShow(fig, show_plot, save_dir, filename)



    def plotTotalCostPercentages(self, bar_ax=None, idx=0, show_plot=False, save_dir=None, filename='plotTotalCostPercentages'):
        self.com_tracking_cost_normalized_sum = (self.com_tracking_cost.sum() / self.total_cost.sum())*100.0/self.max_total_cost
        self.rh_tracking_cost_normalized_sum = (self.rh_tracking_cost.sum() / self.total_cost.sum())*100.0/self.max_total_cost
        self.com_goal_cost_normalized_sum = (self.com_goal_cost.sum() / self.total_cost.sum())*100.0/self.max_total_cost
        self.rh_goal_cost_normalized_sum = (self.rh_goal_cost.sum() / self.total_cost.sum())*100.0/self.max_total_cost
        self.energy_cost_normalized_sum = (self.energy_cost.sum() / self.total_cost.sum())*100.0/self.max_total_cost


        self.com_tracking_cost_normalized_sum += 0.0
        self.rh_tracking_cost_normalized_sum += self.com_tracking_cost_normalized_sum
        self.com_goal_cost_normalized_sum += self.rh_tracking_cost_normalized_sum
        self.rh_goal_cost_normalized_sum += self.com_goal_cost_normalized_sum
        self.energy_cost_normalized_sum += self.rh_goal_cost_normalized_sum

        title = "Total Cost Percentages"
        if bar_ax is None:
            fig, (bar_ax) = plt.subplots(1, 1, num=title, figsize=(10, 8), facecolor='w', edgecolor='k')
            bar_ax.set_title(title)

        bar_ax.bar(idx, self.energy_cost_normalized_sum, color=self.energy_color.medium, label='energy_cost')
        bar_ax.bar(idx, self.rh_goal_cost_normalized_sum, color=self.rh_color.dark, label='rh_goal_cost')
        bar_ax.bar(idx, self.com_goal_cost_normalized_sum, color=self.com_color.dark, label='com_goal_cost')
        bar_ax.bar(idx, self.rh_tracking_cost_normalized_sum, color=self.rh_color.light, label='rh_tracking_cost')
        bar_ax.bar(idx, self.com_tracking_cost_normalized_sum, color=self.com_color.light, label='com_tracking_cost')

        if bar_ax is None:
            plt.legend()
            saveAndShow(fig, show_plot, save_dir, filename)


    def plotJacobianRanks(self, show_plot=False, save_dir=None, filename='plotJacobianRanks'):

        nRows = []
        ranks = []
        for cj, rj, in zip(self.com_task_data.jacobians, self.rh_task_data.jacobians):
            bigJ = np.vstack((cj, rj))
            nRows.append(np.shape(bigJ)[0])
            ranks.append(np.linalg.matrix_rank(bigJ))

        title = "Concatenated Jacobian Ranks"
        fig, (ranks_ax) = plt.subplots(1, 1, num=title, figsize=(10, 8), facecolor='w', edgecolor='k')

        ranks_ax.plot(self.com_task_data.time, ranks, color=palettes.Oranges().dark, lw=2, label='rank')
        ranks_ax.plot(self.com_task_data.time, nRows, color=palettes.Oranges().light, lw=2, label='n_rows')

        ranks_ax.set_ylim(0, max(nRows)+2)
        ranks_ax.set_xlabel('time (sec)')
        ranks_ax.legend()
        ranks_ax.set_title(title)
        saveAndShow(fig, show_plot, save_dir, filename)



    def plotJointPositions(self, show_plot=False, save_dir=None, filename='plotJointPositions'):

        nFigCols = 5
        nFigRows = 5
        fig, joint_ax_arr = plt.subplots(nFigRows, nFigCols, sharex=True, num="Joint Limits", figsize=(16, 10), facecolor='w', edgecolor='k')

        r = 0
        c = 0
        from . import data
        for i in range(len(self.com_task_data.lower_joint_limits)):
            tmp_ax = joint_ax_arr[r,c]
            tmp_ax.plot(self.com_task_data.time, self.com_task_data.jointPositions[:,i]*180.0/np.pi, color=palettes.YlOrRd().medium, lw=2)
            tmp_ax.axhline(self.com_task_data.lower_joint_limits[i]*180.0/np.pi, color=palettes.YlOrRd().light, lw=2, ls='--')
            tmp_ax.axhline(self.com_task_data.upper_joint_limits[i]*180.0/np.pi, color=palettes.YlOrRd().dark, lw=2, ls='--')
            tmp_ax.set_title(data.icub_joint_names[i])
            c+=1
            if c>= nFigCols:
                r += 1
                c = 0

        # TODO: Fix this bug: https://github.com/matplotlib/matplotlib/issues/5456
        # for i in range(5):
        #     joint_ax_arr[4,i].set_xlabel('t (s)')
        #     joint_ax_arr[i,0].set_ylabel("angle (u'\xb0')")
        # fig.tight_layout()

        saveAndShow(fig, show_plot, save_dir, filename)
