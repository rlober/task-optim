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
    def __init__(self, task_data, costs_used=['tracking', 'goal', 'energy'], cost_scaling_factor=1.0):
        self.com_task_data = task_data[0]
        self.rh_task_data = task_data[1]
        self.costs_used = costs_used
        self.cost_scaling_factor = cost_scaling_factor

        self.n_component_costs = 0
        self.useTrackingCost = False
        self.useGoalCost = False
        self.useEnergyCost = False

        for c in self.costs_used:
            if c == 'tracking':
                self.useTrackingCost = True

            if c == 'goal':
                self.useGoalCost = True

            if c == 'energy':
                self.useEnergyCost = True


        self.total_cost = np.zeros(np.shape(self.com_task_data.positionErrorSquaredNormTimeAveraged()))

        if self.useTrackingCost:
            self.com_tracking_cost = self.com_task_data.positionErrorSquaredNormTimeAveraged()
            self.rh_tracking_cost = self.rh_task_data.positionErrorSquaredNormTimeAveraged()
            self.n_component_costs += 2

            self.total_cost += self.com_tracking_cost + self.rh_tracking_cost

        if self.useGoalCost:
            self.com_goal_cost = self.com_task_data.goalPositionErrorSquaredNormPenalized()
            self.rh_goal_cost = self.rh_task_data.goalPositionErrorSquaredNormPenalized()
            self.n_component_costs += 2

            self.total_cost += self.com_goal_cost + self.rh_goal_cost

        if self.useEnergyCost:
            self.energy_cost = self.com_task_data.torquesSquaredNormTimeAveragedScaled()
            self.n_component_costs += 1

            self.total_cost += self.energy_cost


        self.total_cost = self.total_cost / self.cost_scaling_factor

        self.com_color = palettes.Blues()
        self.rh_color = palettes.Reds()
        self.energy_color = palettes.Greens()
        self.total_color = palettes.Purples()

    def plotIndividualCosts(self, show_plot=False, save_dir=None, filename='plotIndividualCosts'):

        fig, (tracking_ax, goal_ax, energy_ax, total_ax) = plt.subplots(4, sharex=True, num='Individual Costs', figsize=(8, 10), facecolor='w', edgecolor='k')

        if self.useTrackingCost:
            tracking_ax.set_ylabel(r'$j_{tracking}$')
            tracking_ax.plot(self.com_task_data.time, self.com_tracking_cost, color=self.com_color.light, lw=3, label='com_tracking_cost')
            tracking_ax.plot(self.rh_task_data.time, self.rh_tracking_cost, color=self.rh_color.light, lw=3, label='rh_tracking_cost')
            tracking_ax.legend()

        if self.useGoalCost:
            goal_ax.set_ylabel(r'$j_{goal}$')
            goal_ax.plot(self.com_task_data.time, self.com_goal_cost , color=self.com_color.dark, lw=3, label='com_goal_cost')
            goal_ax.plot(self.rh_task_data.time, self.rh_goal_cost, color=self.rh_color.dark, lw=3, label='rh_goal_cost')
            goal_ax.legend()

        if self.useEnergyCost:
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
        costs = []
        colors = []
        labels = []
        if self.useTrackingCost:
            costs.append( (self.com_tracking_cost / self.total_cost)*100.0/self.cost_scaling_factor )
            costs.append( (self.rh_tracking_cost / self.total_cost)*100.0/self.cost_scaling_factor )
            colors.append(self.com_color.light)
            colors.append(self.rh_color.light)
            labels.append('com_tracking_cost')
            labels.append('rh_tracking_cost')

        if self.useGoalCost:
            costs.append( (self.com_goal_cost / self.total_cost)*100.0/self.cost_scaling_factor )
            costs.append( (self.rh_goal_cost / self.total_cost)*100.0/self.cost_scaling_factor )
            colors.append(self.com_color.dark)
            colors.append(self.rh_color.dark)
            labels.append('com_goal_cost')
            labels.append('rh_goal_cost')

        if self.useEnergyCost:
            costs.append( (self.energy_cost / self.total_cost)*100.0/self.cost_scaling_factor )
            colors.append(self.energy_color.medium)
            labels.append('energy_cost')


        # Add them so they stack on top of each other.
        for i in range(1,len(costs)):
            costs[i] += costs[i-1]


        title = "Cost Percentages During Movement"
        fig, (overlay_ax) = plt.subplots(1, 1, num=title, figsize=(10, 8), facecolor='w', edgecolor='k')
        overlay_ax.fill_between(self.com_task_data.time, 0, costs[0], color=colors[0], alpha=0.3, label=labels[0])
        for i, (cos,col) in enumerate(zip(costs,colors)):
            overlay_ax.plot(self.com_task_data.time, cos, color=col, lw=3)
            if i < len(costs)-1:
                overlay_ax.fill_between(self.com_task_data.time, costs[i], costs[i+1], color=colors[i+1], alpha=0.3, label=labels[i+1])


        overlay_ax.set_ylabel('Percent of Total Cost')
        overlay_ax.set_xlabel('time (sec)')
        overlay_ax.set_title(title)
        overlay_ax.legend()
        saveAndShow(fig, show_plot, save_dir, filename)



    def plotTotalCostPercentages(self, bar_ax=None, idx=0, show_plot=False, save_dir=None, filename='plotTotalCostPercentages'):
        # normalize costs
        costs = []
        colors = []
        labels = []
        if self.useTrackingCost:
            costs.append( (self.com_tracking_cost.sum() / self.total_cost.sum())*100.0/self.cost_scaling_factor )
            costs.append( (self.rh_tracking_cost.sum() / self.total_cost.sum())*100.0/self.cost_scaling_factor )
            colors.append(self.com_color.light)
            colors.append(self.rh_color.light)
            labels.append('com_tracking_cost')
            labels.append('rh_tracking_cost')

        if self.useGoalCost:
            costs.append( (self.com_goal_cost.sum() / self.total_cost.sum())*100.0/self.cost_scaling_factor )
            costs.append( (self.rh_goal_cost.sum() / self.total_cost.sum())*100.0/self.cost_scaling_factor )
            colors.append(self.com_color.dark)
            colors.append(self.rh_color.dark)
            labels.append('com_goal_cost')
            labels.append('rh_goal_cost')

        if self.useEnergyCost:
            costs.append( (self.energy_cost.sum() / self.total_cost.sum())*100.0/self.cost_scaling_factor )
            colors.append(self.energy_color.medium)
            labels.append('energy_cost')


        # Add them so they stack on top of each other.
        for i in range(1,len(costs)):
            costs[i] += costs[i-1]

        title = "Total Cost Percentages"
        if bar_ax is None:
            fig, (bar_ax) = plt.subplots(1, 1, num=title, figsize=(10, 8), facecolor='w', edgecolor='k')
            bar_ax.set_title(title)

        for i, (cos,col,lab) in reversed(list(enumerate(zip(costs,colors,labels)))):
            bar_ax.bar(idx, cos, color=col, label=lab)

        if bar_ax is None:
            bar_ax.set_xlabel('iteration')
            bar_ax.set_ylabel('%')
            bar_ax.legend()
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

    def compare(self, other, show_plot=False, save_dir=None):
        """Compare cost data between two iterations.

        :param other: Another DataPlots object.
        """
        if self.useTrackingCost:
            fig, (ax) = plt.subplots(1, num='Tracking Costs', figsize=(8,6), facecolor='w', edgecolor='k')
            ax.plot(self.com_task_data.time, self.com_tracking_cost, color=self.com_color.light, lw=3, label='com_tracking_cost')
            ax.plot(self.rh_task_data.time, self.rh_tracking_cost, color=self.rh_color.light, lw=3, label='rh_tracking_cost')
            ax.plot(other.com_task_data.time, other.com_tracking_cost, color=other.com_color.light, lw=3, label='com_tracking_cost_other', ls='--')
            ax.plot(other.rh_task_data.time, other.rh_tracking_cost, color=other.rh_color.light, lw=3, label='rh_tracking_cost_other', ls='--')
            ax.legend()
            ax.set_xlabel('time (sec)')
            ax.set_ylabel(r'$j_{tracking}$')
            saveAndShow(fig, show_plot, save_dir, 'TrackingCostComparaison')

        if self.useGoalCost:
            fig, (ax) = plt.subplots(1, num='Goal Costs', figsize=(8,6), facecolor='w', edgecolor='k')
            ax.plot(self.com_task_data.time, self.com_goal_cost, color=self.com_color.dark, lw=3, label='com_goal_cost')
            ax.plot(self.rh_task_data.time, self.rh_goal_cost, color=self.rh_color.dark, lw=3, label='rh_goal_cost')
            ax.plot(other.com_task_data.time, other.com_goal_cost, color=other.com_color.dark, lw=3, label='com_goal_cost_other', ls='--')
            ax.plot(other.rh_task_data.time, other.rh_goal_cost, color=other.rh_color.dark, lw=3, label='rh_goal_cost_other', ls='--')
            ax.legend()
            ax.set_xlabel('time (sec)')
            ax.set_ylabel(r'$j_{goal}$')
            saveAndShow(fig, show_plot, save_dir, 'GoalCostComparaison')

        if self.useEnergyCost:
            fig, (ax) = plt.subplots(1, num='Energy Costs', figsize=(8,6), facecolor='w', edgecolor='k')
            ax.plot(self.com_task_data.time, self.energy_cost, color=self.energy_color.medium, lw=3, label='energy_cost')
            ax.plot(other.com_task_data.time, other.energy_cost, color=other.energy_color.medium, lw=3, label='energy_cost_other', ls='--')
            ax.legend()
            ax.set_xlabel('time (sec)')
            ax.set_ylabel(r'$j_{energy}$')
            saveAndShow(fig, show_plot, save_dir, 'EnergyCostComparaison')

        fig, (ax) = plt.subplots(1, num='Total Costs', figsize=(8,6), facecolor='w', edgecolor='k')
        ax.plot(self.com_task_data.time, self.total_cost, color=self.total_color.medium, lw=3, label='total_cost')
        ax.plot(other.com_task_data.time, other.total_cost, color=other.total_color.medium, lw=3, label='total_cost_other', ls='--')
        ax.set_ylabel(r'$j_{total}$')
        ax.set_xlabel('time (sec)')
        ax.legend()
        saveAndShow(fig, show_plot, save_dir, 'TotalCostComparaison')
