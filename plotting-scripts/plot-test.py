from one_com_waypoint_static import *
import os
import re
import sys
sys.path.append("../optimization-scripts")
from task_optim.utils.files import *

class ColorPalette():
    """Base class for all plotting colors which work well for articles.

    Checkout http://colorbrewer2.org/ to see where these palettes came from.
    """
    def __init__(self, color_dict):
        self.color_dict = color_dict
        self.max_rgb_value = 255

        self.setRgbNormalize(True)

    def normalize(self, rgb_tuple):
        """Converts a 0-255 rgb values to 0-1 values."""
        return tuple(v / self.max_rgb_value for v in rgb_tuple)

    def setRgbNormalize(self, b):
        """Turn normalization on or off.

        :param b: Boolean
        """
        self.use_normalized_rgb = b
        if self.use_normalized_rgb:
            self.light = self.normalize(self.color_dict['light'])
            self.medium = self.normalize(self.color_dict['medium'])
            self.dark = self.normalize(self.color_dict['dark'])
        else:
            self.light = self.color_dict['light']
            self.medium = self.color_dict['medium']
            self.dark = self.color_dict['dark']

    def getRgbNormalize(self):
        """See if normalization is on or off.

        :return b: Boolean
        """
        return self.use_normalized_rgb

    def getLight(self):
        """Returns the lightest shade in the color palette"""
        return self.light

    def getMedium(self):
        """Returns the medium shade in the color palette"""
        return self.medium

    def getDark(self):
        """Returns the darkest shade in the color palette"""
        return self.dark



# Multi Hue:
class BuGn(ColorPalette):
    """A single hue color palette of BuGn."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(229,245,249), 'medium':(153,216,201), 'dark':(44,162,95)})

class BuPu(ColorPalette):
    """A single hue color palette of BuPu."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(224,236,244), 'medium':(158,188,218), 'dark':(136,86,167)})

class GnBu(ColorPalette):
    """A single hue color palette of GnBu."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(224,243,219), 'medium':(168,221,181), 'dark':(67,162,202)})

class OrRd(ColorPalette):
    """A single hue color palette of OrRd."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(254,232,200), 'medium':(253,187,132), 'dark':(227,74,51)})

class PuBu(ColorPalette):
    """A single hue color palette of PuBu."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(236,231,242), 'medium':(166,189,219), 'dark':(43,140,190)})

class PuBuGn(ColorPalette):
    """A single hue color palette of PuBuGn."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(236,226,240),'medium':(166,189,219),'dark':(28,144,153)})


class PuRd(ColorPalette):
    """A single hue color palette of PuRd."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(231,225,239),'medium':(201,148,199),'dark':(221,28,119)})

class RdPu(ColorPalette):
    """A single hue color palette of RdPu."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(253,224,221),'medium':(250,159,181),'dark':(197,27,138)})

class YlGn(ColorPalette):
    """A single hue color palette of YlGn."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(247,252,185),'medium':(173,221,142),'dark':(49,163,84)})

class YlGnBu(ColorPalette):
    """A single hue color palette of YlGnBu."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(237,248,177),'medium':(127,205,187),'dark':(44,127,184)})

class YlOrBr(ColorPalette):
    """A single hue color palette of YlOrBr."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(255,247,188),'medium':(254,196,79),'dark':(217,95,14)})

class YlOrRd(ColorPalette):
    """A single hue color palette of YlOrRd."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(255,237,160),'medium':(254,178,76),'dark':(240,59,32)})


# Single hue:
class Blues(ColorPalette):
    """A single hue color palette of Blues."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(222,235,247),'medium':(158,202,225),'dark':(49,130,189)})

class Greens(ColorPalette):
    """A single hue color palette of Greens."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(229,245,224),'medium':(161,217,155),'dark':(49,163,84)})

class Greys(ColorPalette):
    """A single hue color palette of Greys."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(240,240,240),'medium':(189,189,189),'dark':(99,99,99)})

class Oranges(ColorPalette):
    """A single hue color palette of Oranges."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(254,230,206),'medium':(253,174,107),'dark':(230,85,13)})

class Purples(ColorPalette):
    """A single hue color palette of Purples."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(239,237,245),'medium':(188,189,220),'dark':(117,107,177)})

class Reds(ColorPalette):
    """A single hue color palette of Reds."""
    def __init__(self):
        ColorPalette.__init__(self, {'light':(254,224,210),'medium':(252,146,114),'dark':(222,45,38)})



test_dir = os.path.abspath("./test_data_bo")
# test_dir = os.path.abspath("./test_data_cma")


dirs = [d for d in os.listdir(test_dir) if os.path.isdir(os.path.join(test_dir, d))]

iter_dirs = [os.path.join(test_dir, d) for d in dirs if re.match('Iteration_.*', d)]
opt_dir = [os.path.join(test_dir, d) for d in dirs if re.match('Optimal_Solution', d)]

print("I found "+str(len(iter_dirs))+" iteration directories in the test directory: "+test_dir)

# rh = right hand
com_task_data, rh_task_data = getDataFromFiles(opt_dir[0])

import matplotlib.pyplot as plt

com_tracking_cost = com_task_data.positionErrorSquaredNormTimeAveraged()
rh_tracking_cost = rh_task_data.positionErrorSquaredNormTimeAveraged()

com_goal_cost = com_task_data.goalPositionErrorSquaredNormPenalized()
rh_goal_cost = rh_task_data.goalPositionErrorSquaredNormPenalized()

energy_cost = com_task_data.torquesSquaredNormTimeAveragedScaled()

total_cost = com_tracking_cost + rh_tracking_cost + com_goal_cost + rh_goal_cost + energy_cost

max_total_cost = max(total_cost)

total_cost = total_cost / max_total_cost

fig, (tracking_ax, goal_ax, energy_ax, total_ax) = plt.subplots(4, sharex=True, num=None, figsize=(8, 10), facecolor='w', edgecolor='k')

tracking_ax.set_ylabel(r'$j_{tracking}$')
plot_com_tracking_cost, = tracking_ax.plot(com_task_data.time, com_tracking_cost, color=Blues().light, lw=3, label='com_tracking_cost')
plot_rh_tracking_cost, = tracking_ax.plot(rh_task_data.time, rh_tracking_cost, color=Reds().light, lw=3, label='rh_tracking_cost')
tracking_ax.legend()

goal_ax.set_ylabel(r'$j_{goal}$')
plot_com_goal_cost,  = goal_ax.plot(com_task_data.time, com_goal_cost , color=Blues().dark, lw=3, label='com_goal_cost')
plot_rh_goal_cost, = goal_ax.plot(rh_task_data.time, rh_goal_cost, color=Reds().dark, lw=3, label='rh_goal_cost')
goal_ax.legend()

energy_ax.set_ylabel(r'$j_{energy}$')
plot_energy_cost, = energy_ax.plot(com_task_data.time, energy_cost, color=Greens().medium, lw=3, label='energy_cost')
energy_ax.legend()

total_ax.set_ylabel(r'$j_{total}$')
plot_total_cost, = total_ax.plot(com_task_data.time, total_cost, color=Purples().medium, lw=3, label='total_cost')
total_ax.set_xlabel('time (sec)')
total_ax.legend()

##########################

# normalize costs
com_tracking_cost_normalized = (com_tracking_cost / total_cost)*100.0/max_total_cost
rh_tracking_cost_normalized = (rh_tracking_cost / total_cost)*100.0/max_total_cost
com_goal_cost_normalized = (com_goal_cost / total_cost)*100.0/max_total_cost
rh_goal_cost_normalized = (rh_goal_cost / total_cost)*100.0/max_total_cost
energy_cos_normalized = (energy_cost / total_cost)*100.0/max_total_cost

# Add them so they stack on top of each other.
rh_tracking_cost_normalized += com_tracking_cost_normalized
com_goal_cost_normalized += rh_tracking_cost_normalized
rh_goal_cost_normalized += com_goal_cost_normalized
energy_cos_normalized += rh_goal_cost_normalized

fig2, (overlay_ax) = plt.subplots(1, 1, num="Overlay", figsize=(10, 8), facecolor='w', edgecolor='k')
overlay_ax.plot(com_task_data.time, com_tracking_cost_normalized, color=Blues().light, lw=3)
overlay_ax.plot(com_task_data.time, rh_tracking_cost_normalized, color=Reds().light, lw=3)
overlay_ax.plot(com_task_data.time, com_goal_cost_normalized, color=Blues().dark, lw=3)
overlay_ax.plot(com_task_data.time, rh_goal_cost_normalized, color=Reds().dark, lw=3)
overlay_ax.plot(com_task_data.time, energy_cos_normalized, color=Greens().medium, lw=3)

overlay_ax.fill_between(com_task_data.time, rh_goal_cost_normalized, energy_cos_normalized, color=Greens().medium, alpha=0.3, label='energy_cost')
overlay_ax.fill_between(com_task_data.time, com_goal_cost_normalized, rh_goal_cost_normalized, color=Reds().dark, alpha=0.3, label='rh_goal_cost')
overlay_ax.fill_between(com_task_data.time, rh_tracking_cost_normalized, com_goal_cost_normalized, color=Blues().dark, alpha=0.3, label='com_goal_cost')
overlay_ax.fill_between(com_task_data.time, com_tracking_cost_normalized, rh_tracking_cost_normalized, color=Reds().light, alpha=0.3, label='rh_tracking_cost')
overlay_ax.fill_between(com_task_data.time, 0, com_tracking_cost_normalized, color=Blues().light, alpha=0.3, label='com_tracking_cost')


overlay_ax.set_ylabel('Percent of Total Cost')
overlay_ax.set_xlabel('time (sec)')
plt.legend()


##################################



com_tracking_cost_normalized_sum = (com_tracking_cost.sum() / total_cost.sum())*100.0/max_total_cost
rh_tracking_cost_normalized_sum = (rh_tracking_cost.sum() / total_cost.sum())*100.0/max_total_cost
com_goal_cost_normalized_sum = (com_goal_cost.sum() / total_cost.sum())*100.0/max_total_cost
rh_goal_cost_normalized_sum = (rh_goal_cost.sum() / total_cost.sum())*100.0/max_total_cost
energy_cos_normalized_sum = (energy_cost.sum() / total_cost.sum())*100.0/max_total_cost


com_tracking_cost_normalized_sum += 0.0
rh_tracking_cost_normalized_sum += com_tracking_cost_normalized_sum
com_goal_cost_normalized_sum += rh_tracking_cost_normalized_sum
rh_goal_cost_normalized_sum += com_goal_cost_normalized_sum
energy_cos_normalized_sum += rh_goal_cost_normalized_sum


fig3, (bar_ax) = plt.subplots(1, 1, num="Bar", figsize=(10, 8), facecolor='w', edgecolor='k')
bar_ax.bar(0, energy_cos_normalized_sum, color=Greens().medium, label='energy_cost')
bar_ax.bar(0, rh_goal_cost_normalized_sum, color=Reds().dark, label='rh_goal_cost')
bar_ax.bar(0, com_goal_cost_normalized_sum, color=Blues().dark, label='com_goal_cost')
bar_ax.bar(0, rh_tracking_cost_normalized_sum, color=Reds().light, label='rh_tracking_cost')
bar_ax.bar(0, com_tracking_cost_normalized_sum, color=Blues().light, label='com_tracking_cost')
plt.legend()


######################

nRows = []
ranks = []
for cj, rj, in zip(com_task_data.jacobians, rh_task_data.jacobians):
    bigJ = np.vstack((cj, rj))
    nRows.append(np.shape(bigJ)[0])
    ranks.append(np.linalg.matrix_rank(bigJ))

fig4, (ranks_ax) = plt.subplots(1, 1, num="Ranks", figsize=(10, 8), facecolor='w', edgecolor='k')

ranks_ax.plot(com_task_data.time, ranks, color=Oranges().dark, lw=2, label='rank')
ranks_ax.plot(com_task_data.time, nRows, color=Oranges().light, lw=2, label='n_rows')

ranks_ax.set_ylim(0, max(nRows)+2)
ranks_ax.set_xlabel('time (sec)')

ranks_ax.legend()


######################

nFigCols = 5
nFigRows = 5
fig5, joint_ax_arr = plt.subplots(nFigRows, nFigCols, sharex=True, num="Joint Limits", figsize=(16, 10), facecolor='w', edgecolor='k')

r = 0
c = 0
for i in range(len(com_task_data.lower_joint_limits)):
    tmp_ax = joint_ax_arr[r,c]
    tmp_ax.plot(com_task_data.time, com_task_data.jointPositions[:,i]*180.0/np.pi, color=YlOrRd().medium, lw=2)
    tmp_ax.axhline(com_task_data.lower_joint_limits[i]*180.0/np.pi, color=YlOrRd().light, lw=2, ls='--')
    tmp_ax.axhline(com_task_data.upper_joint_limits[i]*180.0/np.pi, color=YlOrRd().dark, lw=2, ls='--')
    tmp_ax.set_title(data.icub_joint_names[i])
    c+=1
    if c>= nFigCols:
        r += 1
        c = 0
for i in range(5):
    joint_ax_arr[4,i].set_xlabel('time (sec)')
    joint_ax_arr[i,0].set_ylabel('angle (degree)')

fig5.tight_layout()
plt.show()
