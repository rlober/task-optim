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
        self.use_normalized_rgb = True
        self.max_rgb_value = 255

    def normalize(self, rgb_tuple):
        """Converts a 0-255 rgb values to 0-1 values."""
        return tuple(v / self.max_rgb_value for v in rgb_tuple)

    def setRgbNormalize(self, b):
        """Turn normalization on or off.

        :param b: Boolean
        """
        self.use_normalized_rgb = b

    def getRgbNormalize(self):
        """See if normalization is on or off.

        :return b: Boolean
        """
        return self.use_normalized_rgb

    def light(self):
        """Returns the lightest shade in the color palette"""
        if self.use_normalized_rgb:
            return self.normalize(self.color_dict['light'])
        else:
            return self.color_dict['light']

    def medium(self):
        """Returns the medium shade in the color palette"""
        if self.use_normalized_rgb:
            return self.normalize(self.color_dict['medium'])
        else:
            return self.color_dict['medium']

    def dark(self):
        """Returns the darkest shade in the color palette"""
        if self.use_normalized_rgb:
            return self.normalize(self.color_dict['dark'])
        else:
            return self.color_dict['dark']



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

total_cost = total_cost / max(total_cost)

fig, axarr = plt.subplots(4, sharex=True, num=None, figsize=(8, 10), facecolor='w', edgecolor='k')

axarr[0].set_ylabel(r'$j_{tracking}$')
tc_plot_color_palette = Blues()
print(tc_plot_color_palette.light())
plot_com_tracking_cost, = axarr[0].plot(com_task_data.time, com_tracking_cost, color=tc_plot_color_palette.light(), lw=3, label='com_tracking_cost')
plot_rh_tracking_cost, = axarr[0].plot(rh_task_data.time, rh_tracking_cost, 'b', lw=3, label='rh_tracking_cost')
axarr[0].legend()

axarr[1].set_ylabel(r'$j_{goal}$')
plot_com_goal_cost,  = axarr[1].plot(com_task_data.time, com_goal_cost , 'r', lw=3, label='com_goal_cost')
plot_rh_goal_cost, = axarr[1].plot(rh_task_data.time, rh_goal_cost, 'b', lw=3, label='rh_goal_cost')
axarr[1].legend()

axarr[2].set_ylabel(r'$j_{energy}$')
plot_energy_cost, = axarr[2].plot(com_task_data.time, energy_cost, 'g', lw=3, label='energy_cost')
axarr[2].legend()

axarr[3].set_ylabel(r'$j_{total}$')
plot_total_cost, = axarr[3].plot(com_task_data.time, total_cost, 'k', lw=3, label='total_cost')
axarr[3].set_xlabel('time (sec)')
axarr[3].legend()




plt.show()
