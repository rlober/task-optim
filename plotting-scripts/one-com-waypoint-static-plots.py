# from one_com_waypoint_static import *

# waypoints, costs, lower_bounds, upper_bounds = data_faker.getFakeData(100)
# plot.plot3dScatter(waypoints, costs, lower_bounds, upper_bounds)

from one_com_waypoint_static import *
import os

test_dir = os.path.abspath("./test_data_bo")

test_data = data.TestData(test_dir)
test_data.generatePlots()
