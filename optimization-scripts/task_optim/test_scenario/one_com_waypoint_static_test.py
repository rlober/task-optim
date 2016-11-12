from .base_test import *


class OneComWaypointStaticTest(BaseTest):
    """docstring for OneComWaypointStaticTest."""
    def __init__(self, root_path, right_hand_starting_waypoints, com_starting_waypoints):
        super(OneComWaypointStaticTest, self).__init__(root_path, right_hand_starting_waypoints, com_starting_waypoints)

    def getInitialX(self):
        return np.array([self.task_data[0].goal()])

    def extractTaskWaypointsFromSolutionVector(self, x):
        self.com_waypoints = np.array([x])
        print("New parameters to test:\nCOM [x, y, z]\n", self.com_waypoints)


    def setBounds(self):
        self.X_lower = self.task_data[0].lower_bounds
        self.X_upper = self.task_data[0].upper_bounds
