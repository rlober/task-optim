from . import base_test


class OneComWaypointStaticTest(base_test.BaseTest):
    """docstring for OneComWaypointStaticTest."""
    def __init__(self, root_path, right_hand_starting_waypoints, com_starting_waypoints):
        super(OneComWaypointStaticTest, self, root_path, right_hand_starting_waypoints, com_starting_waypoints).__init__()

    def getInitialX(self):
        return np.array([self.task_data[0].goal()])

    def extractTaskWaypointsFromSolutionVector(self, x):
        self.com_waypoints = np.array([x])

    def setBounds(self):
        self.X_lower = self.task_data[0].lower_bounds
        self.X_upper = self.task_data[0].upper_bounds
