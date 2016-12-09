from .base_test import *


class OneComWaypointStaticTest(BaseTest):
    """docstring for OneComWaypointStaticTest."""
    def __init__(self, root_path, right_hand_starting_waypoints, com_starting_waypoints, costs, skip_init=False, trial_dir=None):
        self.right_hand_waypoints = right_hand_starting_waypoints
        self.com_waypoints = com_starting_waypoints
        super(OneComWaypointStaticTest, self).__init__(root_path, costs, skip_init, trial_dir)

    def initSkipMethod(self):
        """Any custom things to do when initialization is skipped."""
        self.right_hand_waypoints = right_hand_starting_waypoints

    def getInitialX(self):
        return np.array([self.task_data[0].goal()])

    def extractTaskWaypointsFromSolutionVector(self, x):
        self.com_waypoints = np.array([x])
        print("New parameters to test:\nCOM [x, y, z]\n", self.com_waypoints)


    def setBounds(self):
        self.X_lower = self.task_data[0].lower_bounds
        self.X_upper = self.task_data[0].upper_bounds

    def getControllerArgs(self):
        # args = "ocra-icub-server --floatingBase --controllerType HOCRA --solver QPOASES --taskSet " + taskSetPath + " --absolutePath"
        # Task set path
        taskSetPath = self.rootPath + "/reaching-task-sets/icubGazeboSim/TaskOptimizationTaskSet.xml"
        return "ocra-icub-server --floatingBase --taskSet " + taskSetPath + " --absolutePath --useOdometry"

    def getClientArgs(self, isOptimal=False):
        args = "reach-client --rightHandWptFile "+ self.right_hand_waypoint_file_path +" --comWptFile " + self.com_waypoint_file_path

        if isOptimal:
            args += " --home"

        return args

    def getGazeboWorld(self):
        # Gazebo world file
        return self.rootPath + "/gazebo_worlds/balancing.world"

    def saveWaypointsToFile(self, iteration_dir_path, isOptimal=False):
        if isOptimal:
            self.right_hand_waypoint_file_path = iteration_dir_path + "/rightHandWaypoints_optimal.txt"
            self.com_waypoint_file_path = iteration_dir_path + "/comWaypoints_optimal.txt"
        else:
            self.right_hand_waypoint_file_path = iteration_dir_path + "/rightHandWaypoints.txt"
            self.com_waypoint_file_path = iteration_dir_path + "/comWaypoints.txt"

        np.savetxt(self.right_hand_waypoint_file_path, self.right_hand_waypoints)
        np.savetxt(self.com_waypoint_file_path, self.com_waypoints)

        print("Saving com waypoints\n", self.com_waypoints, "\n to: ", self.com_waypoint_file_path)
        print("Saving right hand waypoints\n", self.right_hand_waypoints, "\n to: ", self.right_hand_waypoint_file_path)
