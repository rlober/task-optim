from .base_test import *


class StandUpTest(BaseTest):
    """docstring for StandUpTest."""
    def __init__(self, root_path, com_starting_waypoints, costs, skip_init=False, trial_dir=None):
        self.com_waypoints = com_starting_waypoints
        super(StandUpTest, self).__init__(root_path, costs, skip_init, trial_dir)


    def getInitialX(self):
        return self.task_data[0].middleWaypoints()

    def extractTaskWaypointsFromSolutionVector(self, x):
        self.com_waypoints = np.array([x])
        print("New parameters to test:\nCOM [x, y, z]\n", self.com_waypoints)


    def setBounds(self):
        self.X_lower = self.task_data[0].lower_bounds
        self.X_upper = self.task_data[0].upper_bounds

    def initSkipMethod(self):
        """Any custom things to do when initialization is skipped."""
        self.com_waypoints = com_starting_waypoints

    def getControllerArgs(self):
        # Task set path
        taskSetPath = self.rootPath + "/stand-task-sets/icubGazeboSim/StandUpTasks.xml"
        return "ocra-icub-server --floatingBase --taskSet " + taskSetPath + " --absolutePath --idleAnkles --maintainFinalPosture"

    def getClientArgs(self, isOptimal=False):
        args = "stand-client --comWptFile " + self.com_waypoint_file_path

        return args

    def getGazeboWorld(self):
        # Gazebo world file
        return self.rootPath + "/gazebo_worlds/bench.world"

    def saveWaypointsToFile(self, iteration_dir_path, isOptimal=False):
        if isOptimal:
            self.com_waypoint_file_path = iteration_dir_path + "/comWaypoints_optimal.txt"
        else:
            self.com_waypoint_file_path = iteration_dir_path + "/comWaypoints.txt"

        np.savetxt(self.com_waypoint_file_path, self.com_waypoints)

        print("Saving com waypoints\n", self.com_waypoints, "\n to: ", self.com_waypoint_file_path)
