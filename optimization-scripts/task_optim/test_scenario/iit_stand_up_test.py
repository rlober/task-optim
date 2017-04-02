from .base_test import *
import yarp
import subprocess
import time
import shlex
from task_optim.sim_tools.simulate import GazeboSimulation

class IitStandUpTest(BaseTest):
    """docstring for IitStandUpTest."""
    def __init__(self, root_path, com_starting_waypoints, costs, skip_init=False, trial_dir=None, cost_weights=None, apply_force=False, using_real_robot=False):
        self.com_waypoints = com_starting_waypoints
        self.apply_force = apply_force
        self.yarp_net = yarp.Network()
        self.yarp_net.init()

        self.data_port = yarp.BufferedPortBottle()
        self.data_out_port_name = "/WBIController/learningData:o"
        self.data_port_name = "/TaskOptim/data:i"
        self.data_port.open(self.data_port_name)

        self.using_real_robot = using_real_robot

        self.data_bottle = yarp.Bottle()

        self.com_real = []
        self.com_ref = []
        self.torques = []
        self.time = []

        self.traj_gen = None
        self.simulink_ctrl = None

        if not self.using_real_robot:
            verbose = True
            visual = True
            self.gazebo = GazeboSimulation(self.getGazeboWorld(), verbose, visual)

        # Create com bounds:
        # self.com_bounds = np.array([[-0.106483, 0.08], [-0.226473, 0.02], [0.220648, 0.52]])
        self.com_bounds = np.array([[-0.106483, 0.2], [-0.226473, 0.02], [0.1, 0.52]])
        super(IitStandUpTest, self).__init__(root_path, costs, skip_init, trial_dir, cost_weights)

    def __del__(self):
        self.traj_gen.kill()
        self.data_port.close()

    def waitForDataPortConnection(self):
        self.yarp_net.init()
        while not self.yarp_net.connect(self.data_out_port_name, self.data_port_name):
            time.sleep(0.01)

        print("Data port connected.")


    def listenForData(self):
        print("Listening for data.")
        while self.yarp_net.isConnected(self.data_out_port_name, self.data_port_name):
            self.data_bottle = self.data_port.read(False)
            if self.data_bottle is not None:
                real = []
                ref = []
                tor = []
                for i in range(3):
                    real.append(self.data_bottle.get(i).asDouble())

                for i in range(3,6):
                    ref.append(self.data_bottle.get(i).asDouble())

                for i in range(6,self.data_bottle.size()-1):
                    tor.append(self.data_bottle.get(i).asDouble())

                self.com_real.append(real)
                self.com_ref.append(ref)
                self.torques.append(tor)
                self.time.append(self.data_bottle.get(self.data_bottle.size()-1).asDouble())

                self.data_bottle.clear()

        print("Ports disconnected. Listening stopped. Converting to numpy arrays.")
        com_real_np = np.array(self.com_real)
        com_ref_np = np.array(self.com_ref)
        torques_np = np.array(self.torques)
        time_np = np.array(self.time)
        print("-------------------------------")
        print("com_real_np shape:", com_real_np.shape)
        print("com_ref_np shape:", com_ref_np.shape)
        print("torques_np shape:", torques_np.shape)
        print("time_np shape:", time_np.shape)
        print("-------------------------------")
        print("Saving to file.")
        np.savetxt(self.iteration_dir_path + "/comPositionReal.txt", com_real_np)
        np.savetxt(self.iteration_dir_path + "/comPositionRef.txt", com_ref_np)
        np.savetxt(self.iteration_dir_path + "/torques.txt", torques_np)
        np.savetxt(self.iteration_dir_path + "/timeline.txt", time_np)
        np.savetxt(self.iteration_dir_path + "/comBounds.txt", self.com_bounds)


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
        pass

    def getClientArgs(self, isOptimal=False):
        pass

    def getGazeboWorld(self):
        # Gazebo world file
        return "/home/ryan/Code/codyco-superbuild/main/icub-gazebo-wholebody/worlds/icub_standup_world/icub_standup_world"

    def saveWaypointsToFile(self, iteration_dir_path, isOptimal=False):
        if isOptimal:
            self.com_waypoint_file_path = iteration_dir_path + "/comWaypoints_optimal.txt"
        else:
            self.com_waypoint_file_path = iteration_dir_path + "/comWaypoints.txt"

        np.savetxt(self.com_waypoint_file_path, self.com_waypoints)

        print("Saving com waypoints\n", self.com_waypoints, "\n to: ", self.com_waypoint_file_path)

    def launchTrajectoryGenerator(self):
        if self.traj_gen is not None:
            print("killing old traj gen")
            self.traj_gen.kill()

        traj_gen_args =  "com-traj-gen "
        traj_gen_args += "--x " + str(self.com_waypoints[0,0]) + " "
        traj_gen_args += "--y " + str(self.com_waypoints[0,1]) + " "
        traj_gen_args += "--z " + str(self.com_waypoints[0,2]) + " "
        traj_gen_args += "--savePath " + self.iteration_dir_path

        args = shlex.split(traj_gen_args)

        self.traj_gen = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def startSimulinkController(self):
        pass
        #
        # if self.simulink_ctrl is not None:
        #     print("killing old simulink_ctrl")
        #     self.simulink_ctrl.kill()
        #
        # args = shlex.split("matlab -nodisplay -nosplash -nodesktop -r '/home/ryan/Code/codyco-superbuild/main/WBIToolboxControllers/controllers/torqueBalancing/launchStandUpController.m'; exit;")
        # self.simulink_ctrl = subprocess.Popen(args)#, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def trySimulation(self):
        self.launchTrajectoryGenerator()

        if not self.using_real_robot:
            self.gazebo.reset()
            self.startSimulinkController()

        self.waitForDataPortConnection()
        self.listenForData()
        try:
            print("Parsing task data.")
            self.task_data = getDataFromFiles(self.iteration_dir_path)
        except:
            print("[ERROR] Couldn't parse task data from files.")
