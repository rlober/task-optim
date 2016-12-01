import os
import re
import sys
sys.path.append("../optimization-scripts")
from task_optim.utils.files import *
import pickle

icub_joint_names = ["torso_pitch", "torso_roll", "torso_yaw", "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll", "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"]


class TestData():
    """docstring for TestData."""
    def __init__(self, test_dir):
        self.test_dir = test_dir
        self.dirs = [d for d in os.listdir(self.test_dir) if os.path.isdir(os.path.join(self.test_dir, d))]
        self.iter_dirs = [os.path.join(self.test_dir, d) for d in self.dirs if re.match('Iteration_.*', d)]
        self.opt_dir = [os.path.join(self.test_dir, d) for d in self.dirs if re.match('Optimal_Solution', d)]
        self.n_iterations = len(self.iter_dirs)

        self.opt_data_path = os.path.join(self.test_dir, 'opt_data.pickle')
        self.opt_data = pickle.load( open( self.opt_data_path, 'rb' ) )

        self.solver_parameters_path = os.path.join(self.test_dir, 'solver_parameters.pickle')
        self.solver_parameters = pickle.load( open( self.solver_parameters_path, 'rb' ) )

        self.extractDataFromTest()

        self.printInfoAboutTest()

    def extractDataFromTest(self):
        self.iteration_data = []
        # rh = right hand
        for d in self.iter_dirs:
            com_task_data, rh_task_data = getDataFromFiles(d)
            self.iteration_data.append([com_task_data, rh_task_data])

        com_task_data, rh_task_data = getDataFromFiles(self.iter_dirs[0])
        self.original_data = [com_task_data, rh_task_data]

        com_task_data, rh_task_data = getDataFromFiles(self.opt_dir)
        self.optimal_data = [com_task_data, rh_task_data]


    def printInfoAboutTest(self):

        print("I found "+str(self.n_iterations)+" iteration directories in the test directory: "+self.test_dir)
