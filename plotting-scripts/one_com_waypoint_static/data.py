import os
import re
import sys
sys.path.append("../optimization-scripts")
from task_optim.utils.files import *
import pickle
from . import plot
import matplotlib
import matplotlib.pyplot as plt

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


        com_task_data, rh_task_data = getDataFromFiles(self.opt_dir[0])
        self.optimal_data = [com_task_data, rh_task_data]


    def printInfoAboutTest(self):

        print("I found "+str(self.n_iterations)+" iteration directories in the test directory: "+self.test_dir)

    def generatePlots(self):
        save_path = self.test_dir + '/plots/'

        # Bar graph stuff
        title = "Cost Percentages During Movement"
        bar_fig, (bar_ax) = plt.subplots(1, 1, num=title, figsize=(10, 8), facecolor='w', edgecolor='k')

        import timeit
        time_left = '???'
        elapsed = []
        for i, d in enumerate(self.iteration_data):
            print("Plotting iteration", i, "of", self.n_iterations, "- est. duration:", time_left, "s")
            start_time = timeit.default_timer()

            fn = str(i).zfill(3)
            tmp = plot.DataPlots(d)
            tmp.plotIndividualCosts(show_plot=False, save_dir=save_path+'/IndividualCosts/', filename=fn)
            tmp.plotCostPercentages(show_plot=False, save_dir=save_path+'/CostPercentages/', filename=fn)
            tmp.plotJacobianRanks(show_plot=False, save_dir=save_path+'/JacobianRanks/', filename=fn)
            tmp.plotJointPositions(show_plot=False, save_dir=save_path+'/JointPositions/', filename=fn)
            tmp.plotTotalCostPercentages(bar_ax, i)

            elapsed.append(timeit.default_timer() - start_time)
            time_left = np.mean(elapsed) * (self.n_iterations-i)


        # Do it for the optimal data as well.
        print("Plotting optimal solution")
        tmp = plot.DataPlots(self.optimal_data)
        tmp.plotIndividualCosts(show_plot=False, save_dir=save_path+'/IndividualCosts/', filename='opt')
        tmp.plotCostPercentages(show_plot=False, save_dir=save_path+'/CostPercentages/', filename='opt')
        tmp.plotJacobianRanks(show_plot=False, save_dir=save_path+'/JacobianRanks/', filename='opt')
        tmp.plotJointPositions(show_plot=False, save_dir=save_path+'/JointPositions/', filename='opt')
        tmp.plotTotalCostPercentages(bar_ax, self.n_iterations)

        # Add legend and save bar graph
        # bar_ax.legend()
        handles, labels = bar_ax.get_legend_handles_labels()
        bar_ax.legend(handles[-5:], labels[-5:])
        bar_ax.set_title(title)
        plot.saveAndShow(bar_fig, save_dir=save_path, filename='TotalCostPercentages')
