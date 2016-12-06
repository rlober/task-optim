import os
import re
import sys
sys.path.append("../optimization-scripts")
from task_optim.utils.files import *
import pickle
from . import plot
from . import com_plot
import matplotlib
import matplotlib.pyplot as plt
sys.path.append("../")
from color_palette import palettes

icub_joint_names = ["torso_pitch", "torso_roll", "torso_yaw", "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll", "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"]


class TestData():
    """docstring for TestData."""
    def __init__(self, test_dir):
        self.test_dir = test_dir
        self.dirs = [d for d in os.listdir(self.test_dir) if os.path.isdir(os.path.join(self.test_dir, d))]
        self.iter_dirs = sorted([os.path.join(self.test_dir, d) for d in self.dirs if re.match('Iteration_.*', d)])
        self.opt_dir = [os.path.join(self.test_dir, d) for d in self.dirs if re.match('Optimal_Solution', d)]
        self.n_iterations = len(self.iter_dirs)

        self.opt_data_path = os.path.join(self.test_dir, 'opt_data.pickle')
        self.opt_data = pickle.load( open( self.opt_data_path, 'rb' ) )

        self.solver_parameters_path = os.path.join(self.test_dir, 'solver_parameters.pickle')
        self.solver_parameters = pickle.load( open( self.solver_parameters_path, 'rb' ) )

        self.costs_used_path = os.path.join(self.test_dir, 'costs_used.pickle')
        self.costs_used = pickle.load( open( self.costs_used_path, 'rb' ) )

        self.cost_scaling_factor = self.opt_data['Y_init'][0,0]

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

    def generatePlots(self, save_path=None):
        if save_path is None:
            save_path = self.test_dir + '/plots/'

        self.plot_save_dir = save_path
        # Bar graph stuff
        title = "Total Cost Percentages"
        bar_fig, (bar_ax) = plt.subplots(1, 1, num=title, figsize=(10, 8), facecolor='w', edgecolor='k')

        import timeit
        time_left = '???'
        elapsed = []
        tmp_costs = []

        for i, d in enumerate(self.iteration_data):
            print("Plotting iteration", i, "of", self.n_iterations, "- est. duration:", time_left, "s")
            start_time = timeit.default_timer()

            fn = str(i).zfill(3)
            tmp = plot.DataPlots(d, self.costs_used, self.cost_scaling_factor)
            tmp.plotIndividualCosts(show_plot=False, save_dir=save_path+'/IndividualCosts/', filename=fn)
            tmp.plotCostPercentages(show_plot=False, save_dir=save_path+'/CostPercentages/', filename=fn)
            tmp.plotJacobianRanks(show_plot=False, save_dir=save_path+'/JacobianRanks/', filename=fn)
            tmp.plotJointPositions(show_plot=False, save_dir=save_path+'/JointPositions/', filename=fn)
            tmp.plotTotalCostPercentages(bar_ax, i)
            tmp_costs.append(tmp.total_cost.sum())

            elapsed.append(timeit.default_timer() - start_time)
            time_left = np.mean(elapsed) * (self.n_iterations-i)

        # Add legend and save bar graph
        # There are a bunch of labels and handles because of the approach above so just get the last set.
        handles, labels = bar_ax.get_legend_handles_labels()
        bar_ax.legend(handles[-tmp.n_component_costs:], labels[-tmp.n_component_costs:])
        bar_ax.set_title(title)
        bar_ax.set_xlabel('iteration')
        bar_ax.set_ylabel('%')
        plot.saveAndShow(bar_fig, save_dir=save_path, filename='TotalCostPercentages')

        # Do it for the original and optimal data as well.
        print("Plotting Original task set data.")
        orig = plot.DataPlots(self.original_data, self.costs_used, self.cost_scaling_factor)
        orig.plotIndividualCosts(show_plot=False, save_dir=save_path, filename='Original_Tasks_IndividualCosts')
        orig.plotCostPercentages(show_plot=False, save_dir=save_path, filename='Original_Tasks_CostPercentages')
        orig.plotJacobianRanks(show_plot=False, save_dir=save_path, filename='Original_Tasks_JacobianRanks')
        orig.plotJointPositions(show_plot=False, save_dir=save_path, filename='Original_Tasks_JointPositions')

        print("Plotting Optimal task set data.")
        opt = plot.DataPlots(self.optimal_data, self.costs_used, self.cost_scaling_factor)
        opt.plotIndividualCosts(show_plot=False, save_dir=save_path, filename='Optimal_Tasks_IndividualCosts')
        opt.plotCostPercentages(show_plot=False, save_dir=save_path, filename='Optimal_Tasks_CostPercentages')
        opt.plotJacobianRanks(show_plot=False, save_dir=save_path, filename='Optimal_Tasks_JacobianRanks')
        opt.plotJointPositions(show_plot=False, save_dir=save_path, filename='Optimal_Tasks_JointPositions')

        print("Plotting Original/Optimal comparaisons.")
        orig.compare(opt, save_dir=save_path)

        # Cost figure
        # assert(self.opt_data['Y'][:,0] == tmp_costs)
        print("Plotting the total cost figure.")
        cost_fig, (cost_ax) = plt.subplots(1, 1, num="Cost Curve", figsize=(10, 8), facecolor='w', edgecolor='k')
        cost_ax.plot(self.opt_data['Y'][:,0], color=palettes.Purples().medium, lw=3, label='total costs')
        cost_ax.plot(tmp_costs, color=palettes.Purples().dark, ls='--', lw=3, label='total costs')
        cost_ax.set_xlabel('iteration')
        cost_ax.set_ylabel('cost')
        cost_ax.legend()
        plot.saveAndShow(cost_fig, save_dir=save_path, filename='CostCurve')

        # CoM Scatter figure
        print("Plotting the CoM scatter plot figure.")
        lower_bounds = self.optimal_data[0].lower_bounds
        upper_bounds = self.optimal_data[0].upper_bounds
        com_fig, com_ax = com_plot.plot3dScatter(self.opt_data['X'], self.opt_data['Y'], lower_bounds, upper_bounds)
        plot.saveAndShow(com_fig, save_dir=save_path, filename='CoMScatter')

    def generateHtml(self):
        html_path = os.path.join(self.plot_save_dir, 'results.html')

        f = open(html_path,'w')

        images = ['CostCurve.png', 'GoalCostComparaison.png', 'Optimal_Tasks_CostPercentages.png', 'Optimal_Tasks_IndividualCosts.png', 'Optimal_Tasks_JacobianRanks.png', 'Optimal_Tasks_JointPositions.png', 'Original_Tasks_CostPercentages.png', 'Original_Tasks_IndividualCosts.png', 'Original_Tasks_JacobianRanks.png', 'Original_Tasks_JointPositions.png', 'TotalCostComparaison.png', 'TotalCostPercentages.png', 'TrackingCostComparaison.png']

        html_body = "<html><head></head><body><h1>"+self.plot_save_dir+"</h1><p>"+str(self.solver_parameters)+"</p>"
        for i in images:
            im_path = os.path.join(self.plot_save_dir, i)
            html_body += "<br><img src='"+im_path+"'>"

        html_body += "</body></html>"

        f.write(html_body)
        f.close()

        return self.opt_data, self.solver_parameters, html_path
