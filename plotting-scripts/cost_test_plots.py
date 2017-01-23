import pickle
import os
import sys
import re
sys.path.append("../plotting-scripts")
from one_com_waypoint_static.plot import *

def analyseCostTests(pickle_path):
    cost_test_pickle = pickle.load( open( pickle_path, 'rb' ) )

    sub_tests_dir = ['bo', 'cma']

    # cost_combos:
    combo_0 = ['tracking', 'goal', 'energy']
    combo_1 = ['tracking', 'energy']
    combo_2 = ['tracking', 'goal']
    combo_3 = ['goal', 'energy']
    combo_4 = ['tracking']
    combo_5 = ['goal']
    combo_6 = ['energy']


    bo_cma_data = []

    for i,sub in enumerate(sub_tests_dir):

        combo_0_data = []
        combo_1_data = []
        combo_2_data = []
        combo_3_data = []
        combo_4_data = []
        combo_5_data = []
        combo_6_data = []


        dct = cost_test_pickle[sub]
        total_original_costs = dct['total_original_costs']
        total_optimal_costs = dct['total_optimal_costs']
        optimal_solution_iterations = dct['optimal_solution_iterations']
        number_total_iterations = dct['number_total_iterations']
        costs_used = dct['costs_used']

        for j, (costs, orig_cost, opt_cost, opt_iter, n_iter) in enumerate(zip(costs_used, total_original_costs, total_optimal_costs, optimal_solution_iterations, number_total_iterations)):
            if costs == combo_0:
                combo_0_data.append([orig_cost, opt_cost, opt_iter, n_iter])
            elif costs == combo_1:
                combo_1_data.append([orig_cost, opt_cost, opt_iter, n_iter])
            elif costs == combo_2:
                combo_2_data.append([orig_cost, opt_cost, opt_iter, n_iter])
            elif costs == combo_3:
                combo_3_data.append([orig_cost, opt_cost, opt_iter, n_iter])
            elif costs == combo_4:
                combo_4_data.append([orig_cost, opt_cost, opt_iter, n_iter])
            elif costs == combo_5:
                combo_5_data.append([orig_cost, opt_cost, opt_iter, n_iter])
            elif costs == combo_6:
                combo_6_data.append([orig_cost, opt_cost, opt_iter, n_iter])
            else:
                print('Cost combination is invalid for', j)

        bo_cma_data.append([combo_0_data, combo_1_data, combo_2_data, combo_3_data, combo_4_data, combo_5_data, combo_6_data])


    import numpy as np


    bo_cost_means = []
    bo_cost_std = []

    cma_cost_means = []
    cma_cost_std = []

    for d in bo_cma_data[0]:
        orig = []
        opt = []
        for s in d:
            orig.append(s[0])
            opt.append(s[1])

        costs = np.array(opt) / np.array(orig) # opt_costs/orig_costs
        bo_cost_means.append( np.mean(costs) )
        bo_cost_std.append( np.std(costs) )
    for d in bo_cma_data[1]:
        orig = []
        opt = []
        for s in d:
            orig.append(s[0])
            opt.append(s[1])

        costs = np.array(opt) / np.array(orig) # opt_costs/orig_costs
        cma_cost_means.append( np.mean(costs) )
        cma_cost_std.append( np.std(costs) )

    import matplotlib.pyplot as plt

    N = 7

    ind = np.arange(N)  # the x locations for the groups
    width = 0.35       # the width of the bars

    fig, ax = plt.subplots()
    rects1 = ax.bar(ind, bo_cost_means, width, color='r', yerr=bo_cost_std)

    rects2 = ax.bar(ind + width, cma_cost_means, width, color='y', yerr=cma_cost_std)

    # add some text for labels, title and axes ticks
    ax.set_ylabel('Optimal Cost')
    ax.set_xticks(ind + width / 2)
    ax.set_xticklabels((combo_0, combo_1, combo_2, combo_3, combo_4, combo_5, combo_6))

    ax.legend((rects1[0], rects2[0]), ('BO', 'CMA-ES'))


    def autolabel(rects):
        """
        Attach a text label above each bar displaying its height
        """
        for rect in rects:
            height = rect.get_height()
            ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                    '%d' % float(height),
                    ha='center', va='bottom')

    autolabel(rects1)
    autolabel(rects2)

    plt.show()


def extractCostTestsData(root_dir, test_name='OneComWaypointStaticTest'):

    bo_total_original_costs = []
    bo_total_optimal_costs = []
    bo_optimal_solution_iterations = []
    bo_number_total_iterations = []
    bo_costs_used = []

    cma_total_original_costs = []
    cma_total_optimal_costs = []
    cma_optimal_solution_iterations = []
    cma_number_total_iterations = []
    cma_costs_used = []


    sub_tests_dir = ['bo', 'cma']
    for i,sub in enumerate(sub_tests_dir):
        root_tests_dir = os.path.join(root_dir, sub)
        dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
        test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])

        for j, test in enumerate(test_dirs):
            print("Analysing test", j, "of", len(test_dirs)-1, "for", sub, "trials.")
            test_data = data.TestData(test, extract_data=False)

            if sub == 'bo':
                bo_total_original_costs.append(test_data.opt_data['original_cost'])
                bo_total_optimal_costs.append(test_data.opt_data['optimal_cost'])
                bo_optimal_solution_iterations.append(test_data.opt_data['opt_row'])
                bo_number_total_iterations.append(test_data.opt_data['n_iter'])
                bo_costs_used.append(test_data.costs_used)

            else:
                cma_total_original_costs.append(test_data.opt_data['original_cost'])
                cma_total_optimal_costs.append(test_data.opt_data['optimal_cost'])
                cma_optimal_solution_iterations.append(test_data.opt_data['opt_row'])
                cma_number_total_iterations.append(test_data.opt_data['n_iter'])
                cma_costs_used.append(test_data.costs_used)

    bo_dict = { 'total_original_costs':bo_total_original_costs, 'total_optimal_costs':bo_total_optimal_costs, 'optimal_solution_iterations':bo_optimal_solution_iterations, 'number_total_iterations':bo_number_total_iterations, 'costs_used':bo_costs_used }
    cma_dict = { 'total_original_costs':cma_total_original_costs, 'total_optimal_costs':cma_total_optimal_costs, 'optimal_solution_iterations':cma_optimal_solution_iterations, 'number_total_iterations':cma_number_total_iterations, 'costs_used':cma_costs_used }

    data_dict = {'bo':bo_dict, 'cma':cma_dict}
    pickle_path = os.path.join(root_dir, "cost_test_plotting_data.pickle")
    pickle.dump(data_dict, open( pickle_path, "wb" ) )
    print("Saved data to:", pickle_path)

    return pickle_path





root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests/cost_tests-reaching')
# extractCostTestsData(root_dir, 'OneComWaypointStaticTest')

pickle_path = os.path.join(os.path.expanduser("~"),'Optimization_Tests/cost_tests-reaching/cost_test_plotting_data.pickle')
analyseCostTests(pickle_path)
