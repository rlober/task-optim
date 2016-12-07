import matplotlib.pyplot as plt
import pickle
import os
import sys
import re
sys.path.append("../plotting-scripts")
from one_com_waypoint_static.plot import *
from color_palette import *
import fileinput


##########################################################################################################

server_root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests')
index_html_path = os.path.join(server_root_dir, 'index.html')

#########################################################################################################



def addPlotsToIndexHtml(rel_paths_to_plot, sub):
    html_content = "<br>"
    for link in rel_paths_to_plot:
        html_content += "<a href='"+ link +"'><img src='" + link + "' width='200'></a>"

    with fileinput.FileInput(index_html_path, inplace=True, backup='.bak') as file:
        for line in file:
            textToSearch = "<h2>BO Tests</h2>"
            if sub = 'cma'
                textToSearch = "<h2>CMA-ES Tests</h2>"
            textToReplace = textToSearch + html_content + "<br>"
            print(line.replace(textToSearch, textToReplace), end='')

def generatePlots(opt_data, costs_used, solver_parameters, sub, save_path):
    optimal_costs = []
    iteration_opt_was_found = []
    n_iters = []
    for i, (opt, cos, sol) in enumerate(zip(opt_data, costs_used, solver_parameters)):
        optimal_costs.append(opt_data['optimal_cost'])
        iteration_opt_was_found.append(opt_data['opt_row'])
        n_iters.append(opt_data['n_iter'])


    rel_paths_to_plot = []


    fname = 'OptimalCosts'
    print("Plotting the", fname, "figure.")
    fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
    ax.plot(optimal_costs, color=palettes.Blues().dark, lw=3)
    ax.set_xlabel('test no.')
    ax.set_ylabel('optimal cost')
    plot.saveAndShow(fig, save_dir=save_path, filename=fname)
    rel_paths_to_plot.append( "/" + os.path.relpath(os.path.join(save_path,fname), server_root_dir) )

    fname = 'IterationOfOptimum'
    print("Plotting the", fname, "figure.")
    fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
    ax.plot(iteration_opt_was_found, color=palettes.Purples().dark, lw=3)
    ax.set_xlabel('test no.')
    ax.set_ylabel('iteration of optimum')
    plot.saveAndShow(fig, save_dir=save_path, filename=fname)
    rel_paths_to_plot.append( "/" + os.path.relpath(os.path.join(save_path,fname), server_root_dir) )

    fname = 'NumberOfIterations'
    print("Plotting the", fname, "figure.")
    fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
    ax.plot(n_iters, color=palettes.Greens().dark, lw=3)
    ax.set_xlabel('test no.')
    ax.set_ylabel('total number of iterations')
    plot.saveAndShow(fig, save_dir=save_path, filename=fname)
    rel_paths_to_plot.append( "/" + os.path.relpath(os.path.join(save_path,fname), server_root_dir) )

    addPlotsToIndexHtml(rel_paths_to_plot, sub)


##########################################################################################################



root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests/parameter_tests')

sub_tests_dir = ['bo', 'cma']

for i,sub in enumerate(sub_tests_dir):

    opt_data = []
    costs_used = []
    solver_parameters = []

    root_tests_dir = os.path.join(root_dir, sub)

    dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]

    test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match('OneComWaypointStaticTest.*', d)])

    for j, test in enumerate(test_dirs):
        print("Extracting data from test", j, "of", len(test_dirs), "for", sub, "trials.")
        opt_data.append( pickle.load( open( os.path.join(test, 'opt_data.pickle'), 'rb' ) ) )
        costs_used.append( pickle.load( open( os.path.join(test, 'costs_used.pickle'), 'rb' ) ) )
        solver_parameters.append( pickle.load( open( os.path.join(test, 'solver_parameters.pickle'), 'rb' ) ) )

    generatePlots(opt_data, costs_used, solver_parameters, sub, root_dir)
