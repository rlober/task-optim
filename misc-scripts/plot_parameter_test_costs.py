import matplotlib
# Force matplotlib to not use any Xwindows backend.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pickle
import os
import sys
import re
sys.path.append("../plotting-scripts")
from one_com_waypoint_static import *
from color_palette import *
import fileinput


##########################################################################################################

server_root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests')
index_html_path = os.path.join(server_root_dir, 'index.html')

#########################################################################################################



def addPlotsToIndexHtml(rel_paths_to_plot, sub):
    html_content = "<br>"
    for link in rel_paths_to_plot:
        html_content += "<a href='"+ link +"'><img src='" + link + "' width='400'></a>"

    with fileinput.FileInput(index_html_path, inplace=True, backup='.bak') as file:
        for line in file:
            textToSearch = "<h2>BO Tests</h2>"
            if sub == 'cma':
                textToSearch = "<h2>CMA-ES Tests</h2>"
            textToReplace = textToSearch + html_content + "<br>"
            print(line.replace(textToSearch, textToReplace), end='')

def generatePlots(opt_data, costs_used, solver_parameters, sub, save_path):
    optimal_costs = []
    iteration_opt_was_found = []
    n_iters = []
    for i, (opt, cos, sol) in enumerate(zip(opt_data, costs_used, solver_parameters)):
        optimal_costs.append(opt['optimal_cost'][0,0])
        iteration_opt_was_found.append(opt['opt_row'])
        n_iters.append(opt['n_iter'])


    rel_paths_to_plot = []

    if sub == 'bo':
        fname_prefix = 'BO_'
    else:
        fname_prefix = 'CMA_'

    fname = fname_prefix + 'OptimalCosts'
    print("Plotting the", fname, "figure.")
    fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
    ax.plot(optimal_costs, color=palettes.Blues().dark, lw=3)
    ax.set_xlabel('test no.')
    ax.set_ylabel('optimal cost')
    ax.set_ylabel(fname)
    plot.saveAndShow(fig, show_plot=False, save_dir=save_path, filename=fname)
    rel_paths_to_plot.append( "/" + os.path.relpath(os.path.join(save_path,fname+".png"), server_root_dir) )

    fname = fname_prefix + 'IterationOfOptimum'
    print("Plotting the", fname, "figure.")
    fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
    ax.plot(iteration_opt_was_found, color=palettes.Purples().dark, lw=3)
    ax.set_xlabel('test no.')
    ax.set_ylabel('iteration of optimum')
    ax.set_ylabel(fname)
    plot.saveAndShow(fig, show_plot=False, save_dir=save_path, filename=fname)
    rel_paths_to_plot.append( "/" + os.path.relpath(os.path.join(save_path,fname+".png"), server_root_dir) )

    fname = fname_prefix + 'NumberOfIterations'
    print("Plotting the", fname, "figure.")
    fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
    ax.plot(n_iters, color=palettes.Greens().dark, lw=3)
    ax.set_xlabel('test no.')
    ax.set_ylabel('total number of iterations')
    ax.set_ylabel(fname)
    plot.saveAndShow(fig, show_plot=False, save_dir=save_path, filename=fname)
    rel_paths_to_plot.append( "/" + os.path.relpath(os.path.join(save_path,fname+".png"), server_root_dir) )

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
