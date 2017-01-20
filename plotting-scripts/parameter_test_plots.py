import pickle
import os
import sys
import re
sys.path.append("../plotting-scripts")
from one_com_waypoint_static.plot import *

class HtmlGenerator():
    """docstring for HtmlGenerator."""
    def __init__(self, server_root_dir):
        self.server_root_dir = server_root_dir

        self.bo_table = ""
        self.cma_table = ""

        self.bo_plots = ""
        self.cma_plots = ""

        self.bo_table_begin = ""
        self.bo_table_end = ""
        self.cma_table_begin = ""
        self.cma_table_end = ""


        self.bo_table_entries = []
        self.cma_table_entries = []
        # self.server_root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests')
        self.index_html_path = os.path.join(self.server_root_dir, 'index.html')

    def regenerateTableTags(self):
        self.bo_table_begin = "<h2>BO Tests</h2>"
        self.bo_table_begin += self.bo_plots
        self.bo_table_begin += """<br>
        <table style='width:80%'>
            <tr>
                <th>test no.</th>
                <th>link</th>
                <th>max_iter</th>
                <th>tolfun</th>
                <th>par</th>
                <th>kernel</th>
                <th>acquisition</th>
                <th>maximizer</th>
            </tr>"""

        self.bo_table_end = "</table>"

        self.cma_table_begin = "<h2>CMA-ES Tests</h2>"
        self.cma_table_begin += self.cma_plots
        self.cma_table_begin += """<br>
            <table style='width:80%'>
                <tr>
                    <th>test no.</th>
                    <th>link</th>
                    <th>max_iter</th>
                    <th>tolfun</th>
                    <th>initial_sigma</th>
                </tr>"""

        self.cma_table_end = "</table>"

    def updateIndexTables(self, opt_data, sol_params, html_path, idx):
        entry_link = "<a href='"+ os.path.relpath(html_path, self.server_root_dir) +"'>see trial data</a>"

        if opt_data['solver'] == 'RoboSolver':
            self.bo_table_entries.append( "<tr>\n" + "<td>" + str(idx) + "</td>\n" + "<td>" + entry_link + "</td>\n" + "<td>" + str(sol_params['max_iter']) + "</td>\n" + "<td>" + str(sol_params['tolfun']) + "</td>\n" + "<td>" + str(sol_params['par']) + "</td>\n" + "<td>" + str(sol_params['kernel']) + "</td>\n" + "<td>" + str(sol_params['acquisition']) + "</td>\n" + "<td>" + str(sol_params['maximizer']) + "</td>\n" + "</tr>\n")
        else:
            self.cma_table_entries.append( "<tr>\n" + "<td>" + str(idx) + "</td>\n" + "<td>" + entry_link + "</td>\n" + "<td>" + str(sol_params['max_iter']) + "</td>\n" + "<td>" + str(sol_params['tolfun']) + "</td>\n" + "<td>" + str(sol_params['initial_sigma']) + "</td>\n" + "</tr>\n")

    def regenerateTables(self):
        self.regenerateTableTags()

        self.bo_table = self.bo_table_begin
        for e in self.bo_table_entries:
            self.bo_table += e
        self.bo_table += self.bo_table_end

        self.cma_table = self.cma_table_begin
        for e in self.cma_table_entries:
            self.cma_table += e
        self.cma_table += self.cma_table_end

        self.regenerateIndex()


    def regenerateIndex(self):
        f = open(self.index_html_path,'w')
        html_body = """<html>
            <head>
                <style>
                    table, th, td {text-align: center; border: 2px solid black;}
                    body {text-align: center}
                </style>
            </head>
            <body>
                <h1>parameter_tests</h1>
                <br>"""
        html_body += self.bo_table
        html_body += "<br><br><br>"
        html_body += self.cma_table
        html_body += """<br></body></html>"""
        f.write(html_body)
        f.close()


    def addPlotsToIndexHtml(self, rel_paths_to_plot, sub):
        html_content = "\n<br>\n"
        for link in rel_paths_to_plot:
            html_content += "<a href='"+ link +"'><img src='" + link + "' width='400'></a>\n"

        rank = 1
        html_content += "<br>\n<h3>Top " + str(self.topN) + " Tests</h3><br>\n"
        html_content += "<table style='width:50%'> <tr> <th>rank</th> <th>test no.</th> <th>cost*iter</th> </tr>\n"
        for i, (c,n) in enumerate(zip(self.top_costs, self.top_test_nos)):
            html_content += "<tr>\n" + "<td>" + str(rank) + ".</td>\n" + "<td>" + str(n) + "</td>\n" + "<td>" + str(c) + "</td>\n" + "</tr>\n"
            rank += 1

        html_content += "</table>\n"

        if sub == 'bo':
            self.bo_plots = html_content
        else:
            self.cma_plots = html_content

        self.regenerateTables()


    def generatePlots(self, opt_data, costs_used, solver_parameters, sub, save_path):
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
        ax.set_title(fname)
        ax.set_xticks(range(len(optimal_costs)), minor=True)
        ax.grid(which='both')
        saveAndShow(fig, show_plot=False, save_dir=save_path, filename=fname)
        rel_paths_to_plot.append( os.path.relpath(os.path.join(save_path,fname+".png"), self.server_root_dir) )

        fname = fname_prefix + 'IterationOfOptimum'
        print("Plotting the", fname, "figure.")
        fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
        ax.plot(iteration_opt_was_found, color=palettes.Purples().dark, lw=3)
        ax.set_xlabel('test no.')
        ax.set_ylabel('iteration of optimum')
        ax.set_title(fname)
        ax.set_xticks(range(len(optimal_costs)), minor=True)
        ax.grid(which='both')
        saveAndShow(fig, show_plot=False, save_dir=save_path, filename=fname)
        rel_paths_to_plot.append( os.path.relpath(os.path.join(save_path,fname+".png"), self.server_root_dir) )

        fname = fname_prefix + 'NumberOfIterations'
        print("Plotting the", fname, "figure.")
        fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
        ax.plot(n_iters, color=palettes.Greens().dark, lw=3)
        ax.set_xlabel('test no.')
        ax.set_ylabel('total number of iterations')
        ax.set_title(fname)
        ax.set_xticks(range(len(optimal_costs)), minor=True)
        ax.grid(which='both')
        saveAndShow(fig, show_plot=False, save_dir=save_path, filename=fname)
        rel_paths_to_plot.append( os.path.relpath(os.path.join(save_path,fname+".png"), self.server_root_dir) )

        fname = fname_prefix + 'CostTimesOptIter'
        print("Plotting the", fname, "figure.")
        fig, (ax) = plt.subplots(1, 1, num=fname, figsize=(10, 8), facecolor='w', edgecolor='k')
        cost_opt = np.array(optimal_costs)*np.array(iteration_opt_was_found)
        cost_opt_min = 10000
        cost_opt_min_index = 0
        idxs = []
        cs = []
        for i, c in enumerate(cost_opt):
            if c > 0:
                idxs.append(i)
                cs.append(c)
                if c < cost_opt_min:
                    cost_opt_min = c
                    cost_opt_min_index = i

        ax.plot(idxs, cs, color=palettes.Greys().light, lw=3)
        self.topN = 5
        if self.topN > len(cs):
            self.topN = len(cs)-2

        self.top_costs, self.top_test_nos = (list(t)[:self.topN] for t in zip(*sorted(zip(cs, idxs))))
        ax.plot(self.top_test_nos, self.top_costs, 'o', color=palettes.Greys().medium, ms=10)
        ax.plot(cost_opt_min_index, cost_opt_min, 's', color=palettes.Greys().dark, ms=10)
        ax.set_xlabel('test no.')
        ax.set_ylabel('cost * opt_iter')
        ax.set_title(fname)
        ax.set_xticks(range(len(optimal_costs)), minor=True)
        ax.grid(which='both')
        saveAndShow(fig, show_plot=False, save_dir=save_path, filename=fname)
        rel_paths_to_plot.append( os.path.relpath(os.path.join(save_path,fname+".png"), self.server_root_dir) )

        self.addPlotsToIndexHtml(rel_paths_to_plot, sub)
        self.regenerateIndex()
##########################################################################################################




def analyseParameterTest(root_dir, test_name='OneComWaypointStaticTest', html_only=False):

    page = HtmlGenerator(root_dir)
    sub_tests_dir = ['bo', 'cma']

    for i,sub in enumerate(sub_tests_dir):

        opt_data_list = []
        costs_used_list = []
        solver_parameters_list = []

        root_tests_dir = os.path.join(root_dir, sub)
        dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
        test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])

        for j, test in enumerate(test_dirs):
            print("\n\n======================================================================")
            print("Analysing test", j, "of", len(test_dirs)-1, "for", sub, "trials.")
            print("======================================================================\n")
            if html_only:
                ## to not plot stuff
                test_data = data.TestData(test, extract_data=False)
            else:
                ## to plot stuff
                test_data = data.TestData(test, extract_data=True)
                test_data.generatePlots()

            opt_data, sol_params, html_path = test_data.generateHtml(page.server_root_dir)
            page.updateIndexTables(opt_data, sol_params, html_path, j)

            opt_data_list.append(test_data.opt_data)
            costs_used_list.append(test_data.solver_parameters)
            solver_parameters_list.append(test_data.costs_used)

        page.generatePlots(opt_data_list, costs_used_list, solver_parameters_list, sub, root_dir)





# root_dir = [os.path.join(os.path.expanduser("~"),'Optimization_Tests/parameter_tests')]
#
# for r in root_dir:
#     analyseParameterTest(r)

# root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests/parameter_tests-stand_up')
# analyseParameterTest(root_dir, 'StandUpTest')

root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests/parameter_tests-reaching')
analyseParameterTest(root_dir, 'OneComWaypointStaticTest', html_only=True)
#
# root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests/parameter_tests-standing')
# analyseParameterTest(root_dir, 'StandUpTest', html_only=True)
