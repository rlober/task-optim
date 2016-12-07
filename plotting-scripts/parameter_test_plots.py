import pickle
import os
import sys
import re
sys.path.append("../plotting-scripts")
from one_com_waypoint_static.plot import *

##########################################################################################################

bo_table_begin = "<h2>BO Tests</h2><br><table style='width:80%'><tr><th>link</th><th>max_iter</th><th>tolfun</th><th>par</th><th>kernel</th><th>acquisition</th><th>maximizer</th></tr>"
bo_table_end = "</table>"

cma_table_begin = "<h2>CMA-ES Tests</h2><br><table style='width:80%'><tr><th>link</th><th>max_iter</th><th>tolfun</th><th>initial_sigma</th></tr>"

cma_table_end = "</table>"

bo_table_entries = []
cma_table_entries = []
server_root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests')
index_html_path = os.path.join(server_root_dir, 'index.html')

def createIndexHtml(opt_data, sol_params, html_path):

    entry_link = "<a href='"+ os.path.relpath(html_path, server_root_dir) +"'>see trial data</a>"

    if opt_data['solver'] == 'RoboSolver':
        bo_table_entries.append( "<tr>" + "<td>" + entry_link + "</td>" + "<td>" + str(sol_params['max_iter']) + "</td>" + "<td>" + str(sol_params['tolfun']) + "</td>" + "<td>" + str(sol_params['par']) + "</td>" + "<td>" + str(sol_params['kernel']) + "</td>" + "<td>" + str(sol_params['acquisition']) + "</td>" + "<td>" + str(sol_params['maximizer']) + "</td>" + "</tr>")
    else:
        cma_table_entries.append( "<tr>" + "<td>" + entry_link + "</td>" + "<td>" + str(sol_params['max_iter']) + "</td>" + "<td>" + str(sol_params['tolfun']) + "</td>" + "<td>" + str(sol_params['initial_sigma']) + "</td>" + "</tr>")


    bo_table = bo_table_begin
    for e in bo_table_entries:
        bo_table += e
    bo_table += bo_table_end

    cma_table = cma_table_begin
    for e in cma_table_entries:
        cma_table += e
    cma_table += cma_table_end


    f = open(index_html_path,'w')
    html_body = "<html><head><style>table, th, td {text-align: center; border: 2px solid black;}</style></head><body><h1>parameter_tests</h1><br>" + bo_table + "<br><br><br>" + cma_table + "<br></body></html>"
    f.write(html_body)
    f.close()


##########################################################################################################


root_dir = os.path.join(os.path.expanduser("~"),'Optimization_Tests/parameter_tests')

sub_tests_dir = ['bo', 'cma']

for i,sub in enumerate(sub_tests_dir):

    root_tests_dir = os.path.join(root_dir, sub)

    dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]

    test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match('OneComWaypointStaticTest.*', d)])

    for j, test in enumerate(test_dirs):
        print("\n\n======================================================================")
        print("Analysing test", j, "of", len(test_dirs), "for", sub, "trials.")
        print("======================================================================\n")
        test_data = data.TestData(test)
        test_data.generatePlots()
        opt_data, sol_params, html_path = test_data.generateHtml(server_root_dir)
        createIndexHtml(opt_data, sol_params, html_path)
