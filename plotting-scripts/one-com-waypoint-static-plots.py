# from one_com_waypoint_static import *

# waypoints, costs, lower_bounds, upper_bounds = data_faker.getFakeData(100)
# plot.plot3dScatter(waypoints, costs, lower_bounds, upper_bounds)

from one_com_waypoint_static import *
import os

test_dir = os.path.abspath("./test_data_cma/tracking_goal/")


bo_table_begin = "<h2>BO Tests</h2><br><table style='width:80%'><tr><th>link</th><th>max_iter</th><th>tolfun</th><th>par</th><th>kernel</th><th>acquisition</th><th>maximizer</th></tr>"
bo_table_end = "</table>"

cma_table_begin = "<h2>CMA-ES Tests</h2><br><table style='width:80%'><tr><th>link</th><th>max_iter</th><th>tolfun</th><th>initial_sigma</th></tr>"

cma_table_end = "</table>"

bo_table_entries = []
cma_table_entries = []



test_data = data.TestData(test_dir)
test_data.generatePlots()
opt_data, sol_params, html_path = test_data.generateHtml()



entry_link = "<a href='"+ html_path +"'>see trial data</a>"

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

server_root_dir = "/home/ryan/Optimization_Tests/html_test"
html_path = os.path.join(server_root_dir, 'index.html')
f = open(html_path,'w')
html_body = "<html><head><style>table, th, td {text-align: center; border: 2px solid black;}</style></head><body><h1>parameter_tests</h1><br>" + bo_table + "<br><br><br>" + cma_table + "<br></body></html>"
f.write(html_body)
f.close()
