
from task_optim.sim_tools.simulate import *
from task_optim.utils.files import *
import tkinter as tk
from tkinter import filedialog
import os

root = tk.Tk()
root.withdraw()
init_dir = getLastTestDir('parameter_tests-reaching')
if os.path.isdir(init_dir+"/Optimal_Solution/"):
    init_dir+="/Optimal_Solution/"

root_dir = os.path.expanduser("~") + "/Code/bayesian-task-optimization"

file_path = filedialog.askopenfilename(initialdir=init_dir)
if file_path != "":
    iteration_directory = os.path.split(file_path)[0]
    print("Replaying from: ", iteration_directory)
    isOptimal = True
    pathToRightHandWptFile = iteration_directory + "/rightHandWaypoints_optimal.txt"
    if not os.path.exists(pathToRightHandWptFile):
        isOptimal = False
        pathToRightHandWptFile = iteration_directory + "/rightHandWaypoints.txt"

    pathToComWptFile = iteration_directory + "/comWaypoints_optimal.txt"
    if not os.path.exists(pathToComWptFile):
        pathToComWptFile = iteration_directory + "/comWaypoints.txt"


    taskSetPath = root_dir + "/reaching-task-sets/icubGazeboSim/TaskOptimizationTaskSet.xml"
    controller_args = "ocra-icub-server --floatingBase --taskSet " + taskSetPath + " --absolutePath --useOdometry"

    client_args = "reach-client --rightHandWptFile "+ pathToRightHandWptFile +" --comWptFile " + pathToComWptFile + " --home"


    gazebo_args = root_dir + "/gazebo_worlds/balancing.world"



    simulate(controller_args, client_args, gazebo_args, verbose=True, visual=True, askUserForReplay=True)
