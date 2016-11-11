from simulate import *
from files import *
import tkinter as tk
from tkinter import filedialog
import os

root = tk.Tk()
root.withdraw()
init_dir = getLastTestDir()
if os.path.isdir(init_dir+"/Optimal_Solution/"):
    init_dir+="/Optimal_Solution/"

file_path = filedialog.askopenfilename(initialdir=init_dir)
if file_path != "":
    iteration_directory = os.path.split(file_path)[0]
    print("Replaying from: ", iteration_directory)
    pathToRightHandWptFile = iteration_directory + "/rightHandWaypoints_optimal.txt"
    if not os.path.exists(pathToRightHandWptFile):
        pathToRightHandWptFile = iteration_directory + "/rightHandWaypoints.txt"

    pathToComWptFile = iteration_directory + "/comWaypoints_optimal.txt"
    if not os.path.exists(pathToComWptFile):
        pathToComWptFile = iteration_directory + "/comWaypoints.txt"

    simulate(pathToRightHandWptFile, pathToComWptFile, savePath=None, verbose=True, visual=True, askUserForReplay=True, goToHome=True)
