from simulate import *
import tkinter as tk
from tkinter import filedialog
import os

root_dir = os.path.expanduser("~") + "/Optimization_Tests/"

dirs = [os.path.join(root_dir,o) for o in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir,o))]

root = tk.Tk()
root.withdraw()
file_path = filedialog.askopenfilename(initialdir=dirs[-1])
iteration_directory = os.path.split(file_path)[0]
print("Replaying from: ", iteration_directory)
pathToRightHandWptFile = iteration_directory + "/rightHandWaypoints.txt"
pathToComWptFile = iteration_directory + "/comWaypoints.txt"
simulate(pathToRightHandWptFile, pathToComWptFile, savePath=None, verbose=True, visual=True)
