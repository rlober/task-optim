import numpy as np
import os
from datetime import datetime
from .data import *

def verifyCanonicalPath(path):
    newPath = os.path.normpath(path)
    if path[0] == "~":
        newPath = os.path.expanduser("~") + path[1:]

    if os.path.exists(newPath):
        return newPath

    else:
        print("verifyCanonicalPath did not work.")
        return path

def getDataFromFiles(root_dir):
    root_dir_path = verifyCanonicalPath(root_dir)
    print("Parsing data from: "+root_dir_path)
    timeline = np.loadtxt(root_dir_path + "/timeline.txt")
    comExpectedDuration = np.loadtxt(root_dir_path + "/comExpectedDuration.txt")
    rightHandExpectedDuration = np.loadtxt(root_dir_path + "/rightHandExpectedDuration.txt")
    comPositionReal = np.loadtxt(root_dir_path + "/comPositionReal.txt")
    comPositionRef = np.loadtxt(root_dir_path + "/comPositionRef.txt")
    comWaypoints = np.loadtxt(root_dir_path + "/comWaypoints.txt")
    rightHandPositionReal = np.loadtxt(root_dir_path + "/rightHandPositionReal.txt")
    rightHandPositionRef = np.loadtxt(root_dir_path + "/rightHandPositionRef.txt")
    rightHandWaypoints = np.loadtxt(root_dir_path + "/rightHandWaypoints.txt")
    torques = np.loadtxt(root_dir_path + "/torques.txt")
    comBounds = np.loadtxt(root_dir_path + "/comBounds.txt")

    comData = TaskData(timeline, comExpectedDuration, comPositionReal, comPositionRef, comWaypoints, torques, comBounds, "CoM")
    rightHandData = TaskData(timeline, rightHandExpectedDuration, rightHandPositionReal, rightHandPositionRef, rightHandWaypoints, torques, comBounds, "Right_Hand")
    return [comData, rightHandData]

def getDateAndTimeString():
    dt = datetime.now()
    y = dt.year
    mo = dt.month
    d = dt.day
    h = dt.hour
    m = dt.minute
    s = dt.second
    return str(y)+"-"+str(mo).zfill(2)+"-"+str(d).zfill(2)+"_"+str(h).zfill(2)+":"+str(m).zfill(2)+":"+str(s).zfill(2)

def getLastTestDir():
    root_dir = os.path.expanduser("~") + "/Optimization_Tests/"
    dirs = [os.path.join(root_dir,o) for o in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir,o))]
    dirs.sort()
    return dirs[-1]
