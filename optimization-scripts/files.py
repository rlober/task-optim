import numpy as np
import os
from datetime import datetime
from data import *

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

    comData = TaskData(timeline, comExpectedDuration, comPositionReal, comPositionRef, comWaypoints, torques, "CoM")
    rightHandData = TaskData(timeline, rightHandExpectedDuration, rightHandPositionReal, rightHandPositionRef, rightHandWaypoints, torques, "Right_Hand")
    return [comData, rightHandData]

def getDateAndTimeString():
    dt = datetime.now()
    y = dt.year
    m = dt.month
    d = dt.day
    h = dt.hour
    m = dt.minute
    s = dt.second
    return str(y)+"-"+str(m).zfill(2)+"-"+str(d).zfill(2)+"_"+str(h).zfill(2)+":"+str(m).zfill(2)+":"+str(s).zfill(2)
