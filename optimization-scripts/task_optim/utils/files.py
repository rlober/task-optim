import numpy as np
import os
from datetime import datetime
from .data import *
import re

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
    usingRightHandTask = True

    try:
        rightHandExpectedDuration = np.loadtxt(root_dir_path + "/rightHandExpectedDuration.txt")
        rightHandPositionReal = np.loadtxt(root_dir_path + "/rightHandPositionReal.txt")
        rightHandPositionRef = np.loadtxt(root_dir_path + "/rightHandPositionRef.txt")
        rightHandWaypoints = np.loadtxt(root_dir_path + "/rightHandWaypoints.txt")
        rightHandJacobians = np.loadtxt(root_dir_path + "/rightHandJacobians.txt")
    except:
        usingRightHandTask = False
        print("No right hand cartesian task in this test.")

    comExpectedDuration = np.loadtxt(root_dir_path + "/comExpectedDuration.txt")
    comPositionReal = np.loadtxt(root_dir_path + "/comPositionReal.txt")
    comPositionRef = np.loadtxt(root_dir_path + "/comPositionRef.txt")
    comWaypoints = np.loadtxt(root_dir_path + "/comWaypoints.txt")
    torques = np.loadtxt(root_dir_path + "/torques.txt")
    comBounds = np.loadtxt(root_dir_path + "/comBounds.txt")
    comJacobians = np.loadtxt(root_dir_path + "/comJacobians.txt")
    jointPositions = np.loadtxt(root_dir_path + "/jointPositions.txt")
    jointLimits = np.loadtxt(root_dir_path + "/jointLimits.txt")

    comData = TaskData(timeline, comExpectedDuration, comPositionReal, comPositionRef, comWaypoints, torques, comBounds, comJacobians, jointPositions, jointLimits, "CoM")

    if usingRightHandTask:
        rightHandData = TaskData(timeline, rightHandExpectedDuration, rightHandPositionReal, rightHandPositionRef, rightHandWaypoints, torques, comBounds, rightHandJacobians, jointPositions, jointLimits, "Right_Hand")
        return [comData, rightHandData]
    else:
        return [comData]

def getDateAndTimeString():
    dt = datetime.now()
    y = dt.year
    mo = dt.month
    d = dt.day
    h = dt.hour
    m = dt.minute
    s = dt.second
    return str(y)+"-"+str(mo).zfill(2)+"-"+str(d).zfill(2)+"_"+str(h).zfill(2)+":"+str(m).zfill(2)+":"+str(s).zfill(2)

def getLastTestDir(test_name=''):
    root_dir = os.path.expanduser("~") + "/Optimization_Tests/" + test_name + '/'
    dirs = [os.path.join(root_dir,o) for o in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir,o))]
    dirs.sort()
    return dirs[-1]

def getSortedTestDirs(root_tests_dir, test_name='OneComWaypointStaticTest'):
    dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
    test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])
    return test_dirs
