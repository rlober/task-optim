import subprocess
import time
import shlex

rootPath = "/home/ryan"

def executeWaypoints(pathToRightHandWptFile, pathToComWptFile, savePath):

    # Gazebo world file
    pathToIcubGazeboWorlds = rootPath + "/icub-gazebo/world"
    icubWorldPath = pathToIcubGazeboWorlds + "/icub.world"

    # Task set path
    allTaskSets = rootPath + "/bayesian-task-optimization/reaching-task-sets/icubGazeboSim"
    taskSetPath = allTaskSets + "/TaskOptimizationTaskSet.xml"


    print('Starting script...')
    print('-- Launching yarpserver')
    yarp = subprocess.Popen(["yarpserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)

    print('-- Launching gzserver with icub.world @', icubWorldPath)
    gazebo = subprocess.Popen(["gzserver", icubWorldPath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)


    args1 = "ocra-icub-server --floatingBase --controllerType HOCRA --solver QPOASES --taskSet " + taskSetPath + " --absolutePath"
    args = shlex.split(args1)
    print('-- Launching ocra-icub-server with args: ', args)
    controller = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    time.sleep(5)

    args1 = "reach-client --rightHandWptFile "+pathToRightHandWptFile+" --comWptFile " + pathToComWptFile + " --savePath " + savePath
    args = shlex.split(args1)
    print('-- Launching reach-client with args: ', args)
    client = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    client.wait()


    print('-- Terminating controller')
    controller.terminate()
    controller.wait()
    print('-- Terminating gzserver')
    gazebo.terminate()
    gazebo.wait()
    print('-- Terminating yarpserver')
    yarp.terminate()



savePath = "/home/ryan/bayesian-task-optimization/tmp/"
pathToRightHandWptFile = "/home/ryan/bayesian-task-optimization/rightHandWaypoints.txt"
pathToComWptFile = "/home/ryan/bayesian-task-optimization/comWaypoints.txt"

executeWaypoints(pathToRightHandWptFile, pathToComWptFile, savePath)
