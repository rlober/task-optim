import subprocess
import time
import shlex

# Gazebo world file
pathToIcubGazeboWorlds = "/home/ryan/Code/iCub_Software/icub-gazebo/world"
icubWorldPath = pathToIcubGazeboWorlds + "/icub.world"

# Task set path
allTaskSets = "/home/ryan/Code/bayesian-task-optimization/reaching-task-sets/icubGazeboSim"
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


time.sleep(10)


print('-- Terminating controller')
controller.terminate()
controller.wait()
print('-- Terminating gzserver')
gazebo.terminate()
gazebo.wait()
print('-- Terminating yarpserver')
yarp.terminate()
