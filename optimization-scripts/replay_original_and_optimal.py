import re
from task_optim.sim_tools.simulate import *
from task_optim.utils.files import *
import os
import subprocess
import time
import shlex


def replayOriginalAndOptimalReachingSimulation(test_dir):

    pathToRightHandWptFile = test_dir + "/Optimal_Solution/rightHandWaypoints_optimal.txt"
    pathToComWptFileOptimal = test_dir + "/Optimal_Solution/comWaypoints_optimal.txt"
    pathToComWptFileOriginal = test_dir + "/Iteration_000/comWaypoints.txt"

    code_dir = os.path.expanduser("~") + "/Code/bayesian-task-optimization"

    taskSetPath = code_dir + "/reaching-task-sets/icubGazeboSim/TaskOptimizationTaskSet.xml"
    controller_args = "ocra-icub-server --floatingBase --taskSet " + taskSetPath + " --absolutePath"
    gazebo_args = code_dir + "/gazebo_worlds/balancing_camera.world"

    client_args_original = "reach-client --rightHandWptFile "+ pathToRightHandWptFile +" --comWptFile " + pathToComWptFileOriginal + " --home --record --recordDir " + test_dir + "/ --recordName original"

    client_args_optimal = "reach-client --rightHandWptFile "+ pathToRightHandWptFile +" --comWptFile " + pathToComWptFileOptimal + " --home --record --recordDir " + test_dir + "/ --recordName optimal"

    original_failed = []
    optimal_failed = []
    max_trials = 5
    if not os.path.exists( os.path.join(test_dir, "original.mp4") ):
        number_of_trials = 0
        while number_of_trials <= max_trials:
            simulate(controller_args, client_args_original, gazebo_args, verbose=False, visual=False, askUserForReplay=False, runningRemotely=True)

            if os.path.exists( os.path.join(test_dir, "original.mp4") ):
                break
            else:
                print("Simulation failed. Rerunning. Current number of trials rerun: "+str(number_of_trials))
                killProcesses()
                time.sleep(2.0)
                killProcesses()
                number_of_trials += 1

        if number_of_trials > max_trials:
            print("Couldn't get the original simulation to work after "+str(max_trials)+". I am giving up.")
            original_failed.append(test_dir)
    else:
        print("Video already exists:", os.path.join(test_dir, "original.mp4"))

    if not os.path.exists( os.path.join(test_dir, "optimal.mp4") ):
        number_of_trials = 0
        while number_of_trials <= max_trials:
            simulate(controller_args, client_args_optimal, gazebo_args, verbose=False, visual=False, askUserForReplay=False, runningRemotely=True)

            if os.path.exists( os.path.join(test_dir, "optimal.mp4") ):
                break
            else:
                print("Simulation failed. Rerunning. Current number of trials rerun: "+str(number_of_trials))
                killProcesses()
                time.sleep(2.0)
                killProcesses()
                number_of_trials += 1

        if number_of_trials > max_trials:
            print("Couldn't get the optimal simulation to work after "+str(max_trials)+". I am giving up.")
            optimal_failed.append(test_dir)
    else:
        print("Video already exists:", os.path.join(test_dir, "optimal.mp4"))

    return original_failed, optimal_failed

root_tests_dir = os.path.expanduser("~") + "/Optimization_Tests/rand_right_hand_target_tests/bo/"
test_name = 'OneComWaypointStaticTest'
dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])

for i, t in enumerate(test_dirs):
    print('Recording simulation for test', i+1, 'of', len(test_dirs), '.', (i+1)/len(test_dirs)*100, "% complete.")
    original_failed, optimal_failed = replayOriginalAndOptimalReachingSimulation(t)

print("\n\noriginal_failed\n", original_failed)
print("\n\noptimal_failed\n", optimal_failed)
