
from task_optim.sim_tools.simulate import *
from task_optim.utils.files import *
import os

def replayOriginalAndOptimalReachingSimulation(test_dir):

    pathToRightHandWptFile = test_dir + "/Optimal_Solution/rightHandWaypoints_optimal.txt"
    pathToComWptFileOptimal = test_dir + "/Optimal_Solution/comWaypoints_optimal.txt"
    pathToComWptFileOriginal = test_dir + "/Iteration_000/comWaypoints.txt"

    code_dir = os.path.expanduser("~") + "/Code/bayesian-task-optimization"

    taskSetPath = code_dir + "/reaching-task-sets/icubGazeboSim/TaskOptimizationTaskSet.xml"
    controller_args = "ocra-icub-server --floatingBase --taskSet " + taskSetPath + " --absolutePath"
    gazebo_args = code_dir + "/gazebo_worlds/balancing_camera.world"

    client_args_original = "reach-client --rightHandWptFile "+ pathToRightHandWptFile +" --comWptFile " + pathToComWptFileOriginal + " --home --record --recordDir " + test_dir + " --recordName original"

    client_args_optimal = "reach-client --rightHandWptFile "+ pathToRightHandWptFile +" --comWptFile " + pathToComWptFileOptimal + " --home --record --recordDir " + test_dir + " --recordName optimal"


    simulate(controller_args, client_args_original, gazebo_args, verbose=True, visual=False, askUserForReplay=False)

    simulate(controller_args, client_args_optimal, gazebo_args, verbose=True, visual=False, askUserForReplay=False)


root_tests_dir = os.path.expanduser("~") + "/Optimization_Tests/rand_right_hand_target_tests/bo/"
test_name = 'OneComWaypointStaticTest'
dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])

print(test_dirs, '\n\n\n')
for i, t in enumerate(test_dirs[0:1]):
    print('Recording simulation for test', i+1, 'of', len(test_dirs), '.', (i+1)/len(test_dirs)*100, "% complete.")
    replayOriginalAndOptimalReachingSimulation(t)

    # try:
    #     replayOriginalAndOptimalReachingSimulation(t)
    # except:
    #     print("Couldn't record simulation from:\n", t)
