import re
from task_optim.sim_tools.simulate import *
from task_optim.utils.files import *
import os
import subprocess
import time
import shlex

def combineVideos(test_dir):
    ffmpeg_args = """ffmpeg -y -i """ + test_dir + """/original.mp4 -i """ + test_dir + """/optimal.mp4 -filter_complex "[0:v][1:v]hstack=inputs=2[v]" -map "[v]" -ac 2 """+test_dir+"""/temp_0.mp4"""

    ffmpeg_args = shlex.split(ffmpeg_args)
    proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    proc_1.wait()

    ffmpeg_args = """ffmpeg -i """ + test_dir + """/temp_0.mp4 -y -vf drawtext="text='ORIGINAL':\
    fontcolor=white:\
    fontsize=48:\
    box=1:\
    boxcolor=black@0.8:\
    boxborderw=10:\
    x=(main_w)/4:\
    y=main_h-(text_h*2)" -codec:a copy """ + test_dir + """/temp_1.mp4"""

    ffmpeg_args = shlex.split(ffmpeg_args)
    proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    proc_1.wait()

    ffmpeg_args = """ffmpeg -i """ + test_dir + """/temp_1.mp4 -y -vf drawtext="text='OPTIMAL':\
    fontcolor=white:\
    fontsize=48:\
    box=1:\
    boxcolor=black@0.8:\
    boxborderw=10:\
    x=(main_w)*3/4:\
    y=main_h-(text_h*2)" -codec:a copy """ + test_dir + """/optimal_and_original.mp4"""

    ffmpeg_args = shlex.split(ffmpeg_args)
    proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    proc_1.wait()

    ffmpeg_args = """rm """+test_dir+"""/temp_0.mp4 """+test_dir+"""/temp_1.mp4"""

    ffmpeg_args = shlex.split(ffmpeg_args)
    proc_1 = subprocess.Popen(ffmpeg_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    proc_1.wait()

    print("View video at:\n", test_dir + "/optimal_and_original.mp4")

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

    max_trials = 20

    number_of_trials = 0
    while number_of_trials <= max_trials:
        try:
            simulate(controller_args, client_args_original, gazebo_args, verbose=False, visual=False, askUserForReplay=False, runningRemotely=True)
            break
        except:
            print("Simulation failed. Rerunning. Current number of trials rerun: "+str(number_of_trials))
            killProcesses()
            time.sleep(2.0)
            killProcesses()
            number_of_trials += 1

    if number_of_trials > max_trials:
        print("Couldn't get the original simulation to work after "+str(max_trials)+". I am giving up.")

    number_of_trials = 0
    while number_of_trials <= max_trials:
        try:
            simulate(controller_args, client_args_optimal, gazebo_args, verbose=False, visual=False, askUserForReplay=False, runningRemotely=True)
            break
        except:
            print("Simulation failed. Rerunning. Current number of trials rerun: "+str(number_of_trials))
            killProcesses()
            time.sleep(2.0)
            killProcesses()
            number_of_trials += 1

    if number_of_trials > max_trials:
        print("Couldn't get the optimal simulation to work after "+str(max_trials)+". I am giving up.")



root_tests_dir = os.path.expanduser("~") + "/Optimization_Tests/rand_right_hand_target_tests/bo/"
test_name = 'OneComWaypointStaticTest'
dirs = [d for d in os.listdir(root_tests_dir) if os.path.isdir(os.path.join(root_tests_dir, d))]
test_dirs = sorted([os.path.join(root_tests_dir, d) for d in dirs if re.match(test_name+'.*', d)])

for i, t in enumerate(test_dirs):
    print('Recording simulation for test', i+1, 'of', len(test_dirs), '.', (i+1)/len(test_dirs)*100, "% complete.")
    try:
        replayOriginalAndOptimalReachingSimulation(t)
        combineVideos(t)
    except:
        print("Couldn't record simulation from:\n", t)
