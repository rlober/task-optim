import subprocess
import time
import shlex
import os

rootPath = os.path.expanduser("~")

def killProcesses():
    proc_list = ["yarpserver", "gzserver", "gzclient", "ocra-icub-server", "reach-client"]
    total_killed = 0
    for p in proc_list:
        args = ["pkill", "-c", "-9", p]
        output,error = subprocess.Popen(args,stdout = subprocess.PIPE, stderr= subprocess.PIPE).communicate()
        total_killed += int(output)
    if total_killed > 0:
        print("Killed " + str(total_killed) +" processes")
    else:
        print("No processes were killed.")

    return total_killed

def simulate(pathToRightHandWptFile, pathToComWptFile, savePath=None, verbose=False, visual=False):


    replay = True
    while replay:
        # Gazebo world file
        pathToIcubGazeboWorlds = rootPath + "/icub-gazebo/world"
        icubWorldPath = pathToIcubGazeboWorlds + "/icub.world"

        # Task set path
        allTaskSets = rootPath + "/bayesian-task-optimization/reaching-task-sets/icubGazeboSim"
        taskSetPath = allTaskSets + "/TaskOptimizationTaskSet.xml"


        if verbose:
            print('Starting script...')
        if verbose:
            print('-- Launching yarpserver')
        yarp = subprocess.Popen(["yarpserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)

        if verbose:
            print('-- Launching gzserver with icub.world @', icubWorldPath)
        gzserver = subprocess.Popen(["gzserver", icubWorldPath], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if visual:
            if verbose:
                print('-- Launching visuals with gzclient.')
            gzclient = subprocess.Popen(["gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(4)


        args1 = "ocra-icub-server --floatingBase --controllerType HOCRA --solver QPOASES --taskSet " + taskSetPath + " --absolutePath"
        args = shlex.split(args1)
        if verbose:
            print('-- Launching ocra-icub-server with args: ', args)
        controller = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(5)

        if savePath != None:
            save_args = " --savePath " + savePath
        else:
            save_args = ""

        args1 = "reach-client --rightHandWptFile "+pathToRightHandWptFile+" --comWptFile " + pathToComWptFile + save_args
        args = shlex.split(args1)
        if verbose:
            print('-- Launching reach-client with args: ', args)
        client = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        timeout = 40.0
        try:
            client.wait(timeout)
        except:
            client.kill()

        #### REPLAY STUFF ####

        if visual:
            user_input = input("Replay simulation? [y/n] (y): ")
            if user_input == "" or user_input == "y" or user_input == "Y":
                replay = True
            else:
                replay = False
        else:
            replay = False

        ######################

        timeout = 20.0
        if verbose:
            print('-- Terminating controller')
        controller.terminate()
        try:
            controller.wait(timeout)
        except:
            controller.kill()
        if verbose:
            print('-- Terminating gzserver')
        gzserver.terminate()
        if visual:
            gzclient.terminate()

        try:
            gzserver.wait(timeout)
        except:
            gzserver.kill()

        if visual:
            try:
                gzclient.wait(timeout)
            except:
                gzclient.kill()

        if verbose:
            print('-- Cleaning up ports')
        args1 = "yarp clean"
        args = shlex.split(args1)
        cleanYarp = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            cleanYarp.wait(timeout)
        except:
            cleanYarp.kill()
        if verbose:
            print('-- Terminating yarpserver')
        yarp.terminate()
