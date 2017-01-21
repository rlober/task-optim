import subprocess
import time
import shlex


def killProcesses():
    proc_list = ["yarpserver", "gzserver", "gzclient", "ocra-icub-server", "reach-client", "stand-client"]
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

def simulate(controllerArgs, clientArgs, icubWorldPath, savePath=None, verbose=False, visual=False, askUserForReplay=False, runningRemotely=False):

    replay = True
    while replay:

        if verbose:
            print('Starting script...')
        if verbose:
            print('-- Launching yarpserver')
        yarp = subprocess.Popen(["yarpserver", "--write"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)

        if verbose:
            print('-- Launching gzserver with icub.world @', icubWorldPath)

        gz_args = ["gzserver"]
        if runningRemotely:
            import os
            os.environ["DISPLAY"] = ":0"

        gz_args.append(icubWorldPath)
        gzserver = subprocess.Popen(gz_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if visual:
            if verbose:
                print('-- Launching visuals with gzclient.')
            gzclient = subprocess.Popen(["gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(4)


        ctrl_args = shlex.split(controllerArgs)
        if verbose:
            print('-- Launching ocra-icub-server with args: ', ctrl_args)
        controller = subprocess.Popen(ctrl_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(5)

        if savePath != None:
            clientArgs += " --savePath " + savePath

        clArgs = shlex.split(clientArgs)
        if verbose:
            print('-- Launching client with args: ', clArgs)
        client = subprocess.Popen(clArgs, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        timeout = 40.0
        try:
            client.wait(timeout)
        except:
            client.kill()

        #### REPLAY STUFF ####

        if visual and askUserForReplay:
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
