import os
import sys
import yarp
import shlex
import subprocess

icub_data_ports = ["/icub/torso/stateExt:o", "/icub/left_leg/stateExt:o", "/icub/right_leg/stateExt:o", "/icub/left_arm/stateExt:o", "/icub/right_arm/stateExt:o", "/wholeBodyDynamics/left_arm/endEffectorWrench:o", "/wholeBodyDynamics/right_arm/endEffectorWrench:o", "/amti/first/analog:o", "/amti/second/analog:o", "/xsens/frames:o", "/human-state-provider/state:o", "/human-forces-provider/forces:o", "/human-dynamics-estimator/dynamicsEstimation:o"]

def convertPortNamesToDirNames(port_names):
    dir_names = []
    for n in icub_data_ports:
        tmp = n[1:-2]
        dir_names.append(tmp.replace("/", "_"))

    return dir_names

def getRelativeIterationDirPath(iter_dir):
    return os.path.relpath(iter_dir)

def makeDataSaveDir(iter_dir, port_dir_name):
    save_dir_path = os.path.join(iter_dir, port_dir_name)
    try:
        os.makedirs(save_dir_path)
    except(FileExistsError):
        print("Not making dir for:", port_dir_name, " --> Directory already exists.")
    return getRelativeIterationDirPath(save_dir_path)

def launchYarpDataDumper(port_name, port_dir_name, port_dir_rel_path):
    cmd =  "yarpdatadumper"
    cmd += " --name /"+port_dir_name
    cmd += " --connect "+port_name
    cmd += " --dir "+port_dir_rel_path
    args = shlex.split(cmd)

    yarp_dd_proc = subprocess.Popen(clArgs, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class DataDumper(object):
    """docstring for DataDumper."""
    def __init__(self, iter_dir, port_name):
        self.loggingStarted = False
        self.called_from_destructor = False
        self.port_name = port_name
        self.port_exists = yarp.Network.exists(self.port_name)
        if self.port_exists:
            self.iter_dir = iter_dir
            self.port_dir_name = port_name[1:-2].replace("/", "_")
            self.makeDataSaveDir()
        else:
            print("Port", port_name, "doesn't exist! Doing nothing.")

    def __del__(self):
        self.called_from_destructor = True
        self.stopLogging()

    def makeDataSaveDir(self):
        self.port_dir_abs_path = os.path.join(self.iter_dir, self.port_dir_name)
        try:
            os.makedirs(self.port_dir_abs_path)
        except(FileExistsError):
            print("Not making dir for:", self.port_dir_name, " --> Directory already exists.")

        self.port_dir_rel_path = os.path.relpath(self.port_dir_abs_path)

    def startLogging(self):
        if self.port_exists:
            if not self.loggingStarted:
                cmd =  "yarpdatadumper"
                cmd += " --name /" + self.port_dir_name
                cmd += " --connect " + self.port_name
                cmd += " --dir " + self.port_dir_rel_path + "/"
                args = shlex.split(cmd)

                self.loggingStarted = True
                self.yarp_dd_proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print("------------")
                print("Logging started on port:", self.port_name)
                print("------------")
            else:
                print("Logging already started on port:", self.port_name, "Stop before restarting.")
        else:
            print("Port:", self.port_name, "doesn't exist so I can't log!")

    def stopLogging(self):
        if self.loggingStarted:
            self.yarp_dd_proc.terminate()
            timeout = 3.0
            try:
                self.yarp_dd_proc.wait(timeout)
                self.loggingStarted = False
            except:
                self.yarp_dd_proc.kill()
                self.loggingStarted = False
            print("------------")
            print("Logging finished on port:", self.port_name, "\nData stored at:", self.port_dir_abs_path)
            print("------------")
        else:
            if not self.called_from_destructor:
                print("Logging has not been started on port:", self.port_name, "yet. Doing nothing.")

if __name__ == "__main__":
    # port_dir_names = convertPortNamesToDirNames(icub_data_ports)
    # iter_dir = "/home/ryan/fake_test/"
    # print(getRelativeIterationDirPath(iter_dir))
    # print(makeDataSaveDir(iter_dir, port_dir_names[0]))
    import time
    iter_dir = os.path.expanduser("~") + "/fake_test/"
    d = DataDumper(iter_dir, icub_data_ports[0])
    d2 = DataDumper(iter_dir, "/blawdawhd")
    d.startLogging()
    d2.startLogging()

    time.sleep(3.0)

    d2.stopLogging()
    # d.stopLogging()
