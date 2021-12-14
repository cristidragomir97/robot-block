import json, threading, sys, subprocess, signal, os 
from subprocess import Popen, PIPE, STDOUT, CalledProcessError
from core.utils import *


def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
        print(parent)
    except psutil.NoSuchProcess:
        print("parent process not existing")
        return
    children = parent.children(recursive=True)
    print(children)
    for process in children:
        print("try to kill child: " + str(process))
        process.send_signal(sig)

class Script(threading.Thread):
    def __init__(self, obj):

        threading.Thread.__init__(self)
   
        try:   
            self.name = obj["name"].replace(" ", "_")
            self.command = obj["command"]
            self.args = obj["args"] 


            self.shell = ". ~/catkin_ws/devel/setup.sh &&"
            self.shell += " " + self.command
            self.shell += " "

            for argument in self.args:
                self.shell += argument + ":=" + self.args[argument] + " "

        except KeyError as k:
            formatted = json.dumps(obj, indent=4)                                
            logg(__name__, "ERROR", "Can't load script. Field {} is missing.\n{}\n  ".format(k, formatted))

    def run(self):
        try:
            self.process = Popen(self.shell, shell=True, preexec_fn=os.setsid,  stdout=PIPE, stderr=STDOUT)
            self.pid = self.process.pid  # pid of the roscore process (which has child processes)

            with self.process.stdout:
                try:
                    for line in iter(self.process.stdout.readline, b''):
                        logg(__name__, "INFO", line.decode("utf-8").strip())
                        
                except CalledProcessError as e:
                        logg(__name__, "ERROR", "error launching process {self.name}: {e}")
                    
        except OSError as e:
            logg(__name__, "ERROR","script could not be run {e}")

        
    def terminate(self):
        logg(__name__, "INFO", "trying to kill child pids of script pid: " + str(self.pid))
        kill_child_processes(self.pid)
        self.process.terminate()
        self.process.wait()  # important to prevent from zombie process

