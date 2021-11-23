import json, threading, sys

class Script(threading.Thread):
    def __init__(self, obj):
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        self.isRunning = False
       
        try:   
            self.name = obj["name"].replace(" ", "_")
            self.command = obj["command"]
            self.args = obj["args"] 

            sys.stdout.register('logs/{}.log'.format(self.name.replace("/", "_").lower()))

            self.shell = ". ~/catkin_ws/devel/setup.sh &&"
            self.shell += " " + self.command
            self.shell += " "

            for argument in self.args:
                self.shell += argument + ":=" + self.args[argument] + " "

        except KeyError as k:
            formatted = json.dumps(obj, indent=4)                                
            print("{}[error] Can't load external. Field {} is missing.\n{}{}\n  ".format(Fore.RED, k, Fore.RESET, formatted))
    
        
    
    def run(self):
        self.isRunning = True

        process = subprocess.Popen(self.shell , shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        # Poll process for new output until finished
        for line in iter(process.stdout.readline, ""):
            print(line)

        process.wait()
        exitCode = process.returncode

        if (exitCode == 0):
            pass
        else:
            raise Exception(command, exitCode, output)
  
    # function using _stop function
    def stop(self):
        self._stop.set()
 
    def stopped(self):
        return self._stop.isSet()