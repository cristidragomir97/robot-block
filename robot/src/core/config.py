import json 
from colorama import Fore

def execute(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = ''

    # Poll process for new output until finished
    for line in iter(process.stdout.readline, ""):
        print(line)
        output += str(line)

    process.wait()
    exitCode = process.returncode

    if (exitCode == 0):
        return output
    else:
        raise Exception(command, exitCode, output)

class Interface():
    def __init__(self, obj):
        try:
            self.name = obj["name"]
            self.library = obj["library"]  
            self.address = obj["address"]  
            self.channels = obj["channels"]
        except KeyError as k:
            raise Exception("CE PULA MEA")
            formatted = json.dumps(obj, indent=4)                                
            print("{}[error] Can't load interface. Field {} is missing.\n{}{}\n  ".format(Fore.RED, k, Fore.RESET, formatted))

class Device():
    def __init__(self, obj):
        try:
            self.type = obj["type"]
            self.topic = obj["topic"]  
            self.role = obj["role"]
            self.library = obj["library"]
            self.args = obj["args"] 
        except KeyError as k:
            formatted = json.dumps(obj, indent=4)                                
            print("{}[error] Can't load device. Field {} is missing.\n{}{}\n  ".format(Fore.RED, k, Fore.RESET, formatted))

class External():
    def __init__(self, obj):
        try:
            self.name = obj["name"]
            self.role = obj["role"]
            self.source = obj["source"]
            self.build = obj["build"]
            self.command = obj["command"]
            self.package = obj["package"]
            self.file = obj["file"]
            self.args = obj["args"] 

            self.shell = ". ~/catkin_ws/devel/setup.sh &&"

        except KeyError as k:
            formatted = json.dumps(obj, indent=4)                                
            print("{}[error] Can't load external. Field {} is missing.\n{}{}\n  ".format(Fore.RED, k, Fore.RESET, formatted))
    
    def cli(self):
        shell = ". ~/catkin_ws/devel/setup.sh &&"
        shell += " " + self.command
        shell += " " + self.package
        shell += " " + self.file
        shell += " "

        for argument in self.args:
            shell += argument + ":=" + self.args[argument] + " "
        
        return shell
        #execute(shell)


class Config():

    def __init__(self, _file):
        print("FILEEEE: {}".format(_file))
      
        with open(_file) as f:
            self.contents = json.load(f)[0]
            _, self.dev, self.ext, self.interface = self.parse(self.contents)
           
    
    def parse(self, contents):
        err = []
        dev = []
        ext = []
        interface = []

        try:
                name = contents["name"]
                desc = contents["desc"]
                components = contents["components"]
                
                if "interface" in components:
                    for element in components["interface"]:
                        interface.append(Interface(element))

                if "sensors" in components:
                    for element in components["sensors"]:
                        dev.append(Device(element))

                if "actuators" in components:
                    for element in components["actuators"]:
                        dev.append(Device(element))

                if "external" in components:
                    for element in components["external"]:
                        ext.append(External(element))

        except Exception as e :
            print(Fore.YELLOW + "[warning] Can't load {} - field {} is missing ".format(self.name, k) + Fore.RESET)
            err.append(
                json.dumps(e,
                    default = lambda o: o.__dict__,
                    sort_keys = True,
                    indent = 4)
            )
        #except json.decoder.JSONDecodeError as j:
            #print(Fore.YELLOW + "[warning] JSON malformed: {}".format(j) + Fore.RESET)
            #err.append(str(j))

        return err, dev, ext, interface

    def pretty_print(self):
        print(Fore.GREEN, "DEVICES: ")
        for dev in self.dev:
            print("\t * ",dev.type, dev.library, dev.role, dev.topic, dev.args)

        print(Fore.CYAN, "EXTERNAL: ")
        for ext in self.ext:
            print("\t * ", ext.cli())
        
        print(Fore.MAGENTA, "INTERFACES: ")
        for io in self.interface:
            print("\t * ", io.name, io.library,  io.address, io.channels)

        print(Fore.RESET)

    def interfaces(self):
        return self.interface

    def devices(self):
        return self.dev

    def external(self):
        return self.ext

    def contents(self):
        return self.contents
