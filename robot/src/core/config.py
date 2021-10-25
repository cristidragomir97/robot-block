import json 
from colorama import Fore
from utils import execute

class Interface():
    def __init__(self, obj):
        try:
            self.library = obj["library"]  
            self.signal = obj["signal"]
            self.address = obj["address"]  
            self.channels = obj["channels"]
        except KeyError as k:
            formatted = json.dumps(obj, indent=4)                                
            print("{}[error] Can't load interface. Field {} is missing.\n{}{}\n  ".format(Fore.RED, k, Fore.RESET, formatted))

class Device():
    def __init__(self, obj):
        try:
            self.type = obj["type"]
            self.topic = obj["topic"]  
            self.role = obj["role"]
            self.library = obj["library"]  
            self.address = obj["address"]  
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
        self.dev = []
        self.ext = []
        self.interface = []

        with open(_file) as f:

            try:
                contents = json.load(f)[0]
                self.name = contents["name"]
                self.desc = contents["desc"]

                components = contents["components"]
                
                if "interface" in components:
                    for element in components["interface"]:
                        self.interface.append(Interface(element))

                if "sensors" in components:
                    for element in components["sensors"]:
                        self.dev.append(Device(element))

                if "actuators" in components:
                    for element in components["actuators"]:
                        self.dev.append(Device(element))

                if "external" in components:
                    for element in components["external"]:
                        self.ext.append(External(element))

            except KeyError as k:
                    print(Fore.YELLOW + "[warning] Can't load {} - field {} is missing ".format(self.name, k) + Fore.RESET)
            except json.decoder.JSONDecodeError as j:
                print(Fore.YELLOW + "[warning] JSON malformed: {}".format(j) + Fore.RESET)

    def pretty_print(self):
        print(Fore.GREEN, "DEVICES: ")
        for dev in self.dev:
            print("\t * ",dev.type, dev.library, dev.role, dev.topic, dev.address, dev.args)

        print(Fore.CYAN, "EXTERNAL: ")
        for ext in self.ext:
            print("\t * ", ext.cli())
        
        print(Fore.MAGENTA, "INTERFACES: ")
        for io in self.interface:
            print("\t * ", io.library, io.signal, io.address, io.channels)

        print(Fore.RESET)


    def interfaces(self):
        return self.interface

    def devices(self):
        return self.dev

    def external(self):
        return self.ext

    
   