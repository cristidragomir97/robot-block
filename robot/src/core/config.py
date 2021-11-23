import json, sys
from colorama import Fore
from core.script import Script


class Interface():
    def __init__(self, obj):
        try:
            self.name = obj["name"]
            self.library = obj["library"]  
            self.address = obj["address"]  
            self.channels = obj["channels"]
        except KeyError as k:
            formatted = json.dumps(obj, indent=4)                                
            print("{}[error] Can't load interface. Field {} is missing.\n{}{}\n  ".format(Fore.RED, k, Fore.RESET, formatted))

class Device():
    def __init__(self, obj, categ):
        try:
            self.name = obj["name"]
            self.type = obj["type"]
            self.topic = obj["topic"]  
            self.role = obj["role"]
            self.library = obj["library"]
            self.args = obj["args"] 
            self.categ = categ
        except KeyError as k:
            formatted = json.dumps(obj, indent=4)                                
            print("{}[error] Can't load device. Field {} is missing.\n{}{}\n  ".format(Fore.RED, k, Fore.RESET, formatted))



class Container():
    def __init__(self, obj):
        pass


class Config():

    def __init__(self, _file):
        sys.stdout.register('logs/main.log') 
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
                        dev.append(Device(element, categ="sensors"))

                if "actuators" in components:
                    for element in components["actuators"]:
                        dev.append(Device(element, categ="actuators"))

                if "scripts" in components:
                    for element in components["scripts"]:
                        ext.append(Script(element))

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
        print("- DEVICES: ")
        for dev in self.dev:
            print(" --> ",dev.type, dev.library, dev.role, dev.topic, dev.args)

        print("- EXTERNAL: ")
        for ext in self.ext:
            print(" --> ", ext.name)
        
        print("- INTERFACES: ")
        for io in self.interface:
            print(" --> ", io.name, io.library,  io.address, io.channels)



    def interfaces(self):
        return self.interface

    def devices(self):
        return self.dev

    def scripts(self):
        return self.ext

    def contents(self):
        return self.contents
