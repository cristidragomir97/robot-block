import json, sys
from core.script import Script
from core.utils import *


class Channel():
    def __init__(self, name, role, pin, topic, args):
        self.name = name
        self.role = role
        self.pin = pin
        self.topic = topic
        self.args = args
    
    
class Device():
    def __init__(self, obj):

        self.channels = []
        try:
            self.name = obj["name"]
            if ("channel_no" in obj):
                self.channel_no = int(obj["channel_no"])

                for item in range(0, int(obj["channel_no"]) ):
                    _name = obj["channels"][item]["name"]
                    _role = obj["channels"][item]["role"]
                    _pin = obj["channels"][item]["channel"]
                    _topic = obj["channels"][item]["topic"]
                    _args = obj["channels"][item]["args"]

                    self.channels.append(Channel(_name, _role, _pin, _topic, _args))

            else:
                self.channel_no = 0
                self.role = obj["role"]
                self.topic = obj["topic"]  

            self.library = obj["library"]
            self.args = obj["args"] 

        except KeyError as k:
            formatted = json.dumps(obj, indent=4)   
            logg(__name__, "ERROR", "Can't load device. Field {} is missing.\n{}\n  ".format(k, formatted))


    def get_channel(self, n):
        return self.channels[n]


class Config():

    def __init__(self,  _file):
        with open(_file) as f:
            self.contents = json.load(f)[0]

            logg(__name__, "INFO", "loading configuration file: {}".format(_file))

            _, self.dev, self.ext, = self.parse(self.contents)
            
           
    
    def parse(self, contents):
        err = []
        dev = []
        scripts = []

        try:
                name = contents["name"]
                desc = contents["desc"]
                components = contents["components"]
                
                if "input" in components:
                    for element in components["input"]:
                        dev.append(Device(element))

                if "output" in components:
                    for element in components["output"]:
                        dev.append(Device(element))

                if "scripts" in components:
                    for element in components["scripts"]:
                        scripts.append(Script(element))

        except Exception as e :
            logg(__name__, "WARNING","Can't load configuration item {} - field {} is missing ".format(name ,e))

            err.append(
                json.dumps(e,
                    default = lambda o: o.__dict__,
                    sort_keys = True,
                    indent = 4)
            )

        return err, dev, scripts 

    def pretty_print(self):

        for dev in self.dev:
            
            if dev.channel_no > 0:
                logg(__name__, "INFO", "[DEVICE]: {}, {}, {}".format(dev.name, dev.library, dev.args))

                for channel in dev.channels:
                    logg(__name__, "INFO", "[CHANNEL{}][{}, {}, {}".format(channel.pin, channel.name, channel.role, channel.topic))
            else:
                logg(__name__, "INFO", "[DEVICE]: {}, {}, {}, {}, {}".format(dev.name, dev.library, dev.role, dev.topic, dev.args))

        for ext in self.ext: logg(__name__, "INFO", "[SCRIPT] {}".format(ext.name))

    def get_devices(self):
        return self.dev

    def get_scripts(self):
        return self.ext