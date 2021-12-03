import json, sys
from core.script import Script

'''
      "name": "pwm_driver", 
                "library": "PCA9648",
                "channel_no": "4",
                "channels":[
                    {   
                        "name": "camera_tilt_servo", 
                        "role": "service",
                        "pin": "0",
                        "topic": "/servo/tilt", 
                        "args":{}
                    }
                ], 
                "args":{
                    "address": "0x29"
                }
            }, 
            {   
                "name": "motor_driver", 
                "role": "subscriber",
                "topic": "/cmd_vel",
                "library": "SCMD",
                "address": "0x5d", 
                "args":{
                    "radius": 0.0325,
                    "flip": "true"
                }
            }
'''

class Channel():
    def __init__(self, name, role, pin, topic, args):
        self.name = name
        self.role = role
        self.pin = pin
        self.topic = topic
        self.args = args

        print("Channel:", self.name, self.role, self.pin, self.topic, self.args)
    

class Device():
    def __init__(self, name, obj):

        self.channels = []
        try:
            self.name = obj["name"]
            if ("channel_no" in obj):
                self.channel_no = int(obj["channel_no"])

                for item in range(0, int(obj["channel_no"]) - 1):
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
            print("[error] Can't load device. Field {} is missing.\n{}\n  ".format(k, formatted))



class Config():

    def __init__(self,  _file):
        sys.stdout.register('logs/main.log') 
        with open(_file) as f:
            self.contents = json.load(f)[0]
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
            print("[warning] Can't load - field {} is missing ".format(k))
            err.append(
                json.dumps(e,
                    default = lambda o: o.__dict__,
                    sort_keys = True,
                    indent = 4)
            )

        return err, dev, scripts 

    def pretty_print(self):
        print("- DEVICES: ")
        
        for dev in self.dev:
            
            if dev.channel_no > 0:
                print(" * ", dev.name, dev.library, dev.args)
                for channel in dev.channels:
                    print("\t - Channel[{}]: {}, {}, {}, {}".format(channel.pin, channel.name, channel.role, channel.topic, channel.args))
            else:
                 print(" * " ,dev.name, dev.library, dev.role, dev.topic, dev.args)


        print("- SCRIPTS: ")
        for ext in self.ext:
            print(" --> ", ext.name)
    



    def interfaces(self):
        return self.interface

    def devices(self):
        return self.dev

    def scripts(self):
        return self.ext

    def contents(self):
        return self.contents
