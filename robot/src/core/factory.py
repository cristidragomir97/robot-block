import json, importlib, importlib.machinery, rospy, sys

from colorama import Fore
from core.library import Library, Package
from core.config import Config, Interface, Device, Script
from core.subscriber import Subscriber
from core.publisher import Publisher
from core.api import API
from core.utils import *

class Factory():

    def __init__(self, library, config):
        sys.stdout.register('logs/main.log')
        self.threads = {}

        self.lib = library
        self.conf = config

        self.lib.pretty_print()
        self.conf.pretty_print()

        for device in self.conf.devices():
            
            if device.role == "publisher":
                name, topic, categ, library, thread = self.make_publisher(device)
                self.threads[name] = {
                    'thread': thread, 
                    'info': {
                        'name': name,   
                        'categ': categ,
                        'topic': topic, 
                        'library': library,
                        'role': 'publisher'
                    }
                }

                
            elif device.role == "subscriber":
                name, topic, categ, library, thread = self.make_subscriber(device)
                self.threads[name] = {
                    'thread': thread,
                    'info': {
                        'name': name,   
                        'categ': categ,
                        'topic': topic, 
                        'library': library,
                        'role': 'subscriber'
                    }
                }

            elif device.role == "service":
                name, topic, categ, library, thread  = self.make_service(device)
                self.threads[name] =  {
                    'thread': thread, 
                    'info': {
                        'name': name,
                        'categ': categ, 
                        'topic': topic, 
                        'library': library,
                        'role': 'service'
                    }
                }

                

        for script in self.conf.scripts():
            self.threads[script.name] =  {
                    'thread': script, 
                    'info':{
                        'name': script.name,
                        'command': script.command, 
                        'role': 'script'
                    }
                }
    
    def reload(self):
        for thread in self.threads.values():
            if thread is not None:
                if thread.stopped() is not True:
                    thread.stop() 
        
    def make_subscriber(self, dev):
        package = self.lib.get_package(dev.library)
        name = dev.name.replace(" ", "_").lower()
        subscriber = Subscriber(name, dev.topic, package.ros_message, package.callback, package.name, package.python, dev.args)
        return name, dev.topic, dev.categ, package.name, subscriber
        

    def make_publisher(self, dev):
        package = self.lib.get_package(dev.library)
        if dev.topic is not None:
            return dev.name.replace(" ", "_").lower(), dev.topic, dev.categ, package.name, None

    def make_service(self, dev):
        package = self.lib.get_package(dev.library)
        if dev.topic is not None:
            return dev.name.replace(" ", "_").lower(), dev.topic, dev.categ, package.name, None

    def threads(self):
        return self.threads

    def stop_thread(self, id):
        print("stop thread: ",id)
    
    def start_thread(self, id):
        print("start thread: ",id)





