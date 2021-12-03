import json, importlib, importlib.machinery, rospy, sys

from core.library import Library, Package
from core.config import Config, Device, Script
from core.subscriber import Subscriber
from core.publisher import Publisher
from core.api import API
from core.utils import *

class Factory():
    def __init__(self, library, config):
        #sys.stdout.register('logs/main.log')
        self.threads = {}

        self.lib = library
        self.conf = config

        self.lib.pretty_print()
        self.conf.pretty_print()

        self._devices()
        self._scripts()

        print(self.threads)


    def _devices(self):
        for device in self.conf.devices():
            if device.role == "publisher":
                name, topic, categ, library, thread = self.make_publisher(device)
                print(device.name)
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

    def _scripts(self):
        for script in self.conf.scripts():
            self.threads[script.name] =  {
                    'thread': script.start(), 
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
        #print(package.name, package.callback, 
      
        #subscriber = Subscriber(name, dev.topic, package.ros_message, package.callback, package.name, package.python, dev.args)
        return name, dev.topic, dev.categ, package.name, None
        
    def make_publisher(self, dev):
        package = self.lib.get_package(dev.library)
        name = dev.name.replace(" ", "_").lower()
        publisher = Publisher(name, dev.topic, package.ros_message, package.callback, package.name, package.python, dev.args)
        return name, dev.topic, dev.categ, package.name, publisher.start()

    def make_service(self, dev):
        package = self.lib.get_package(dev.library)
        if dev.topic is not None:
            return dev.name.replace(" ", "_").lower(), dev.topic, dev.categ, package.name, None

    def threads(self):
        return self.threads

    def info_thread(self, id):
        thread = self.threads[id]['thread']
        return {'thread': str(id), 'status': [thread.isRunning, thread.stopped()]}

    def stop_thread(self, id):
        thread = self.threads[id]['thread']
 
        return {'thread': str(id), 'action': 'stopping', 'status': [thread.isRunning, thread.stopped()]}
    
    def start_thread(self, id):
        thread = self.threads[id]['thread']
        thread.start()
        return {'thread': str(id), 'action': 'stopping', 'status': [thread.isRunning, thread.stopped()]}
