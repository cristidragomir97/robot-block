import json, importlib, importlib.machinery, rospy

from colorama import Fore
from core.library import Library, Package
from core.config import Config, Interface, Device, External
from core.subscriber import Subscriber
from core.publisher import Publisher
from core.api import API
from core.utils import *

class Factory():

    def __init__(self, library, config):
        self.threads = {}

        self.lib = library
        self.conf = config

        self.lib.pretty_print()
        self.conf.pretty_print()

        for device in self.conf.devices():
            if device.role == "publisher":
                thread = self.make_publisher(device)
                self.threads[device.topic] = thread
                
            elif device.role == "subscriber":
                thread = self.make_subscriber(device)
                self.threads[device.topic] = thread

            elif device.role == "service":
                thread = self.make_service(device)
                self.threads[device.topic] = thread

            elif device.role == "external":
                thread = self.make_external(device)
                self.threads[device.topic] = thread
    
    
    def reload(self):

        for thread in self.threads.values():
            if thread is not None:
                if thread.stopped() is not True:
                    print("stopping current thread")
                    thread.stop()
              
        
    def make_subscriber(self, dev):
        package = self.lib.get_package(dev.library)

        if dev.topic is not None:
            return Subscriber(dev.topic, package.ros_message, package.callback, package.name, package.python, dev.args)
        

    def make_publisher(self, dev):
        pass
        #return Publisher(self.make_pubsub(dev)).run()

    def make_service(self, device):
        pass

    def make_external(self, device): 
        print(device.cli())
        pass

    def threads(self):
        return self.threads

    def stop_thread(self, id):
        print("stop thread: ",id)
    
    def start_thread(self, id):
        print("start thread: ",id)

    def info_thread(self, id):
        print("info thread: ",id)

        


