import json, importlib, importlib.machinery, rospy

from colorama import Fore
from library import Library, Package
from config import Config, Interface, Device, External
from subscriber import Subscriber
from publisher import Publisher
from api import API

class Factory():

    def __init__(self):
        self.threads = {}

        self.lib = Library()
        self.conf = Config('../config.json')

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


    
    
    def make_pubsub(self, dev):
        try:
            # looks into hashmap and retrieves the package object for the device name given in the config file
            package = self.lib.get_package(dev.library)

            # cretes source loader, where package.name is the class name, and package.python is the path to the source file
            loader = importlib.machinery.SourceFileLoader(package.name, package.python)
            
            # imports the python source file in the current context 
            module = loader.load_module()
            
            # basically retrieves constructor for the device 
            instance = getattr(module, package.name)
            
            # runs constructor, unpacks arguments, and initialises object
            instance = instance(*list(dev.args.values()))
            
            # gets callback method 
            callback = getattr(instance, package.callback)
            
            # imports the ROS messages 
            message = importlib.import_module(package.ros_message[0])
            message = getattr(message, package.ros_message[1])
            
            return dev.topic, message, callback

        except AttributeError as e:
            print(dev.library, " has not been found library. Error: ", e)
            return None, None, None
        except KeyError as e:
            print(Fore.RED + "[error] key {} is invalid".format(e) + Fore.RESET)
            return None, None, None
        
        
    def make_subscriber(self, dev):
        topic, msg, cb =  self.make_pubsub(dev)

        if topic is not None:
            return Subscriber(topic, msg, cb)
        else:
            print("skipping device")

            
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

        

if __name__ == "__main__":
    rospy.init_node("bot", anonymous=False, disable_signals=True)
    factory = Factory()
    api = API(factory)
    api.start()


    for thread in factory.threads.values(): 
        if thread is not None:
            thread.start()



    
