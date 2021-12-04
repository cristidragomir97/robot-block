# import rospy
import threading, rospy


'''
class Publisher(threading.Thread):
    
    def __init__(self, name, topic, message, package_callback, package_name, package_source,  arguments, queue_size=100, rate=24):
        print('name:', package_name, 'source:', package_source,  'args:', arguments)
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        self.isRunning = False

        self.callback, message = self._instance(
            topic=topic, 
            msg=message,
            package_callback=package_callback,
            package_name=package_name,
            package_source=package_source,
            arguments=arguments
        )

        self.pub = rospy.Publisher(topic, message, queue_size=queue_size)
  
    def _instance(self, topic, msg, package_callback, package_name, package_source, arguments):
        # cretes source loader, where package.name is the class name, and package.python is the path to the source file
        loader = importlib.machinery.SourceFileLoader(package_name, package_source)
        print(package_name, package_source)
        # imports the python source file in the current context 
        module = loader.load_module()
                
        # basically retrieves constructor for the device 
        instance = getattr(module, package_name)
                
        # runs constructor, unpacks arguments, and initialises object
        instance = instance(*list(arguments.values()))
                
        # gets callback method 
        callback = getattr(instance, package_callback)
                
        # imports the ROS messages 

        message = importlib.import_module(msg[0])
        message = getattr(message, msg[1])

        return callback, message
        
    # function using _stop function
    def stop(self):
        self._stop.set()
 
    def stopped(self):
        return self._stop.isSet()
  
    def run(self):  
        self.isRunning = False
        while True:
            ret = self.callback()
            self.pub.publish(ret)

'''
import rospy
import trace
import threading, sys

class Publisher(threading.Thread):
    def __init__(self, topic, message, callback, queue_size=100, rate=60):
        threading.Thread.__init__(self)
        self.killed = False

        pub = rospy.Publisher(topic, message, queue_size=queue_size)

        self.pub = pub
        self.callback = callback
      
    def run(self):  
        self.isRunning = False
        while True:
            ret = self.callback()
            self.pub.publish(ret)

