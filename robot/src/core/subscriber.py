#import rospy, rospy
import threading, rospy, sys, importlib, importlib.machinery


class Subscriber(threading.Thread):
    def __init__(self,  name, topic, message, package_callback, package_name, package_source,  arguments):
        threading.Thread.__init__(self)
        self._stop = threading.Event()
        self.isRunning = False

        callback, message = self._instance(
            topic=topic, 
            msg=message,
            package_callback=package_callback,
            package_name=package_name,
            package_source=package_source,
            arguments=arguments
        )

        rospy.Subscriber(topic, message, callback)
        

    def _instance(self, topic, msg, package_callback, package_name, package_source, arguments):
        # cretes source loader, where package.name is the class name, and package.python is the path to the source file
        loader = importlib.machinery.SourceFileLoader(package_name, package_source)
                
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


    def run(self):
        self.isRunning = True
        rospy.spin()
  
    # function using _stop function
    def stop(self):
        self._stop.set()
 
    def stopped(self):
        return self._stop.isSet()


