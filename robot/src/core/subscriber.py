#import rospy, rospy
import threading, rospy

class Subscriber(threading.Thread):
    def __init__(self,  topic, message, callback):
  
        threading.Thread.__init__(self)
        rospy.Subscriber(topic, message, callback)
        self._stop = threading.Event()

    def run(self):
        rospy.spin()
  
    # function using _stop function
    def stop(self):
        self._stop.set()
 
    def stopped(self):
        return self._stop.isSet()


