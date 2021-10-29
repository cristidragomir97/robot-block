#import rospy, rospy
import threading, rospy

class Subscriber(threading.Thread):
    def __init__(self,  topic, message, callback):
  
        threading.Thread.__init__(self)
        rospy.Subscriber(topic, message, callback)

    def run(self):
        rospy.spin()
  
    
    def get_id(self):
        # returns id of the respective thread
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id
