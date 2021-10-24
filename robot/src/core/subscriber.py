import rospy
import threading

class Subscriber(threading.Thread):
    def __init__(self,  topic, message, callback):
        threading.Thread.__init__(self)
      
        rospy.Subscriber(topic, message, callback)

    def run(self):
        rospy.spin()

