import rospy
import threading 

class Publisher(threading.Thread):
    def __init__(self, topic, message, callback, queue_size=100, rate=24):
        threading.Thread.__init__(self)

        pub = rospy.Publisher(topic, message, queue_size=queue_size)
        #rate = rospy.Rate(rate)

        self.pub = pub
        self.callback = callback
      

    def run(self):  
        while True:
            ret = self.callback()
            self.pub.publish(ret)