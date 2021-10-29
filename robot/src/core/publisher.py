# import rospy
import threading 

class Publisher(threading.Thread):
    def __init__(self, topic, message, callback, queue_size=100, rate=24):
        pass 

        """
        threading.Thread.__init__(self)

        pub = rospy.Publisher(topic, message, queue_size=queue_size)
        #rate = rospy.Rate(rate)

        self.pub = pub
        self.callback = callback
      
        """
       

    def run(self):  
        """
        while True:
            ret = self.callback()
            self.pub.publish(ret)
        """
    
    def get_id(self):
 
        # returns id of the respective thread
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id