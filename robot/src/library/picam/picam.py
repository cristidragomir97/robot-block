import cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time, logging 
from core.utils import *




class picam():

    def __init__(self):
        
        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)
        logg(__name__, "INFO", "successfully initialised camera on /dev/video0")
        #self.camera.set('Brightness', 20)
        #self.camera.set('Contrast', 128)
        #self.camera.set('Saturation', 255)

    def release(self):
        self.camera.release()

    def read(self):
        ret, frame = self.camera.read()
        cv2.normalize(frame, frame, 0, 255, cv2.NORM_MINMAX)
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        return img_msg
