
from std_msgs.msg import Int32
import qwiic_dual_encoder_reader
import time, logging, rospy

from core.utils import *


class SparkfunTwist():
    def __init__(self):
        self.enc = qwiic_dual_encoder_reader.QwiicDualEncoderReader()

        if self.enc.connected == False:
            logg(__name__, "WARNING","Sparkfun Qwiic Dual Encoder Reader device isn't connected to the system. Please check your connection")
            return
        else:
            logg(__name__, "INFO", "Sparkfun Qwiic Dual Encoder Reader has been succesfully initialised")

        self.enc.begin()

    def create_msg(self, data):
        msg = Int32()
        msg.data = data
        return msg

    def read0(self): return self.create_msg(self.enc.count1)

    def read1(self): return self.create_msg(self.enc.count2)