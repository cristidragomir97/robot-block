import rospy, time
import board, busio
import adafruit_ads1x15.ads1015 as ADS
from threading import Thread
from adafruit_ads1x15.analog_in import AnalogIn
from std_msgs.msg import Int32
from queue import Queue
from core.utils import *

BUF_SIZE = 20
q0 = Queue(BUF_SIZE)
q1 = Queue(BUF_SIZE)
q2 = Queue(BUF_SIZE)
q3 = Queue(BUF_SIZE)


class ADS1115():
    def __init__(self, address):

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.device = ADS.ADS1015(self.i2c)
        self.chan_0 = AnalogIn(self.device, ADS.P0)
        self.chan_1 = AnalogIn(self.device, ADS.P1)
        self.chan_2 = AnalogIn(self.device, ADS.P2)
        self.chan_3 = AnalogIn(self.device, ADS.P3)

        time.sleep(2)

        logg(__name__, "INFO", "ADS1015 4-channel ADC initialised ")
        Thread(target = self.refresh, args = ()).start()

    def refresh(self):
        while True:
            try:
                q0.put(self.chan_0.value)
                q1.put(self.chan_1.value)
                q2.put(self.chan_2.value)
                q3.put(self.chan_3.value)
    
            except Exception as e:
                logg(__name__, "ERROR", str(e))
                
            time.sleep(1/60)


    def create_msg(self, data):
        msg = Int32()
        msg.data = data
        return msg
        
    def read0(self):
        return self.create_msg(q0.get())
    
    def read1(self):
        return self.create_msg(q1.get())

    def read2(self):
        return self.create_msg(q2.get())
      
    def read3(self):
        return self.create_msg(q3.get())


