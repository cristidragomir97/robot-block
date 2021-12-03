import time
import board
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from std_msgs.msg import Int32

class ADS1115():
    def __init__(self, address):
        i2c = board.I2C()
        self.device = ADS.ADS1115(i2c)
        self.chan_0 = AnalogIn(self.device, ADS.P0)
        self.chan_1 = AnalogIn(self.device, ADS.P1)
        self.chan_2 = AnalogIn(self.device, ADS.P2)
        self.chan_3 = AnalogIn(self.device, ADS.P3)

    def create_msg(self, data):
        msg = Int32()
        msg.data = data
        return msg
        
    def read0(self):
        return self.create_msg(self.chan_0.value)
        
    def read1(self):
        return self.create_msg(self.chan_1.value)

    def read2(self):
        return self.create_msg(self.chan_2.value)

    def read3(self):
        return self.create_msg(self.chan_3.value)



