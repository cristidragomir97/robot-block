import rospy, VL53L1X
import RPi.GPIO as GPIO
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
import time

from core.utils import *


SENSOR_A = 17
SENSOR_B = 27
SENSOR_C = 22

ADDRESS = 0x29

SENSOR_A_ADDRESS = 0x30
SENSOR_B_ADDRESS = 0x31
SENSOR_C_ADDRESS = 0x32

def select(A, B, C):
    GPIO.output(SENSOR_A, A)
    GPIO.output(SENSOR_B, B)
    GPIO.output(SENSOR_C, C)
    time.sleep(0.5)

def change(new):
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=ADDRESS)
    tof.open()
    tof.change_address(new)
    tof.close()
    time.sleep(1)



class VL53L1_Array():

    def __init__(self):

        GPIO.setmode(GPIO.BCM) 
        try:
            GPIO.setwarnings(False) 

            GPIO.setup(SENSOR_A, GPIO.OUT)
            GPIO.setup(SENSOR_B, GPIO.OUT)
            GPIO.setup(SENSOR_C, GPIO.OUT)

            logg(__name__, "DEBUG", "changing address for first sensor")
            select(GPIO.LOW, GPIO.HIGH, GPIO.HIGH)
            change(SENSOR_A_ADDRESS)
            time.sleep(1)

            # change address of second sensor
            logg(__name__, "DEBUG", "changing address for second sensor")
            select(GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
            change(SENSOR_B_ADDRESS)
            time.sleep(1)

            # change address of the last sensor
            logg(__name__, "DEBUG", "changing address for third sensor")
            select(GPIO.HIGH, GPIO.HIGH, GPIO.LOW)
            change(SENSOR_C_ADDRESS)
            time.sleep(1)

            # take sensors out of sleep
            select(GPIO.HIGH, GPIO.HIGH, GPIO.HIGH)
            logg(__name__, "DEBUG", "resetting all sensors")

            time.sleep(1)

        except Exception as e:
            logg(__name__, "ERROR", "Error changing addresses of ToF Sensors {e}")

        try:
            self.tof0 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=SENSOR_C_ADDRESS)
            self.tof0.open()
            self.tof0.start_ranging(0) 
            self.tof0.set_timing(30000, 33)

            self.tof1 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=SENSOR_B_ADDRESS)
            self.tof1.open()
            self.tof1.start_ranging(0) 
            self.tof1.set_timing(30000, 33)

            self.tof2 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
            self.tof2.open()
            self.tof2.start_ranging(0) 
            self.tof2.set_timing(30000, 33)
        except Exception as e:
            logg(__name__, "ERROR", "Error initialising array{e}")

    def create_msg(self, data):
        msg = Int32()
        msg.data = data
        return msg

    def read0(self): return self.create_msg(self.tof0.get_distance())
    def read1(self): return self.create_msg(self.tof1.get_distance())
    def read2(self): return self.create_msg(self.tof2.get_distance())
        

        






    
    


  