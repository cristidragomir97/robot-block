import json, sys, rospy, roslaunch, time, os, threading, subprocess
from time import sleep

# ROS MESSAFES
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState

# LIRARY
from roscore import Roscore
from utils import scan_bus, detect_usb, tof_address_fix
from config import ConfigParser
from subscriber import Subscriber
from publisher import Publisher

# HARDWARE
from hardware.driver.Sparkfun import SparkfunDriver
from hardware.imu._LSM9DS1 import _LSM9DS1


if __name__ == "__main__":
    # read robot configuration from file and create parser instance
    parser = ConfigParser()

    # create instance of roscore, that shuts down along with this 
    Roscore().run()
    time.sleep(2)

    # register nodes
    rospy.init_node("bot", anonymous=False, disable_signals=True)
    
    # checks for multiple VL53L1X sensors, and fixes addresses accordingly 
    #fixxer()

    # Motor Driver 
    handle_driver()

    # I2C Connected sensors
    #handle_imus()
    #handle_ranging()
    #handle_power()

    # these sensors are handled external scripts, 
    # so we run them in seppararate threads
    #threading.Thread(target=handle_lidar).start()
    #threading.Thread(target=handle_cameras).start()
    #threading.Thread(target=handle_filter).start()