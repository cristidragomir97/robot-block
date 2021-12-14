
import rospy, time, board, busio, math, logging
import qwiic_icm20948
from sensor_msgs.msg import Imu
from core.utils import * 

# Calibration constants
gRes = 6.1e-05
aRes = 0.00875
mRes = 0.00875

gOff = [-0.05, 0.35, -0.73]
aOff = [0.04, -0.1, -0.01]
mOff = [-0.00563, 0.01678, -0.0068]

soft_iron = [0.99, -0.002, -0.005, -0.002, 0.989, -0.075, -0.005, -0.075, 1.027]

class ICM20948():
    def __init__(self, address):
        self.sensor = qwiic_icm20948.QwiicIcm20948()
        self.sensor.begin()

        if self.sensor.connected == False:
            logg(__name__, "WARNING", "Error connecting to device. Please check your connection")
            return
        logg(__name__, "INFO", "Succesfully connected")

    def read(self):
        if self.sensor.dataReady():
            self.sensor.getAgmt()

        msg = Imu()
        msg.header.frame_id = "imu"
        msg.header.stamp = rospy.get_rostime()

        msg.angular_velocity.x = (self.sensor.gxRaw *  gRes - gOff[0]) * math.pi / 180.0
        msg.angular_velocity.y = (self.sensor.gyRaw *  gRes - gOff[1]) * math.pi / 180.0
        msg.angular_velocity.z = (self.sensor.gzRaw *  gRes - gOff[2]) * math.pi / 180.0

        msg.angular_velocity_covariance[0] = gOff[0] * gOff[0]
        msg.angular_velocity_covariance[4] = gOff[1] * gOff[1]
        msg.angular_velocity_covariance[8] = gOff[2] * gOff[2]

        msg.linear_acceleration.x = (self.sensor.axRaw *  aRes - aOff[0]) * math.pi / 180.0
        msg.linear_acceleration.y = (self.sensor.ayRaw *  aRes - aOff[1]) * math.pi / 180.0
        msg.linear_acceleration.z = (self.sensor.azRaw *  aRes - aOff[2]) * math.pi / 180.0

        msg.linear_acceleration_covariance[0] = aOff[0] * aOff[0]
        msg.linear_acceleration_covariance[4] = aOff[1] * aOff[1]
        msg.linear_acceleration_covariance[8] = aOff[2] * aOff[2]

        return msg 





