
import rospy, time, board, busio, math, logging
import adafruit_lsm9ds1
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

class LSM9DS1():
    def __init__(self):
    

        # I2C connection:
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

        rospy.loginfo("* IMU successfullly initialised")



    def read(self):
        accel_x, accel_y, accel_z = self.sensor.acceleration
        gyro_x, gyro_y, gyro_z = self.sensor.gyro


        msg = Imu()
        msg.header.frame_id = "imu"
        msg.header.stamp = rospy.get_rostime()

        msg.angular_velocity.x = (gyro_x *  gRes - gOff[0]) * math.pi / 180.0
        msg.angular_velocity.y = (gyro_y *  gRes - gOff[1]) * math.pi / 180.0
        msg.angular_velocity.z = (gyro_z *  gRes - gOff[2]) * math.pi / 180.0

        msg.angular_velocity_covariance[0] = gOff[0] * gOff[0]
        msg.angular_velocity_covariance[4] = gOff[1] * gOff[1]
        msg.angular_velocity_covariance[8] = gOff[2] * gOff[2]

        msg.linear_acceleration.x = (accel_x *  aRes - aOff[0]) * math.pi / 180.0
        msg.linear_acceleration.y = (accel_y *  aRes - aOff[1]) * math.pi / 180.0
        msg.linear_acceleration.z = (accel_z *  aRes - aOff[2]) * math.pi / 180.0

        msg.linear_acceleration_covariance[0] = aOff[0] * aOff[0]
        msg.linear_acceleration_covariance[4] = aOff[1] * aOff[1]
        msg.linear_acceleration_covariance[8] = aOff[2] * aOff[2]

        return msg 


