import rospy, qwiic_scmd, logging
from core.utils import *


class SCMD():
     
    def __init__(self, radius, flip):
        self.radius = radius
        self.flip = flip
        

        try:
            self.sparkfun = qwiic_scmd.QwiicScmd()

            if self.sparkfun.connected == False:
                logg(__name__, "WARNING", "Sparkfun SCMD Motor Driver not connected. Check Connections")

            self.sparkfun.begin()
            self.sparkfun.set_drive(0, 0, 0)
            self.sparkfun.set_drive(1, 0, 0)
            self.sparkfun.enable()

            logg(__name__, "INFO", "Motors successfullly initialised")
        except Exception as e:
            logg(__name__, "ERROR", "Exception initialisitng Sparkfun SCMD Motor Driver {e}")

    def update(self, msg):
        sep = 0.295
        angular = float(msg.angular.z)
        linear = float(msg.linear.x)
        radius = float(self.radius)
        
        right_pwm = (linear / radius) + ((angular * sep) / (2.0 * radius)) * 10
        left_pwm = (linear / radius)  - ((angular * sep) / (2.0 * radius)) * 10

        if right_pwm > 250: right_pwm = 250
        if left_pwm > 250: left_pwm = 250

        logg(__name__, "DEBUG", "updating motor values [angular: {}] [linear: {}] [right_pwm: {}] [left_pwm: {}]".format(angular, linear, right_pwm, left_pwm))

        self.sparkfun.set_drive(0, 0, right_pwm)
        self.sparkfun.set_drive(1, 1, left_pwm)
        