import rospy, qwiic_scmd
from utils import compute_pwm

class SparkfunDriver():
     
    def __init__(self, radius, flip):
        self.radius = radius
        self.flip = flip

        try:
            self.sparkfun = qwiic_scmd.QwiicScmd()

            if self.sparkfun.connected == False:
                print('Motor Driver not connected. Check Connections')

            self.sparkfun.begin()
            self.sparkfun.set_drive(0, 0, 0)
            self.sparkfun.set_drive(1, 0, 0)
            self.sparkfun.enable()

            print("* Motors	successfullly initialised")
        except Exception as e:
            print("\033[91m* Exception initialisitng Sparkfun Driver", e )

    def update(self, msg):
        angular = msg.angular.z
        linear = msg.linear.x

        
        right_pwm, left_pwm = compute_pwm(angular, linear, self.radius)
        print(right_pwm, left_pwm, angular, linear)

        self.sparkfun.set_drive(0, 0, right_pwm)
        self.sparkfun.set_drive(1, 1, left_pwm)
        