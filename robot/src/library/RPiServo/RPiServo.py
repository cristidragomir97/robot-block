import Jetson.GPIO as GPIO

class GPIOServo(Servo):
    

    def __init___(self, name, pin):

        GPIO.setmode(GPIO.BOARD)     # Board pin-numbering scheme
        GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH) # set pin as an output pin with optional initial state of HIGH
        self.pwm = GPIO.PWM(self.pin, 50) # 50 = frequency 
        self.pwm.start(VALUE)

    def set_angle(self, angle)
        pwm = super().angle_to_pwm(angle)
        self.pwm.ChangeDutyCycle(pwm)

    def stop(self):
        self.pwm.stop()
        GPIO.cleanup()


