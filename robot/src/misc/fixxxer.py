import RPi.GPIO as GPIO
import VL53L1X
import time

# default address of VL53L1X
ADDRESS = 0x29

# pins for the shutdown pins
SENSOR_A = 17
SENSOR_B = 27
SENSOR_C = 22

# desired addresses
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
    time.sleep(0.5)

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM) 
    GPIO.setwarnings(False) 

    GPIO.setup(SENSOR_A, GPIO.OUT)
    GPIO.setup(SENSOR_B, GPIO.OUT)
    GPIO.setup(SENSOR_C, GPIO.OUT)

    # change address of second sensor
    select(GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
    change(SENSOR_B_ADDRESS)

    # change address of the last sensor
    select(GPIO.HIGH, GPIO.HIGH, GPIO.LOW)
    change(SENSOR_C_ADDRESS)

    # take sensors out of sleep
    select(GPIO.HIGH, GPIO.HIGH, GPIO.HIGH)


    


  