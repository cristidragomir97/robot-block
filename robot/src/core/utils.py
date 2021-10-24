import time
from constants import *

### CONVERSION UTILS 
###################
def compute_pwm(angular, linear, radius):
    sep = 0.295
    right_pwm = (linear / radius) + ((angular * sep) / (2.0 * radius)) * 10
    left_pwm = (linear / radius)  - ((angular * sep) / (2.0 * radius)) * 10
    return right_pwm, left_pwm

def angle_to_pwm( angle):
    return (angle/18.0) + 2.5

### INTERFACE UTILS 
###################
def detect_usb():
    import re
    import subprocess
    rgx = b"Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$"
    device_re = re.compile(rgx, re.I)
    df = subprocess.check_output("lsusb")
    devices = []
    for i in df.split(b'\n'):
        if i:
            info = device_re.match(i)
            if info:
                dinfo = info.groupdict()
                dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
                devices.append(dinfo)
                
    print(devices)

def tof_address_fix(pins = [17, 27, 22]):

    import RPi.GPIO as GPIO
    import VL53L1X
    import time

    set_pins = lambda x: GPIO.setup(x, GPIO.OUT)
    set_state = lambda x, y: GPIO.output(x, y)

    def change(new, old):
        tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=old)
        tof.open()
        tof.change_address(new)
        tof.close()
        time.sleep(0.5)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False) 

    # setup sensor shutdown pins as output
    map(set_pins, pins)


    map(set_state, pins , [GPIO.HIGH, GPIO.LOW, GPIO.HIGH])
    time.sleep(0.2)

    change(0x31, 0x29)

    map(set_state, pins , [GPIO.HIGH, GPIO.HIGH, GPIO.LOW])
    time.sleep(0.2)
    
    change(0x32, 0x29)
    
    map(set_state, pins , [GPIO.HIGH, GPIO.HIGH, GPIO.HIGH])

#!/usr/bin/env python
def scan_bus():
    import smbus2
    lst = []

    bus = smbus2.SMBus(1) # 1 indicates /dev/i2c-1

    for device in range(128):

        try:
            bus.read_byte(device)
            lst.append(hex(device))

        except: # exception if read_byte fails
            pass

    return lst
