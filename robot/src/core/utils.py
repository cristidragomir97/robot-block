import time

def logg(module, level, message, file="/usr/deploy/robot.log"):
    import datetime

    _format = "[{}] [{}] [{}] {}".format(datetime.datetime.now(), level, module, message)

    with open(file, 'a+') as f:
        print(_format, file=f)
    
    print(_format)
    
    


def delete_folder_contents(folder):
    import os, shutil
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))

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
def scan_usb():
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


