import VL53L1X

from sensor_msgs.msg import Range

class _VL53L1():

    def __init__(self, address=0x29):
        address = int(address, 16)
        print("initialising sensor with address: {}".format(hex(address)))
        
        try:
            self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=address)
            self.tof.open()
            self.tof.start_ranging(0) 
            self.tof.set_timing(30000, 33)
        except Exception as e:
            print(e)

    def set_range(self, rng):
        if rng < 4 and rng >= 0:
            self.tof.set_range()
        else:
            raise Exception("Invalid range: 1 - short, 2 - med,  3 - long")

    def set_fov(self, mode):

        if mode == "wide": 
            roi = VL53L1X.VL53L1xUserRoi(0, 15, 15, 0)

        elif mode == "center":
            roi = VL53L1X.VL53L1xUserRoi(6, 9, 9, 6)

        elif mode == "top":
            roi = VL53L1X.VL53L1xUserRoi(6, 15, 9, 12)

        elif mode == "bottom":
            roi = VL53L1X.VL53L1xUserRoi(6, 3, 9, 0)

        elif mode == "left":
            roi = VL53L1X.VL53L1xUserRoi(0, 9, 3, 6)

        elif mode == "right":
            roi = VL53L1X.VL53L1xUserRoi(12, 9, 15, 6)
            
        else:
            roi = VL53L1X.VL53L1xUserRoi(0, 15, 15, 0)

        self.tof.set_user_roi(roi)

    def read(self):

        dist = self.tof.get_distance()
   
        msg = Range()
        msg.radiation_type = 1
        msg.field_of_view = 27
        msg.min_range = 0
        msg.max_range = 400
        msg.range = float(dist)

        return msg 