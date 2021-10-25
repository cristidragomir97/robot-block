import json, importlib, importlib.machinery

from library import LibraryItem, Library
from config import Config, Interface, Device, External
from subscriber import Subscriber
from publisher import Publisher

class Factory():

    def __init__(self):
        self.threads = {}

        self.lib = Library()
        self.conf = Config('../config.json')

        self.lib.pretty_print()
        self.conf.pretty_print()

        for device in self.conf.devices():
            if device.role == "publisher":
                thread = self.make_publisher(device)
                self.threads[device.topic] = thread
                
            elif device.role == "subscriber":
                thread = self.make_subscriber(device)
                self.threads[device.topic] = thread

            elif device.role == "service":
                thread = self.make_service(device)
                self.threads[device.topic] = thread

            elif device.role == "external":
                thread = self.make_external(device)
                self.threads[device.topic] = thread

    def make_publisher(self, dev):
        try:
            lib = self.lib.get_device(dev.library)
            print(lib.python, lib.name, lib.callback, lib.ros_message)

        except AttributeError as e:
            print(dev.library, " has not been found  Error: ", e)


    def make_subscriber(self, dev):
        try:
            lib = self.lib.get_device(dev.library)
            loader = importlib.machinery.SourceFileLoader(lib.name, lib.python)
            module = loader.load_module()
            
            instance = getattr(module, lib.name)
            instance(dev.args)

            callback = getattr(instance, lib.callback)

            return Subscriber(dev.topic, lib.ros_message, callback)

        except AttributeError as e:
            print(dev.library, " has not been found library. Error: ", e)

    def make_service(self, device):
        pass

    def make_external(self, device):
        pass

if __name__ == "__main__":
    factory = Factory()





'''

def handle_imus():
       for imu in range(0,parser.imus()):
            topic, kind, address, use_filter, filter_type = parser.get_imu(imu)
            
            if kind == "LSM9DS1":
                from hardware.imu._LSM9DS1 import _LSM9DS1
                imu = _LSM9DS1()

            if kind == "BNOO55":
                from hardware.imu_BNO055 import _BNO055
                imu = _BNO055()

            Publisher(topic, Imu, imu.read).start()

def handle_ranging():
         for index in range(0,parser.ranging()):
            kind, topic, address, angle, fov, _range, rate,  = parser.get_ranging(index)
             
            if kind == "VL53L1":
                 from hardware.sensors.VL53L1 import _VL53L1
                 sensor = _VL53L1(address=address)

            Publisher(topic, Range, sensor.read).start()

def handle_power():
    for index in range(0, parser.power()):
        kind, topic, address = parser.get_power(index)
            
        if kind == "INA219":
            from hardware.sensors.INA219 import _INA219
            sensor = _INA219(address)

            Publisher(topic, BatteryState, sensor.read).start()

def handle_cameras():
    for index in range (0, parser.cameras()):
        kind, model , color_obj, depth_obj = parser.get_camera(index)

        if kind == "realsense":
            arguments = " filters:=pointcloud Initial_reset:=true "
         
            depth_en = depth_obj["enable"]
            if depth_en == "true":
                depth_fps = depth_obj["rate"]
                depth_res = depth_obj["resolution"]
                depth_width = depth_res[0]
                depth_height = depth_res[1]

                arguments += "depth_width:={} depth_height:={} depth_fps:={} ".format(depth_width, depth_height, depth_fps)

            color_en = color_obj["enable"]
            if color_en == "true":
                color_fps = color_obj["rate"]
                color_res = color_obj["resolution"]
                color_width = color_res[0]
                color_height = color_res[1]

                arguments += "color_width:={} color_height:={} color_fps:={}".format(color_width, color_height, color_fps)
            

            execute(". ~/catkin_ws/devel/setup.sh && roslaunch realsense2_camera rs_camera.launch {}".format(arguments))
            
def handle_lidar():
    kind, *_ = parser.get_lidar()

    if kind == "RPLidar":
        execute(". ~/catkin_ws/devel/setup.sh && roslaunch rplidar_ros rplidar.launch")
            
def handle_driver():

    pid, radius, kind, address, flip = parser.get_driver()

    # switch based on driver type
    if kind == "sparkfun":
        sparkfun = SparkfunDriver(radius=radius, flip=flip)

        # check for connection 
        if address in scan_bus():

            sub = Subscriber(
                topic="cmd_vel", 
                message=Twist,
                callback=sparkfun.update)

            sub.start()

    if kind == "L298N":
        print("sorry, motor driver not implemented yet")
        pass

    if kind == "adafruit":
        print("sorry, motor driver not implemented yet")
        pass

def handle_filter():
    execute("rosrun imu_complementary_filter complementary_filter_node _fixed_frame:=camera_link, _use_mag:=false _do_bias_estimation:=true _do_adaptive_gain:=false _publish_tf:=true")

'''




