from library import Device, Library
from config import ConfigParser
from subscriber import Subscriber
from publisher import Publisher

def import_device(device):
    loader = importlib.machinery.SourceFileLoader(device.name, device.python)
    return loader.load_module()


def init_device(module, device, args=None):
    return getattr(module, device.name)(args)



def execute(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = ''

    # Poll process for new output until finished
    for line in iter(process.stdout.readline, ""):
        print(line)
        output += str(line)

    process.wait()
    exitCode = process.returncode

    if (exitCode == 0):
        return output
    else:
        raise Exception(command, exitCode, output)

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




class Factory():
    def __init__(self):
        self.lib = Library()
        self.parser = ConfigParser()

        self.handle_driver()

        self.handle_imus()
       

    
    


