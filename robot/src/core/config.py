
import json
from colorama import Fore, Back

class Sensors:
    def __init__(self):
        self.cameras = []
        self.lidar = {}
        self.ranging = []
        self.imu = []
        self.power = []

        # camera
        if has_key(sensors, "camera"):
           
            cameras = sensors["camera"]  
            print("\n[config] {} camera(s):".format(len(cameras))) 

            for camera in cameras: 
                print("[config]\t ---> type: {}, model: {}".format(camera["type"], camera["model"]))
                self.sensors.cameras.append(camera)
                

        # LIDAR
        if has_key(sensors, "LIDAR"):
            lidar = sensors["LIDAR"]
            print("\n* LIDAR: ")  
            print("[config]\t ---> type: {}, model: {}".format(lidar["type"], lidar["model"]))
            self.sensors.lidar = lidar

         # range sensors
        if has_key(sensors, "ranging"):
            print("\n[config] {} ranging sensor(s) :".format(len(sensors["ranging"])))

            ranging = sensors["ranging"]
            for sensor in ranging:
                print("[config] \t ---> type: {}, topic: {}".format(sensor["type"], sensor["topic"]))
                self.sensors.ranging.append(sensor)
        
        if has_key(sensors, "IMU"):
            print("\n[config]  IMU available")
            imus = sensors["IMU"]

            for sensor in imus:
                topic = sensor["topic"]
                kind = sensor["type"]
                print("[config] \t ---> type:{}, topic: {}".format(kind, topic))

                self.sensors.imu.append(sensor)


        if has_key(sensors, "power"):
            print("\n[config]  power available")
            power = sensors["power"]

            for sensor in power:
                kind = sensor["type"]
                topic = sensor["topic"]
                print("[config] \t ---> type:{}, topic: {}".format(kind, topic))

                self.sensors.power.append(sensor)



class Sensor:
     def __init__(self):
        pass


class Actuators:
    def __init__(self):
        pass

def has_key(object, key):
    try: 
        object["{}".format(key)]
        return True
    except ValueError:
        return False

# helper classes that allow us to do deep chaining 


class ConfigParser:

    def __init__(self, file="config.json"):
        print("Configuration loaded: \n")
        with open(file) as f:

            self.sensors = Sensors()

            config = json.load(f)
            config = config[0]

            # general stuff
            self.name = config["name"]
            self.desc = config["desc"]
    
            # driver
            self.driver = config["driver"]

            # actuators
            if has_key(config, "actuators"):
                actuators = config["actuators"]
                self.parse_actuators(actuators)

            # sensors
            if has_key(config, "sensors"):
                sensors = config["sensors"]
                self.parse_sensors(sensors)


    def parse_sensors(self, sensors):
     

    def parse_actuators(self, actuators):
        print("[config] {} ranging sensor(s) :".format(len(actuators)))
        for actuator in actuators:
            print("[config]\t * type: {}, topic: {}".format(actuator["type"], actuator["topic"]))

    def get_driver(self):
        pid, radius, kind, address, flip = list(self.driver.values())
        return pid, radius, kind, address, flip

    def imus(self):
        return len(self.sensors.imu)

    def get_imu(self, i):
        return self.sensors.imu[i].values()

    def ranging(self):
        return len(self.sensors.ranging)

    def get_ranging(self, i):
        return self.sensors.ranging[i].values()
    
    def power(self):
        return len(self.sensors.power)

    def get_power(self, i):
        return self.sensors.power[i].values()

    def cameras(self):
        return len(self.sensors.cameras)

    def get_camera(self, i):
        return self.sensors.cameras[i].values()

    def get_lidar(self):
        return self.sensors.lidar.values()







