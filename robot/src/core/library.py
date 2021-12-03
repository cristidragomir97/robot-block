import os, json, importlib, sys, subprocess, pkg_resources 

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])
'''
{
    "name": "ICM20948",
    "info": "9-Axis MEMS IMU",
    "dependencies": [
        {
            "type": "pip3",
            "package":"sparkfun-qwiic-icm20948"
        }
    ],
    "callback": "read",
    "ros_message": ["sensor_msgs.msg","Imu"],
    "address": ["0x69"]
}
'''
class Package():
        def __init__(self, file):
            with open(file, 'r') as f:
                try:
                    contents = json.load(f)
                    self.python = file.split('.')[0] + '.py'
                    self.name = contents['name']
                    self.info = contents['info']
                    self.dependencies = contents["dependencies"]
                    self.callback = contents["callback"]
                    self.ros_message = contents["ros_message"]
                    self.valid = True
                
                    for dep in self.dependencies:
                        if dep["type"] == "pip3":
                            # check if package is installed first 

                            # and install if needed 
                            install(dep["package"])

                except Exception as e:
                    print(e)
                    self.valid = False
                    
                except KeyError as k:
                    print("[warning] Can't load {} - field is missing ".format(k))
                    self.valid = False

        def is_valid(self):
            return self.valid

                

class Library():

    def __init__(self):
        self.packages = dict()
        sys.stdout.register('logs/main.log')

        library_dir = os.path.realpath('library')
        library_folders = [f.path  for f in os.scandir(library_dir) if f.is_dir()]

        for full_path in library_folders:
            name = full_path.split("/").pop()
            json_filename = full_path + '/{}.json'.format(name)
            python_filename = full_path + '/{}.py'.format(name)


            if os.path.exists(python_filename):
                if os.path.exists(json_filename):
                    pack = Package(name, json_filename,)
                    if pack.is_valid():
                        self.packages[name] = pack
                        print("[info] Package {} loaded".format(name), pack)
                    else:
                        print("INVALID FMM")
                else:
                    print("[warning] config file for {} missing, skipping package !".format(name))
            else:
                print("[warning] python file for {} missing, skipping package !".format(name))


    def pretty_print(self):
        print("- Packages: ")
        for package in self.packages: 
            p = self.packages[package]
            print("-->", p.name, p.info, p.dependencies, p.callback, p.ros_message, p.python)
    

    def has_package(self, package):
        return package in self.packages
    
    def get_package(self, package):
        return self.packages[package]



  


    
    

    

   