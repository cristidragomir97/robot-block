import os, json, importlib
from colorama import Fore, Back, Style

class Device():
        def __init__(self, name, file):
            with open(file, 'r') as f:
                try:
                    contents = json.load(f)
                    self.python = file.split('.')[0] + '.py'
                    self.name = contents["name"]
                    self.categ = contents["categ"]
                    self.type = contents["type"]
                    self.callback = contents["callback"]
                    self.ros_message = contents["ros_message"]
                    self.default_topic = contents["default_topic"]
                except KeyError as k:
                    print(Fore.YELLOW + "[warning] Can't load {} - field {} is missing ".format(name, k) + Fore.RESET)


class Library():

    def __init__(self):
        self.devices = dict()

        library_dir = os.path.realpath('../library')
        library_folders = [f.path  for f in os.scandir(library_dir) if f.is_dir()]

        for full_path in library_folders:
            name = full_path.split("/").pop()
            json_filename = full_path + '/{}.json'.format(name)
            python_filename = full_path + '/{}.py'.format(name)

            if os.path.exists(python_filename):
                if os.path.exists(json_filename):
                    self.devices[name] = Device(name, json_filename,)
                else:
                    print(Fore.YELLOW + "[warning] config file for {} missing, skipping device !".format(name) + Fore.RESET)
            else:
                print(Fore.YELLOW + "[warning] python file for {} missing, skipping device !".format(name) + Fore.RESET)

    def has_device(self, device):
        return device in self.devices
    
    def get_device(self, device):
        return self.devices[device]



  


    
    

    

   