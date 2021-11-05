import os, json, importlib
from colorama import Fore

class Package():
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
                    self.valid = True
                    
                except KeyError as k:
                    print(Fore.YELLOW + "[warning] Can't load {} - field {} is missing ".format(name, k) + Fore.RESET)
                    self.valid = False

        def is_valid(self):
            return self.valid

                

class Library():

    def __init__(self):
        self.packages = dict()
        

        library_dir = os.path.realpath('library')
        library_folders = [f.path  for f in os.scandir(library_dir) if f.is_dir()]

        for full_path in library_folders:
            print(full_path)
            name = full_path.split("/").pop()
            json_filename = full_path + '/{}.json'.format(name)
            python_filename = full_path + '/{}.py'.format(name)

            '''
            if not cond1:
                print pula1
            elif not cond2:
                print pula2
            elif not cond3:
                [rint pula3]
            else:
                stmt 
            '''

            if os.path.exists(python_filename):
                if os.path.exists(json_filename):
                    pack = Package(name, json_filename,)
                    if pack.is_valid():
                        self.packages[name] = pack
                    else:
                        print("INVALID FMM")
                else:
                    print(Fore.YELLOW + "[warning] config file for {} missing, skipping package !".format(name) + Fore.RESET)
            else:
                print(Fore.YELLOW + "[warning] python file for {} missing, skipping package !".format(name) + Fore.RESET)


    def pretty_print(self):
        print(Fore.YELLOW, "Packages: ")
        for package in self.packages: 
            p = self.packages[package]
            print("\t * ", p.name, p.categ, p.type, p.callback, p.ros_message, p.python)
        
        print(Fore.RESET)


    def has_package(self, package):
        return package in self.packages
    
    def get_package(self, package):
        return self.packages[package]



  


    
    

    

   