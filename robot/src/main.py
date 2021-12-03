import rospy, argparse, sys, time, os
from pathlib import Path

from core.utils import *
from core.log import Log
from core.library import Library
from core.config import Config
from core.factory import Factory 
from core.roscore import Roscore
from core.api import API

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--api", action="store_true")
    parser.add_argument("--roscore", action="store_true")
    parser.add_argument("--config")
    return parser.parse_args()

def ros_startup(args):
    try:
        if(args.roscore):
            Roscore().run()
            time.sleep(5)

        rospy.init_node("robotblock", anonymous=False, disable_signals=True)
    except Exception as e:
        print("* error launching ros", e)
        sys.exit(1)
    
if __name__ == "__main__":
    Path("logs").mkdir(parents=True, exist_ok=True)
    delete_folder_contents('logs')
    args = parse_arguments()
    ros_startup(args)
    
    sys.stdout = Log()
    sys.stdout.register('logs/main.log')
    
    library = Library()
    config_path = os.getcwd() + "/" + args.config
    
    config = Config(config_path)
    factory = Factory(library, config)

    if args.api:
        api = API(factory, config)
        api.start()        

    for thread in factory.threads.values():
        print(thread)
