import rospy, argparse, sys, time, os

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
            print(args.roscore)
            Roscore().run()
            time.sleep(5)

        rospy.init_node("robotblock", anonymous=False, disable_signals=True)
    except Exception as e:
        print("* error launching ros", e)
        sys.exit(1)
    
if __name__ == "__main__":
    args = parse_arguments()
    ros_startup(args)
    
    
    library = Library()
    config_path = os.getcwd() + "/" + args.config
    print(config_path)
    
    config = Config(config_path)
    factory = Factory(library, config)

    if args.api:
        api = API(factory, config)
        api.start()        

