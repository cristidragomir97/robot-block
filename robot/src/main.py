import rospy, argparse, sys, time, os
from pathlib import Path
from core.library import Library
from core.config import Config
from core.factory import Factory 
from core.script import Script
from core.utils import *

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--api", action="store_true")
    parser.add_argument("--roscore", action="store_true")
    parser.add_argument("--config")
    return parser.parse_args()

def ros_startup():
    #try:
    roscore = Script({
        "name": "roscore",
        "command": "roscore",
        "args": []
    })
    roscore.run()
    time.sleep(5)
    rospy.init_node("robotblock", disable_signals=True)
    #except Exception as e:
    #print("error launching ros", e)
    #   sys.exit(1)
    
if __name__ == "__main__":
    args = parse_arguments()
    conf = os.getcwd() + "/" + args.config
    ros_startup()
    
    factory = Factory(library= Library(), config=Config(conf))
