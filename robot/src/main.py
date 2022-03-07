import rospy, argparse, sys, time, os
from pathlib import Path
from core.library import Library
from core.config import Config
from core.factory import Factory 
from core.script import Script
from core.utils import *

    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    args =  parser.parse_args()
    conf = os.getcwd() + "/" + args.config
    rospy.init_node("robotblock", disable_signals=True)
    Factory(library= Library(), config=Config(conf))


