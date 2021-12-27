
#!/bin/bash

git clone https://github.com/cristidragomir97/robot-block-lib /usr/deploy/src/library && . /opt/ros/noetic/setup.sh && python3 main.py --config=config.json --roscore