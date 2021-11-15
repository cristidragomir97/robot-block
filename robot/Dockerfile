FROM ubuntu:20.04

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


RUN apt-get update && apt-get install -y python3-pip python3-dev python3 curl ca-certificates \
    curl \
    gnupg \
    lsb-release \
    tzdata

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
RUN echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null
  
# INSTALL THE MINIMUM AMOUNT OF ROS POSSIBLE
RUN apt-get update && apt-get install -y ros-noetic-catkin \
    python3-catkin-pkg \
    ros-noetic-genmsg \
    ros-noetic-genpy \
    ros-noetic-std-msgs \
    python3-rosdep \
    ros-noetic-rosbash \
    ros-noetic-rosboost-cfg \
    ros-noetic-message-generation \x
    ros-noetic-rosbuild \
    ros-noetic-rosclean \
    ros-noetic-roscreate \
    ros-noetic-rosgraph \
    ros-noetic-rosgraph-msgs \
    ros-noetic-roslang \
    ros-noetic-roslaunch \
    ros-noetic-roslib \
    ros-noetic-rosmake \
    ros-noetic-rosmaster \
    ros-noetic-rosmsg \
    ros-noetic-rosnode \
    ros-noetic-rosout \
    ros-noetic-rosparam \
    ros-noetic-rospy \
    ros-noetic-rosservice \
    ros-noetic-rostime \
    ros-noetic-rostopic \
    ros-noetic-rosunit \
    ros-noetic-tf2 \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-pkg-modules \
    python3-rosdep-modules 

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

# TODO: Move to upper layers
RUN apt-get update && apt-get install -y python3 python3-pip i2c-tools usbutils

# Set our working directory
WORKDIR /usr/src/app

# Copy requirements.txt first for better cache on later pushes
COPY requirements.txt requirements.txt

# pip install python deps from requirements.txt on the resin.io build server
RUN pip install -r requirements.txt

# This will copy all files in our root to the working  directory in the container
COPY . ./

# Enable udevd so that plugged dynamic hardware devices show up in our container.
ENV UDEV=1

RUN groupadd -f -r gpio && usermod -a -G gpio && modprobe i2c-dev 
# main.py will run when container starts up on the device

CMD sleep infinity