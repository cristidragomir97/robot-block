FROM balenalib/raspberrypi3-64-debian:buster

ARG ros_source=/opt/ros/noetic/setup.sh
ARG catkin_ws=/root/catkin_ws
ARG key_url=https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc

ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost

RUN apt update && apt install -y lsb-release ca-certificates wget cmake build-essential  \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - 

RUN install_packages curl python3 python3-pip python3-dev python3 python3-setuptools i2c-tools usbutils ros-noetic-catkin \
    v4l-utils \
    libopus-dev \
    libvpx-dev \
    libsrtp2-dev \
    libopencv-dev \
    libatlas3-base \
    libatlas-base-dev \
    libavcodec-dev \
    libcamera0 \
    libcamera-apps \
    libcamera-tools \
    libcamera-dev \
    libraspberrypi0 \
    libraspberrypi-dev \
    libraspberrypi-doc \
    python3-catkin-pkg \
    ros-noetic-genmsg \
    ros-noetic-genpy \
    ros-noetic-std-msgs \
    python3-rosdep \
    ros-noetic-rosboost-cfg \
    ros-noetic-message-generation \
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
    python3-rosdep-modules \
    ros-noetic-sensor-msgs \
    ros-noetic-cv-bridge 

RUN . ${ros_source} && mkdir --parents ${catkin_ws}/src && cd ${catkin_ws} && catkin_make

RUN cd ${catkin_ws}/src && \
    git clone https://github.com/dheera/rosboard 

RUN  . ${ros_source} && cd ${catkin_ws} && catkin_make && catkin_make install

# Enable the v4l2 driver for the Raspberry Pi camera
RUN printf "bcm2835-v4l2\n" >> /etc/modules

# Set our working directory
WORKDIR /usr/deploy

COPY requirements.txt requirements.txt

# pip install python deps from requirements.txt on the resin.io build server
RUN pip3 install --upgrade pip && pip3 install -r requirements.txt

# This will copy all files in our root to the working  directory in the container
COPY . ./

CMD . /opt/ros/noetic/setup.sh \
    && cd src && python3 main.py --roscore --api --config=conf/sudo.json