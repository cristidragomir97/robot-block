FROM ubuntu:20.04 
# Set our working directory

RUN apt-get update && apt-get install -y  build-essential python3 python3-dev python3-pip 
WORKDIR /usr/deploy

COPY requirements.txt requirements.txt

# pip install python deps from requirements.txt on the resin.io build server
RUN pip3 install --upgrade pip && CFLAGS="-fcommon" pip3 install -r requirements.txt 
RUN pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy

COPY . ./
WORKDIR /usr/deploy/src

CMD echo 'bash /opt/ros/noetic/setup.bash && python3 main.py --config=config.json' > /usr/deploy/run.sh && chmod +x /usr/deploy/run.sh && sleep infinity

