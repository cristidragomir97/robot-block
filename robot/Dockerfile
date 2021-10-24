FROM cristidragomir97/rospack:latest

COPY --from=cristidragomir97/rospack:latest /opt/ros/ /opt/ros/
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

CMD groupadd -f -r gpio && usermod -a -G gpio && modprobe i2c-dev 
# main.py will run when container starts up on the device

CMD ["python3","-u","src/main.py"]
#CMD ["i2cdetect", "-r", "-y", "1"]


