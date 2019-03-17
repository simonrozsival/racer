FROM esteve/ros:kinetic-ros-base-armhf

ENV ROS_IP 192.168.0.147

RUN apt-get update && apt-get install -y \
   git \
   tmux \
   net-tools \
   ros-kinetic-hector-slam \
   ros-kinetic-rosserial-arduino \
   ros-kinetic-rosserial \
   ros-kinetic-amcl

RUN apt-get update && \
    apt-get install -y wget software-properties-common python-software-properties && \
    wget https://archive.raspbian.org/raspbian.public.key -O - | apt-key add - && \
    add-apt-repository -y "deb http://archive.raspberrypi.org/debian/ stretch main" && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 82B129927FA3303E && \
    apt-get update && \
    apt-get install -y libraspberrypi-dev libraspberrypi-bin

# Prepare catkin workspace directory structure
# and download the necessary packages for ROS from GitHub
RUN mkdir -p /ros_catkin_ws/src && \
  cd /ros_catkin_ws/src && \
  git clone https://github.com/eaibot/ydlidar && \
  git clone https://github.com/yoshito-n-students/bno055_usb_stick && \
  git clone https://github.com/yoshito-n-students/bno055_usb_stick_msgs && \
  git clone https://github.com/UbiquityRobotics/raspicam_node

ADD launcher /ros_catkin_ws/src/launcher 

RUN usermod -a -G video root

CMD ["bash"]
