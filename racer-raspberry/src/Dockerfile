FROM esteve/ros:kinetic-ros-base-armhf

ENV ROS_IP 192.168.0.2

RUN apt-get update && apt-get install -y \
   git \
   tmux \
   net-tools \
   ros-kinetic-pcl-conversions \
   ros-kinetic-hector-slam \
   ros-kinetic-rosserial-arduino \
   ros-kinetic-rosserial \
   ros-kinetic-joy \
   ros-kinetic-raspicam-node

# Prepare catkin workspace directory structure
# and download the necessary packages for ROS from GitHub
RUN mkdir -p /ros_catkin_ws/src && \
  cd /ros_catkin_ws/src && \
  git clone https://github.com/eaibot/ydlidar && \
  git clone https://github.com/yoshito-n-students/bno055_usb_stick && \
  git clone https://github.com/yoshito-n-students/bno055_usb_stick_msgs

ADD launcher /ros_catkin_ws/src/launcher 
ADD joystick_controller /ros_catkin_ws/src/joystick_controller

CMD ["bash"]
