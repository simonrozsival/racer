FROM esteve/ros:kinetic-ros-base-armhf

ENV ROS_IP 192.168.0.2

RUN apt-get update && apt-get install -y \
   git \
   tmux \
   net-tools \
   ros-kinetic-razor-imu-9dof \
   ros-kinetic-pcl-conversions \
   ros-kinetic-pointcloud-to-laserscan \
   ros-kinetic-hector-slam \
   ros-kinetic-rosserial-arduino \
   ros-kinetic-rosserial

# Install Sweep SDK
RUN git clone https://github.com/scanse/sweep-sdk && \
  cd sweep-sdk/libsweep && \
  mkdir build && \
  cd build && \
  cmake .. -DCMAKE_BUILD_TYPE=Release && \
  cmake --build . && \
  cmake --build . --target install && \
  ldconfig && \
  cd ../../../ && \
  rm -rf sweep-sdk \
  rm /ros_entry_point.sh

# Prepare catkin workspace directory structure
# and download the sweep package for ROS
RUN mkdir -p /ros_catkin_ws/src && \
  cd /ros_catkin_ws/src && \
  git clone https://github.com/scanse/sweep-ros

ADD launcher /ros_catkin_ws/src/launcher 

CMD ["bash"]
