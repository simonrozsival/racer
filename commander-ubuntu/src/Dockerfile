FROM osrf/ros:kinetic-desktop-full

RUN apt-get update && apt-get install -y \
   iputils-ping \
   net-tools \
   curl \
   wget \
   vim \
   sudo \
   libgl1-mesa-glx \
   libgl1-mesa-dri \
   mesa-utils \
   unzip \
   python-visual \
   ros-kinetic-razor-imu-9dof \
   ros-kinetic-pcl-conversions \
   ros-kinetic-pointcloud-to-laserscan \
   && rm -rf /var/likb/apt/lists/*

# Prepare Catkin workspace directory structure
RUN mkdir -p /ros_catkin_ws/src

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
  rm -rf sweep-sdk

# Download Sweep ROS package
RUN cd ros_catkin_ws/src && git clone https://github.com/scanse/sweep-ros

ENV ROS_MASTER_URI http://192.168.1.2:11311
ENV ROS_IP 192.168.1.1

ADD . /commander-ubuntu

CMD ["bash"]

