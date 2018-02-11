FROM esteve/ros:kinetic-ros-base-armhf

RUN apt-get update && apt-get install -y \
   git \
   net-tools \
   ros-kinetic-razor-imu-9dof \
   ros-kinetic-pcl-conversions \
   ros-kinetic-pointcloud-to-laserscan

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

# Prepare catkin workspace directory structure
# and download the sweep package for ROS
RUN mkdir -p ros_catkin_ws/src && \
  cd ros_catkin_ws/src && \
