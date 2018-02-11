FROM esteve/ros:kinetic-ros-base-armhf

RUN apt-get update && apt-get install -y \
   git \
   net-tools \
   ros-kinetic-razor-imu-9dof

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

ADD . /robot

ENV ROS_IP 192.168.12.147

CMD ["bash"]
