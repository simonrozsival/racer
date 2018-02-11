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
    && rm -rf /var/likb/apt/lists/*

ENV ROS_MASTER_URI http://192.168.1.2:11311
ENV ROS_IP 192.168.1.1

CMD ["bash"]

