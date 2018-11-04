#!/bin/bash

#
# Install software
#

# Tools
sudo apt install git vim dirmngr

# Docker
curl -fsSL get.docker.com -o get-docker.sh && sh get-docker.sh
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker

# Add repositories for Ubiquity Robotics Packages
sudo sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8
sudo apt update

#
# USB devices
#

## Add USB rules for YDLIDAR
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar-V2.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar-2303.rules

## Add USB rule for Bosch IMU
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="152a", ATTRS{idProduct}=="802a", MODE:="0666", GROUP:="dialout", SYMLINK+="imu"' > /etc/udev/rules.d/bosch-imu.rules

service udev reload
sleep 2
service udev restart

