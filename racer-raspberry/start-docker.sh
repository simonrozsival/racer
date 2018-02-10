#/bin/bash

# setup
DOCKER_IMAGE=$1

# run the docker with access to the LIDAR
docker run --device /dev/ttyUSB0:/dev/sweep --device /dev/ttyACM0:/dev/imu -it $DOCKER_IMAGE bash 
