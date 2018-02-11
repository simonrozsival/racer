#/bin/bash

# setup
DOCKER_IMAGE=$1

# test if the Scanse Sweep Lidar is connected to the correct port
# ???

# test if the Razor IMU 9DoF is connected to the correct port
# ???

# run the docker with access to the LIDAR
docker run \
	--device /dev/ttyUSB0:/dev/sweep \
	--device /dev/ttyACM0:/dev/imu \
	--net=host \
	--publish-all \
	-it $DOCKER_IMAGE bash 
