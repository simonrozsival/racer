#/bin/bash

if [ "$#" -ne 2 ]; then
	SCRIPT_NAME=`basename $0`
	echo "Usage: $SCRIPT_NAME <image_name>:<image_tag> <container_name>"
	exit
fi

# setup
IMAGE_NAME=$1
CONTAINER_NAME=$2

# test if the Lidar is connected to the correct port
LIDAR_PORT="/dev/lidar"
ls -l $SWEEP_PORT > /dev/null 2> /dev/null
if [ $? -ne 0 ]
then
	echo "The LIDAR is not connected on $SWEEP_PORT"
	exit 1
fi

# test if the IMU is connected to the correct port
IMU_PORT="/dev/imu"
ls -l $IMU_PORT > /dev/null 2> /dev/null
if [ $? -ne 0 ]
then
	echo "Thie IMU is not connected on $IMU_PORT"
	exit 2
fi

# arduino which controlls the steering
ARDUINO_PORT="/dev/ttyUSB1"
ls -l $ARDUINO_PORT > /dev/null 2> /dev/null
if [ $? -ne 0 ]
then
	echo "Arduino (ESC and servo control) is not connected."
	exit 3
fi

# arduino which collects information from the wheel encoders
ARDUINO_2_PORT="/dev/ttyUSB2"
ls -l $ARDUINO_2_PORT > /dev/null 2> /dev/null
if [ $? -ne 0 ]
then
	echo "Arduino (wheel encoders) is not connected."
	exit 3
fi



# build docker image
echo "> build image $IMAGE_NAME"
docker build -t $IMAGE_NAME ./src

# run the docker with access to the LIDAR
echo "> run the image $IMAGE_NAME as container $CONTAINER_NAME"
docker run \
        --device $IMU_PORT:/dev/imu \
        --device $LIDAR_PORT:/dev/lidar \
	--device $ARDUINO_PORT:/dev/arduino \
	--device $ARDUINO_2_PORT:/dev/arduino_odometry \
	--device /dev/vchiq \
	--net=host \
	--publish-all \
	--name $CONTAINER_NAME \
	-i \
	$IMAGE_NAME \
	bash < ./src/initialize.bash

if [ $? -ne 0 ]
then
	echo "Running docker container failed."
	exit 4
fi

# start the container
echo
echo "> start the container"
docker start $CONTAINER_NAME

echo
echo "> check if the container is running:"
echo "    docker ps -l"

echo
echo "> connect to the container:"
echo "    docker exec -it $CONTAINER_NAME bash'"

