#/bin/bash

if [ "$#" -ne 2 ]; then
	SCRIPT_NAME=`basename $0`
	echo "Usage: $SCRIPT_NAME <image_name>:<image_tag> <container_name>"
	exit
fi

# setup
IMAGE_NAME=$1
CONTAINER_NAME=$2

# test if the Scanse Sweep Lidar is connected to the correct port
# ???

# test if the Razor IMU 9DoF is connected to the correct port
# ???

# build docker image
echo "> build image $IMAGE_NAME"
docker build -t $IMAGE_NAME ./src

# run the docker with access to the LIDAR
echo
echo "> run the image $IMAGE_NAME as container $CONTAINER_NAME"
docker run \
	--device /dev/ttyUSB0:/dev/sweep \
	--device /dev/ttyACM0:/dev/imu \
	--net=host \
	--publish-all \
	--name $CONTAINER_NAME \
	-i \
	$IMAGE_NAME \
	bash < ./src/initialize.bash

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

