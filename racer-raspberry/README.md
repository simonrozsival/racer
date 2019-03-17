# The Racecar Robot Repository - diploma thesis

This directory contains the source code for the `racer` Raspberry PI computer - this means the code for _the planning algorithm itself_ and also the Docker configuration and additional ROS packages for communication with the `commander` PC.

This guide will describe how to use this repository to install and run all necessary programs in order to use the race car.

## Requirements

- The source code of this repository is intended to run on Raspberry Pi 3 (processor architecture `armv71`)
- User must have privileges to run Docker
- Docker must be installed on the OS
- The YDLIDAR X2 must be connected to the computer and be accessible at `/dev/lidar`
- The Bosch BNO051 with the correct firmwar must be connected to the computer via usb and be accessible at `/dev/ttyACM0`

## Build The Docker Image

Build and run the image from the given Dockerfile like this:

```
(sudo) ./build-run-start.sh <image_name>:<tag> <container_name>
```

Select some tag which is not used by any other container on your local machine. When the container is running(check using `docker ps -l`), start a new bash shell inside the container using:

```
docker exec -it <container_name> bash
```

