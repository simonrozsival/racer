# The Racecar Robot Repository - diploma thesis

This directory contains the source code for the `racer` Raspberry PI computer - this means the code for _the planning algorithm itself_ and also the Docker configuration and additional ROS packages for communication with the `commander` PC.

This guide will describe how to use this repository to install and run all necessary programs in order to use the race car.

## Requirements

- The source code of this repository is intended to run on Raspberry Pi 3 (processor architecture `armv71`)
- User must have privileges to run Docker
- Docker must be installed on the OS
- The Scanse Sweep LIDAR must be connected to the computer and be accessible at `/dev/ttyUSB0`
- The Razor IMU 9DOF with the correct firmware (see http://wiki.ros.org/razor\_imu\_9dof) must be connected to the computer via usb and be accessible at `/dev/ttyACM0`

## Build The Docker Image

Build the image from the given Dockerfile like this:

```
(sudo) docker build -t <tag> .
```

Select some tag which is not used by any other container on your local machine. After the image is built, you can run this image with the provided bash script:

```
./start-docker.sh <tag>
```

Replace `<tag>` with the tag of your container.
