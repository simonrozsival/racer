#!/bin/bash
xhost +local:
docker run -it --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_GRAPHICSSYSTEM=native \
  -e CONTAINER_NAME=ros-kinetic-dev \
  --workdir=/home/$USER \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  --name=ros-kinetic-dev \
  kinetic:dev
