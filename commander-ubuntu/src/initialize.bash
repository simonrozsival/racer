#!/bin/bash

# first of all update rosdep
echo "> update rosdep"
rosdep update --as-root

# build catkin workspace
echo "> build the workspace"
cd /ros_catkin_ws/src
catkin_init_workspace
cd sweep-ros
rosdep install -a -y -r --as-root apt:false
cd ../..
catkin_make
source devel/setup.bash

echo "> ready"

