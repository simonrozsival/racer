#!/bin/bash

# first of all update rosdep
echo "> update rosdep"
rosdep update --as-root apt:false

# prepare catkin workspace
echo "> build the workspace"
cd /ros_catkin_ws/src
catkin_init_workspace

# install sweep dependencies
cd /ros_catkin_ws/src/sweep-ros
rosdep install -a -y -r --as-root apt:false

# build the workspace
cd /ros_catkin_ws
catkin_make
source /ros_catkin_ws/devel/setup.bash

echo "> ready"

