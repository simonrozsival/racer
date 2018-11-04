# setup the ROS enviromnment
source /ros_entrypoint.sh

# create catkin workspace
echo "> create catkin workspace"
cd /ros_catkin_ws/src
catkin_init_workspace

# install dependencies for the raspicam
cd /ros_catkin_ws
echo 'yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml' > /etc/ros/rosdep/sources.list.d/30-ubiquity.list
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y --as-root apt:false

# build the workspace
echo "> build the workspace"
cd /ros_catkin_ws
catkin_make
source /ros_catkin_ws/devel/setup.bash

echo "> ready"

echo "> removing self"
rm ./initialize.bash 
