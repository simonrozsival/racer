# setup the ROS enviromnment
source /ros_entrypoint.sh

# create catkin workspace
echo "> create catkin workspace"
cd /ros_catkin_ws/src
catkin_init_workspace

# install dependencies for the Scanse Sweep LIDAR
cd /ros_catkin_ws/src/sweep-ros
rosdep install -a -y -r --as-root apt:false

# build the workspace
echo "> build the workspace"
cd /ros_catkin_ws
catkin_make
source /ros_catkin_ws/devel/setup.bash

# setup the Razor IMU
echo "> setup the IMU"
roscd razor_imu_9dof/config
cat razor.yaml | sed s/ttyUSB0/imu/ > my_razor.yaml

# setup Hector SLAM
echo "> setup the Hector SLAM"
roscd hector_slam_launch/launch
# todo

echo "> ready"

# echo "> removing self"
# rm -- $0
