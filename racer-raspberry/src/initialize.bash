# setup the ROS enviromnment
source /ros_entrypoint.sh

# create catkin workspace
echo "> create catkin workspace"
cd /ros_catkin_ws/src
catkin_init_workspace

# copy custom robot package into the workspace
# echo "> copy custom package into the workspace"
# cp -r /robot/src/racer_package .

# install dependencies for the Scanse Sweep LIDAR
cd sweep-ros
rosdep install -a -y -r --as-root apt:false

# build the workspace
echo "> build the workspace"
cd ../..
catkin_make
source devel/setup.bash

# setup the Razor IMU
echo "> setup the IMU"
roscd razor_imu_9dof/config
cat razor.yaml | sed s/ttyUSB0/imu/ > my_razor.yaml

echo "> ready"
