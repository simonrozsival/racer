# setup the ROS enviromnment
source /ros_entrypoint.sh

# create catkin workspace
echo "> create catkin workspace"
mkdir -p /ros_catkin_ws/src
cd /ros_catkin_ws/src
catkin_init_workspace

# copy custom robot package into the workspace
# echo "> copy custom package into the workspace"
# cp -r /robot/src/racer_package .

# setup the Scanse Sweep IMU
git clone https://github.com/scanse/sweep-ros.git
cd sweep-ros
rosdep install -a -y -r
cd ..

# build the workspace
echo "> build the workspace"
cd ..
catkin_make
source devel/setup.bash

# setup the Razor IMU
echo "> setup the IMU"
apt-get install ros-kinetic-razor-imu-9dof -y
roscd razor_imu_9dof/config
cat razor.yaml | sed s/ttyUSB0/imu/ > my_razor.yaml

echo "> ready"
