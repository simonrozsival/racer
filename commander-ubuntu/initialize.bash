# first of all update rosdep
echo "> update rosdep"
rosdep update

# create catkin workspace
mkdir -p ros_catkin_ws/src
cd ros_catkin_ws/src
catkin_workspace_init
cd ..
catkin make
source devel/setup.bash

# setup the Scanse Sweep LIDAR
echo "> setup the LIDAR"
rm -rf sweep-ros
git clone https://github.com/scanse/sweep-ros.git
cd sweep-ros
rosdep install -a -y -r
cd ..
catkin_make

# setup the Razor IMU
echo "> setup the IMU"
apt-get install ros-kinetic-razor-imu-9dof -y
roscd razor_imu_9dof/config
cat razor.yaml | sed s/ttyUSB0/imu/ > my_razor.yaml

echo "> ready"

