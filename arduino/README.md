# Controller

## Installation

First it is necessary to install the ROS library for Arduino with all the necessary message types' header files - this is done dynamically using the `rosserial_arduino` package:

```
sudo apt-get install ros-indigo-rosserial-arduino ros-indigo-rosserial ros-indigo-angles -y
cd ~/Arduino/libraries
rm -rf ros_lib
source ~/jetsoncar/devel/setup.bash
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
```

When the libarry is installed, upload the `steering.ino` program to the Arduino. The default settings assume that the steering servo will be connected to pin 9 and the ESC (which acts like a servo in Traxxas and D-Power RC cars) is connected to pin 10. Change the necessary appropriate `#define` statements to adjust the values to different setups.
