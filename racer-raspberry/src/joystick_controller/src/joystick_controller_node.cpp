#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "JoystickTransformer.h"

#define DRIVING_TOPIC "/racer/driving/commands"
#define JOY_TOPIC "joy"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_controller");
  ros::NodeHandle nh;

  ros::Publisher steering_pub = nh.advertise<geometry_msgs::Twist>(DRIVING_TOPIC, 1, true);
  JoystickTransformer transformer(steering_pub);

  ros::Subscriber joystick_input_sub = nh.subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, &JoystickTransformer::process_joystick_input, &transformer);

  ros::spin();
}
