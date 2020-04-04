#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <racer_msgs/State.h>

#include "racer/math.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/motor_model.h"
#include "racer/vehicle_model/steering_servo_model.h"

#include "racer_ros/utils.h"

std::optional<racer::vehicle_configuration> last_known_configuration;

void command_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
}

void state_callback(const racer_msgs::State::ConstPtr &msg)
{
  last_known_configuration = msg_to
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "current_state");
  ros::NodeHandle node("~");

  // load parameters
  std::string command_topic, state_topic;
  node.param<std::string>("command_topic", command_topic, "/racer/commands");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  auto command_sub = node.subscribe<geometry_msgs::Twist>(command_topic, 1, command_callback);
  auto state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, state_callback);

  ros::spin();

  return 0;
}
